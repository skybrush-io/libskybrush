/*
 * This file is part of libskybrush.
 *
 * Copyright 2020-2022 CollMot Robotics Ltd.
 *
 * libskybrush is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * libskybrush is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include <skybrush/error.h>
#include <skybrush/formats/binary.h>
#include <skybrush/utils.h>
#include <string.h>
#include <unistd.h>

/**
 * Common part of the different \c "sb_binary_file_parser_init_*" methods.
 */
static sb_error_t sb_i_binary_file_parser_init_common(sb_binary_file_parser_t* parser);

/**
 * Reads the header of the next block from the file being parsed.
 */
static sb_error_t sb_i_binary_file_read_next_block_header(sb_binary_file_parser_t* parser);

static sb_error_t sb_i_binary_file_get_crc32(sb_binary_file_parser_t* parser, uint32_t* result);
static off_t sb_i_binary_file_get_current_offset(sb_binary_file_parser_t* parser);
static ssize_t sb_i_binary_file_read(sb_binary_file_parser_t* parser, void* buf, size_t nbytes);
static sb_error_t sb_i_binary_file_seek(sb_binary_file_parser_t* parser, off_t offset);

sb_error_t sb_binary_file_parser_init_from_buffer(
    sb_binary_file_parser_t* parser, uint8_t* buf, size_t nbytes)
{
    if (buf == 0) {
        return SB_EINVAL;
    }

    parser->buf = buf;
    parser->buf_ptr = buf;
    parser->buf_end = buf + nbytes;

    parser->fd = -1;

    return sb_i_binary_file_parser_init_common(parser);
}

sb_error_t sb_binary_file_parser_init_from_file(sb_binary_file_parser_t* parser, int fd)
{
    if (fd < 0) {
        return SB_EINVAL;
    }

    parser->buf = 0;
    parser->buf_ptr = 0;
    parser->buf_end = 0;

    parser->fd = fd;

    return sb_i_binary_file_parser_init_common(parser);
}

void sb_binary_file_parser_destroy(sb_binary_file_parser_t* parser)
{
    parser->buf = 0;
    parser->buf_ptr = 0;
    parser->buf_end = 0;

    parser->fd = -1;
}

uint8_t sb_binary_file_parser_get_version(const sb_binary_file_parser_t* parser)
{
    return parser->version;
}

sb_binary_block_t sb_binary_file_get_current_block(const sb_binary_file_parser_t* parser)
{
    return parser->current_block;
}

sb_bool_t sb_binary_file_is_current_block_valid(const sb_binary_file_parser_t* parser)
{
    return parser->current_block.type != SB_BINARY_BLOCK_NONE;
}

sb_error_t sb_binary_file_read_current_block(sb_binary_file_parser_t* parser, uint8_t* buf)
{
    ssize_t bytes_read;

    if (!sb_binary_file_is_current_block_valid(parser)) {
        /* end of file reached */
        return SB_EREAD;
    }

    SB_CHECK(sb_i_binary_file_seek(parser, parser->current_block.start_of_body));

    bytes_read = sb_i_binary_file_read(parser, buf, parser->current_block.length);
    if (bytes_read != parser->current_block.length) {
        /* read failed */
        return SB_EREAD;
    }

    return SB_SUCCESS;
}

sb_error_t sb_binary_file_rewind(sb_binary_file_parser_t* parser)
{
    SB_CHECK(sb_i_binary_file_seek(parser, parser->start_of_first_block));
    SB_CHECK(sb_i_binary_file_read_next_block_header(parser));
    return SB_SUCCESS;
}

sb_error_t sb_binary_file_seek_to_next_block(sb_binary_file_parser_t* parser)
{
    if (!sb_binary_file_is_current_block_valid(parser)) {
        /* end of file reached */
        return SB_EREAD;
    }

    SB_CHECK(sb_i_binary_file_seek(parser,
        parser->current_block.start_of_body + parser->current_block.length));
    SB_CHECK(sb_i_binary_file_read_next_block_header(parser));

    return SB_SUCCESS;
}

sb_error_t sb_binary_file_find_first_block_by_type(
    sb_binary_file_parser_t* parser, sb_binary_block_type_t block_type)
{
    sb_binary_block_t block;

    SB_CHECK(sb_binary_file_rewind(parser));

    while (1) {
        if (!sb_binary_file_is_current_block_valid(parser)) {
            return SB_ENOENT;
        }

        block = sb_binary_file_get_current_block(parser);
        if (block.type == block_type) {
            return SB_SUCCESS;
        }

        SB_CHECK(sb_binary_file_seek_to_next_block(parser));
    }
}

/* ************************************************************************** */

static sb_error_t sb_i_binary_file_parser_init_common(sb_binary_file_parser_t* parser)
{
    char buf[4];
    long int offset;
    uint32_t expected_crc32 = 0, observed_crc32;

    /* read and check the header */
    if (sb_i_binary_file_read(parser, buf, 4) != 4) {
        return SB_EPARSE;
    }

    if (strncmp(buf, "skyb", 4)) {
        return SB_EPARSE;
    }

    /* read and check the version number */
    if (sb_i_binary_file_read(parser, &parser->version, 1) != 1) {
        return SB_EPARSE;
    }

    if (parser->version != 1 && parser->version != 2) {
        return SB_EPARSE;
    }

    /* Version 2 files have 8 "feature bits" coming right after the version
     * number that describe whether certain additional features (checksum etc)
     * are present in the header. Version 1 files are equivalent to version 2
     * files with all feature bits set to zero. */
    if (parser->version == 2) {
        if (sb_i_binary_file_read(parser, &parser->features, 1) != 1) {
            return SB_EPARSE;
        }
    } else {
        parser->features = 0;
    }

    /* Read the CRC32 from the header if it is there */
    if (parser->features & SB_BINARY_FEATURE_CRC32) {
        unsigned char unsigned_buf[4];

        if (sb_i_binary_file_read(parser, unsigned_buf, 4) != 4) {
            return SB_EPARSE;
        }

        expected_crc32 = 0;
        expected_crc32 |= unsigned_buf[3];
        expected_crc32 <<= 8;
        expected_crc32 |= unsigned_buf[2];
        expected_crc32 <<= 8;
        expected_crc32 |= unsigned_buf[1];
        expected_crc32 <<= 8;
        expected_crc32 |= unsigned_buf[0];
    }

    /* Remember where the "real" data starts in the file or buffer */
    offset = sb_i_binary_file_get_current_offset(parser);
    if (offset < 0) {
        return SB_EREAD;
    }
    parser->start_of_first_block = offset;

    /* Validate the CRC32 checksum if we have one */
    if (parser->features & SB_BINARY_FEATURE_CRC32) {
        SB_CHECK(sb_i_binary_file_get_crc32(parser, &observed_crc32));
        if (expected_crc32 != observed_crc32) {
            return SB_ECORRUPTED;
        }
    }

    /* Rewind the file or buffer and read the header of the first block */
    SB_CHECK(sb_binary_file_rewind(parser));

    return SB_SUCCESS;
}

static sb_error_t sb_i_binary_file_read_next_block_header(sb_binary_file_parser_t* parser)
{
    uint8_t type;
    uint8_t length[2];
    long int offset;
    off_t bytes_read;

    bytes_read = sb_i_binary_file_read(parser, &type, 1);

    if (bytes_read < 0) {
        return SB_EREAD;
    } else if (bytes_read == 0) {
        parser->current_block.type = SB_BINARY_BLOCK_NONE;
        parser->current_block.length = 0;
        parser->current_block.start_of_body = 0;
    } else {
        parser->current_block.type = type;

        if (sb_i_binary_file_read(parser, &length, 2) != 2) {
            return SB_EREAD;
        }

        offset = sb_i_binary_file_get_current_offset(parser);
        if (offset < 0) {
            return SB_EREAD;
        }

        parser->current_block.length = length[0] + (length[1] << 8);
        parser->current_block.start_of_body = offset;
    }

    return SB_SUCCESS;
}

/* ************************************************************************** */

/* File operation abstraction layer so we can support in-memory files and
 * file descriptors as well */

static sb_error_t sb_i_binary_file_get_crc32(sb_binary_file_parser_t* parser, uint32_t* result)
{
    off_t original_offset = sb_i_binary_file_get_current_offset(parser);
    uint8_t buf[256];
    uint32_t checksum;
    uint32_t offset;

    SB_CHECK(sb_i_binary_file_seek(parser, 0));

    checksum = 0;
    offset = 0;
    while (1) {
        ssize_t bytes_read = sb_i_binary_file_read(parser, buf, sizeof(buf));
        if (bytes_read < 0) {
            return SB_EREAD;
        }

        /* replace the checksum in the file with zeros. If there is a checksum,
         * it is at bytes 6-10 */
        if (offset == 0 && bytes_read >= 10) {
            buf[6] = buf[7] = buf[8] = buf[9] = 0;
        }

        offset += bytes_read;

        if (bytes_read > 0) {
            checksum = sb_ap_crc32_update(checksum, buf, bytes_read);
        }

        if (bytes_read < sizeof(buf)) {
            /* end of file reached */
            break;
        }
    }

    SB_CHECK(sb_i_binary_file_seek(parser, original_offset));

    if (result) {
        *result = checksum;
    }

    return SB_SUCCESS;
}

static off_t sb_i_binary_file_get_current_offset(sb_binary_file_parser_t* parser)
{
    if (parser->buf) {
        /* Case of in-memory buffer */
        return parser->buf_ptr - parser->buf;
    } else {
        /* Case of file descriptor */
        return lseek(parser->fd, 0, SEEK_CUR);
    }
}

static ssize_t sb_i_binary_file_read(sb_binary_file_parser_t* parser, void* buf, size_t nbytes)
{
    const uint8_t* new_ptr;

    if (parser->buf) {
        size_t to_read;

        /* Reading in-memory file */
        if (parser->buf_ptr < parser->buf) {
            return -1;
        }

        new_ptr = parser->buf_ptr + nbytes;
        if (new_ptr > parser->buf_end) {
            new_ptr = parser->buf_end;
            to_read = new_ptr >= parser->buf_ptr ? new_ptr - parser->buf_ptr : 0;
        } else {
            to_read = nbytes;
        }

        if (to_read > 0) {
            memcpy(buf, parser->buf_ptr, to_read);
        }

        parser->buf_ptr = new_ptr;

        return to_read;
    } else {
        /* Reading file descriptor */
        return read(parser->fd, buf, nbytes);
    }
}

static sb_error_t sb_i_binary_file_seek(sb_binary_file_parser_t* parser, off_t offset)
{
    if (parser->buf) {
        /* Seeking in-memory buffer */
        if (offset < 0 || offset > parser->buf_end - parser->buf) {
            return SB_EREAD;
        }

        parser->buf_ptr = parser->buf + offset;
        return SB_SUCCESS;
    } else {
        /* Seeking file descriptor */
        return lseek(parser->fd, offset, SEEK_SET) == offset ? SB_SUCCESS : SB_EREAD;
    }
}
