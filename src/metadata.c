/*
 * This file is part of libskybrush.
 *
 * Copyright 2020-2026 CollMot Robotics Ltd.
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

/**
 * @file metadata.c
 * @brief Parsing of the show metadata block of the Skybrush binary show
 *        file format.
 */

#include <string.h>

#include <skybrush/formats/binary.h>
#include <skybrush/formats/tlv.h>
#include <skybrush/memory.h>
#include <skybrush/metadata.h>

#include "parsing.h"

/**
 * \brief Length of the value of the tag that holds the index of the drone
 *        within the show, in bytes.
 */
#define SB_I_SHOW_METADATA_DRONE_INDEX_LENGTH sizeof(uint16_t)

/**
 * \brief Result of parsing the body of a show metadata block.
 *
 * The struct holds the last occurrence of each known tag; tags that were
 * not present in the block at all are left at their default values.
 */
typedef struct
{
    uint8_t show_id[SB_SHOW_METADATA_SHOW_ID_LENGTH]; /**< The last unique show ID found in the block */
    uint16_t drone_index; /**< The last drone index found in the block */
} sb_i_show_metadata_parse_result_t;

static sb_error_t sb_i_show_metadata_parse(
    const uint8_t* buf, size_t size,
    sb_i_show_metadata_parse_result_t* result);

static sb_error_t sb_i_show_metadata_update_from_buffer(
    sb_show_metadata_t* metadata, const uint8_t* buf, size_t size);

static sb_error_t sb_i_show_metadata_update_from_parser(
    sb_show_metadata_t* metadata, sb_binary_file_parser_t* parser);

sb_error_t sb_show_metadata_init(sb_show_metadata_t* metadata)
{
    if (metadata == 0) {
        return SB_EINVAL;
    }

    memset(metadata->show_id, 0, sizeof(metadata->show_id));
    metadata->drone_index = 0;

    return SB_SUCCESS;
}

sb_error_t sb_show_metadata_clear(sb_show_metadata_t* metadata)
{
    return sb_show_metadata_init(metadata);
}

void sb_show_metadata_destroy(sb_show_metadata_t* metadata)
{
    if (metadata == 0) {
        return;
    }

    memset(metadata, 0, sizeof(sb_show_metadata_t));
}

sb_error_t sb_show_metadata_get_show_id(const sb_show_metadata_t* metadata, uint8_t* show_id)
{
    if (metadata == 0 || show_id == 0) {
        return SB_EINVAL;
    }

    memcpy(show_id, metadata->show_id, sizeof(metadata->show_id));

    return SB_SUCCESS;
}

sb_error_t sb_show_metadata_get_drone_index(const sb_show_metadata_t* metadata, uint16_t* drone_index)
{
    if (metadata == 0 || drone_index == 0) {
        return SB_EINVAL;
    }

    *drone_index = metadata->drone_index;

    return SB_SUCCESS;
}

sb_error_t sb_show_metadata_update_from_buffer(
    sb_show_metadata_t* metadata, const uint8_t* buf, size_t size)
{
    return sb_i_show_metadata_update_from_buffer(metadata, buf, size);
}

sb_error_t sb_show_metadata_update_from_binary_file(sb_show_metadata_t* metadata, int fd)
{
    sb_binary_file_parser_t parser;
    sb_error_t retval;

    SB_CHECK(sb_binary_file_parser_init_from_file(&parser, fd));
    retval = sb_i_show_metadata_update_from_parser(metadata, &parser);
    sb_binary_file_parser_destroy(&parser);

    return retval;
}

sb_error_t sb_show_metadata_update_from_binary_file_in_memory(
    sb_show_metadata_t* metadata, uint8_t* buf, size_t length)
{
    sb_binary_file_parser_t parser;
    sb_error_t retval;

    SB_CHECK(sb_binary_file_parser_init_from_buffer(&parser, buf, length));
    retval = sb_i_show_metadata_update_from_parser(metadata, &parser);
    sb_binary_file_parser_destroy(&parser);

    return retval;
}

/**
 * Parses the body of a show metadata block into a parse result, without
 * touching the show metadata object itself.
 *
 * Unknown tags are skipped. When the same tag appears multiple times, the
 * last occurrence wins.
 */
static sb_error_t sb_i_show_metadata_parse(
    const uint8_t* buf, size_t size,
    sb_i_show_metadata_parse_result_t* result)
{
    sb_tlv_parser_t parser;
    sb_tlv_entry_t entry;
    sb_error_t retval;

    memset(result, 0, sizeof(sb_i_show_metadata_parse_result_t));

    SB_CHECK(sb_tlv_parser_init(&parser, buf, size));

    while (1) {
        retval = sb_tlv_parser_next(&parser, &entry);
        if (retval == SB_ENOENT) {
            /* end of the tag-length-value stream */
            break;
        }
        SB_CHECK(retval);

        switch (entry.tag) {
        case SB_SHOW_METADATA_TAG_SHOW_ID:
            if (entry.length != SB_SHOW_METADATA_SHOW_ID_LENGTH) {
                return SB_ECORRUPTED;
            }
            memcpy(result->show_id, entry.value, sizeof(result->show_id));
            break;

        case SB_SHOW_METADATA_TAG_DRONE_INDEX: {
            size_t offset = 0;

            if (entry.length != SB_I_SHOW_METADATA_DRONE_INDEX_LENGTH) {
                return SB_ECORRUPTED;
            }
            result->drone_index = sb_parse_uint16(entry.value, &offset);
            break;
        }

        default:
            /* unknown tags are skipped so that the block can be extended
             * with new tags in the future without breaking older parsers */
            break;
        }
    }

    return SB_SUCCESS;
}

/**
 * Common part of the \c sb_show_metadata_update_from_* functions.
 *
 * Parses the given body of a show metadata block and applies the parsed
 * values to the show metadata object. The body is not retained by the
 * object, so it may be released as soon as this function returns.
 */
static sb_error_t sb_i_show_metadata_update_from_buffer(
    sb_show_metadata_t* metadata, const uint8_t* buf, size_t size)
{
    sb_i_show_metadata_parse_result_t result;

    if (metadata == 0) {
        return SB_EINVAL;
    }

    SB_CHECK(sb_i_show_metadata_parse(buf, size, &result));

    /* the body of the block is copied into the object in full so that
     * missing tags yield their default values */
    memcpy(metadata->show_id, result.show_id, sizeof(metadata->show_id));
    metadata->drone_index = result.drone_index;

    return SB_SUCCESS;
}

/**
 * Common part of the functions that update a show metadata object from a
 * Skybrush binary show file parser.
 */
static sb_error_t sb_i_show_metadata_update_from_parser(
    sb_show_metadata_t* metadata, sb_binary_file_parser_t* parser)
{
    sb_error_t retval;
    uint8_t* buf;
    size_t size;
    sb_bool_t owned;

    SB_CHECK(sb_binary_file_find_first_block_by_type(parser, SB_BINARY_BLOCK_SHOW_METADATA));
    SB_CHECK(sb_binary_file_read_current_block_ex(parser, &buf, &size, &owned));

    /* the body of the block does not outlive this function because the
     * metadata object copies the few values that it cares about */
    retval = sb_i_show_metadata_update_from_buffer(metadata, buf, size);

    if (owned) {
        sb_free(buf);
    }

    return retval;
}
