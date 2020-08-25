#include <string.h>
#include <unistd.h>
#include <skybrush/error.h>
#include <skybrush/formats/binary.h>

/**
 * Common part of the different \c "sb_binary_file_parser_init_*" methods.
 */
static sb_error_t sb_i_binary_file_parser_init_common(sb_binary_file_parser_t *parser);

/**
 * Reads the header of the next block from the file being parsed.
 */
static sb_error_t sb_i_binary_file_read_next_block_header(sb_binary_file_parser_t *parser);

static off_t sb_i_binary_file_get_current_offset(sb_binary_file_parser_t *parser);
static ssize_t sb_i_binary_file_read(sb_binary_file_parser_t *parser, void *buf, size_t nbytes);
static sb_error_t sb_i_binary_file_seek(sb_binary_file_parser_t *parser, off_t offset);

sb_error_t sb_binary_file_parser_init_from_buffer(
    sb_binary_file_parser_t *parser, uint8_t *buf, size_t nbytes)
{
    if (buf == 0)
    {
        return SB_EINVAL;
    }

    parser->buf = buf;
    parser->buf_ptr = buf;
    parser->buf_end = buf + nbytes;

    parser->fd = -1;

    return sb_i_binary_file_parser_init_common(parser);
}

sb_error_t sb_binary_file_parser_init_from_file(sb_binary_file_parser_t *parser, int fd)
{
    if (fd < 0)
    {
        return SB_EINVAL;
    }

    parser->buf = 0;
    parser->buf_ptr = 0;
    parser->buf_end = 0;

    parser->fd = fd;

    return sb_i_binary_file_parser_init_common(parser);
}

void sb_binary_file_parser_destroy(sb_binary_file_parser_t *parser)
{
    parser->buf = 0;
    parser->buf_ptr = 0;
    parser->buf_end = 0;

    parser->fd = -1;
}

uint8_t sb_binary_file_parser_get_version(const sb_binary_file_parser_t *parser)
{
    return parser->version;
}

sb_binary_block_t sb_binary_file_get_current_block(const sb_binary_file_parser_t *parser)
{
    return parser->current_block;
}

sb_bool_t sb_binary_file_is_current_block_valid(const sb_binary_file_parser_t *parser)
{
    return parser->current_block.type != SB_BINARY_BLOCK_NONE;
}

sb_error_t sb_binary_file_read_current_block(sb_binary_file_parser_t *parser, uint8_t *buf)
{
    ssize_t bytes_read;

    if (!sb_binary_file_is_current_block_valid(parser))
    {
        /* end of file reached */
        return SB_EREAD;
    }

    SB_CHECK(sb_i_binary_file_seek(parser, parser->current_block.start_of_body));

    bytes_read = sb_i_binary_file_read(parser, buf, parser->current_block.length);
    if (bytes_read != parser->current_block.length)
    {
        /* read failed */
        return SB_EREAD;
    }

    return SB_SUCCESS;
}

sb_error_t sb_binary_file_rewind(sb_binary_file_parser_t *parser)
{
    SB_CHECK(sb_i_binary_file_seek(parser, parser->start_of_first_block));
    SB_CHECK(sb_i_binary_file_read_next_block_header(parser));
    return SB_SUCCESS;
}

sb_error_t sb_binary_file_seek_to_next_block(sb_binary_file_parser_t *parser)
{
    if (!sb_binary_file_is_current_block_valid(parser))
    {
        /* end of file reached */
        return SB_EREAD;
    }

    SB_CHECK(sb_i_binary_file_seek(parser,
                                   parser->current_block.start_of_body + parser->current_block.length));
    SB_CHECK(sb_i_binary_file_read_next_block_header(parser));

    return SB_SUCCESS;
}

sb_error_t sb_binary_file_find_first_block_by_type(
    sb_binary_file_parser_t *parser, sb_binary_block_type_t block_type)
{
    sb_binary_block_t block;

    SB_CHECK(sb_binary_file_rewind(parser));

    while (1)
    {
        if (!sb_binary_file_is_current_block_valid(parser))
        {
            return SB_FAILURE;
        }

        block = sb_binary_file_get_current_block(parser);
        if (block.type == block_type)
        {
            return SB_SUCCESS;
        }

        SB_CHECK(sb_binary_file_seek_to_next_block(parser));
    }
}

/* ************************************************************************** */

static sb_error_t sb_i_binary_file_parser_init_common(sb_binary_file_parser_t *parser)
{
    char buf[4];
    long int offset;

    /* read and check the header */
    if (sb_i_binary_file_read(parser, buf, 4) != 4)
    {
        return SB_EPARSE;
    }

    if (strncmp(buf, "skyb", 4))
    {
        return SB_EPARSE;
    }

    /* read and check the version number */
    if (sb_i_binary_file_read(parser, &parser->version, 1) != 1)
    {
        return SB_EPARSE;
    }

    if (parser->version != 1)
    {
        return SB_EPARSE;
    }

    offset = sb_i_binary_file_get_current_offset(parser);
    if (offset < 0)
    {
        return SB_EREAD;
    }
    parser->start_of_first_block = offset;

    SB_CHECK(sb_binary_file_rewind(parser));

    return SB_SUCCESS;
}

static sb_error_t sb_i_binary_file_read_next_block_header(sb_binary_file_parser_t *parser)
{
    uint8_t type;
    uint8_t length[2];
    long int offset;
    off_t bytes_read;

    bytes_read = sb_i_binary_file_read(parser, &type, 1);

    if (bytes_read < 0)
    {
        return SB_EREAD;
    }
    else if (bytes_read == 0)
    {
        parser->current_block.type = SB_BINARY_BLOCK_NONE;
        parser->current_block.length = 0;
        parser->current_block.start_of_body = 0;
    }
    else
    {
        parser->current_block.type = type;

        if (sb_i_binary_file_read(parser, &length, 2) != 2)
        {
            return SB_EREAD;
        }

        offset = sb_i_binary_file_get_current_offset(parser);
        if (offset < 0)
        {
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

static off_t sb_i_binary_file_get_current_offset(sb_binary_file_parser_t *parser)
{
    if (parser->buf)
    {
        /* Case of in-memory buffer */
        return parser->buf_ptr - parser->buf;
    }
    else
    {
        /* Case of file descriptor */
        return lseek(parser->fd, 0, SEEK_CUR);
    }
}

static ssize_t sb_i_binary_file_read(sb_binary_file_parser_t *parser, void *buf, size_t nbytes)
{
    const uint8_t *new_ptr;
    size_t to_read;

    if (parser->buf)
    {
        /* Reading in-memory file */
        if (parser->buf_ptr < parser->buf)
        {
            return -1;
        }

        new_ptr = parser->buf_ptr + nbytes;
        if (new_ptr > parser->buf_end)
        {
            new_ptr = parser->buf_end;
            to_read = new_ptr >= parser->buf_ptr ? new_ptr - parser->buf_ptr : 0;
        }
        else
        {
            to_read = nbytes;
        }

        if (to_read > 0)
        {
            memcpy(buf, parser->buf_ptr, to_read);
        }

        parser->buf_ptr = new_ptr;

        return to_read;
    }
    else
    {
        /* Reading file descriptor */
        return read(parser->fd, buf, nbytes);
    }
}

static sb_error_t sb_i_binary_file_seek(sb_binary_file_parser_t *parser, off_t offset)
{
    if (parser->buf)
    {
        /* Seeking in-memory buffer */
        if (offset < 0 || offset > parser->buf_end - parser->buf)
        {
            return SB_EREAD;
        }

        parser->buf_ptr = parser->buf + offset;
        return SB_SUCCESS;
    }
    else
    {
        /* Seeking file descriptor */
        return lseek(parser->fd, offset, SEEK_SET) == offset ? SB_SUCCESS : SB_EREAD;
    }
}
