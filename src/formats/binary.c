#include <string.h>
#include <skybrush/error.h>
#include <skybrush/formats/binary.h>

/**
 * Reads the header of the next block from the file being parsed.
 */
static sb_error_t sb_i_binary_file_read_next_block_header(sb_binary_file_parser_t *parser);

sb_error_t sb_binary_file_parser_init(sb_binary_file_parser_t *parser, FILE *fp)
{
    char buf[4];
    long int offset;

    parser->fp = fp;

    /* read and check the header */
    if (fread(buf, sizeof(char), 4, parser->fp) != 4)
    {
        return SB_EPARSE;
    }

    if (strncmp(buf, "skyb", 4))
    {
        return SB_EPARSE;
    }

    /* read and check the version number */
    if (fread(&parser->version, sizeof(uint8_t), 1, parser->fp) != 1)
    {
        return SB_EPARSE;
    }

    if (parser->version != 1)
    {
        return SB_EPARSE;
    }

    offset = ftell(parser->fp);
    if (offset < 0)
    {
        return SB_EREAD;
    }
    parser->start_of_first_block = offset;

    SB_CHECK(sb_binary_file_rewind(parser));

    return SB_SUCCESS;
}

void sb_binary_file_parser_destroy(sb_binary_file_parser_t *parser)
{
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
    if (!sb_binary_file_is_current_block_valid(parser))
    {
        /* end of file reached */
        return SB_EREAD;
    }

    if (fseek(parser->fp, parser->current_block.start_of_body, SEEK_SET))
    {
        /* seek failed */
        return SB_EREAD;
    }

    if (fread(buf, sizeof(uint8_t), parser->current_block.length, parser->fp) != parser->current_block.length)
    {
        /* read failed */
        return SB_EREAD;
    }

    return SB_SUCCESS;
}

sb_error_t sb_binary_file_rewind(sb_binary_file_parser_t *parser)
{
    if (fseek(parser->fp, parser->start_of_first_block, SEEK_SET))
    {
        /* seek failed */
        return SB_EREAD;
    }

    /* read the next block */
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

    if (fseek(parser->fp, parser->current_block.start_of_body + parser->current_block.length, SEEK_SET))
    {
        /* seek failed */
        return SB_EREAD;
    }

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

static sb_error_t sb_i_binary_file_read_next_block_header(sb_binary_file_parser_t *parser)
{
    uint8_t type;
    uint8_t length[2];
    long int offset;

    if (fread(&type, sizeof(uint8_t), 1, parser->fp) != 1)
    {
        if (feof(parser->fp))
        {
            parser->current_block.type = SB_BINARY_BLOCK_NONE;
            parser->current_block.length = 0;
            parser->current_block.start_of_body = 0;
        }
        else
        {
            return SB_EREAD;
        }
    }
    else
    {
        parser->current_block.type = type;

        if (fread(&length, sizeof(uint8_t), 2, parser->fp) != 2)
        {
            return SB_EREAD;
        }

        offset = ftell(parser->fp);
        if (offset < 0)
        {
            return SB_EREAD;
        }

        parser->current_block.length = length[0] + (length[1] << 8);
        parser->current_block.start_of_body = offset;
    }

    return SB_SUCCESS;
}
