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

#include <skybrush/formats/tlv.h>

#include "../parsing.h"

/**
 * \brief Number of bytes in the header of a single TLV entry (one byte for
 * the tag and two bytes for the length).
 */
#define SB_I_TLV_HEADER_LENGTH 3

sb_error_t sb_tlv_parser_init(sb_tlv_parser_t* parser, const uint8_t* buf, size_t size)
{
    if (parser == 0) {
        return SB_EINVAL;
    }

    if (buf == 0 && size > 0) {
        return SB_EINVAL;
    }

    parser->buf = buf;
    parser->size = size;
    parser->offset = 0;

    return SB_SUCCESS;
}

sb_error_t sb_tlv_parser_next(sb_tlv_parser_t* parser, sb_tlv_entry_t* entry)
{
    size_t offset;
    uint8_t tag;
    uint16_t length;

    if (parser == 0 || entry == 0) {
        return SB_EINVAL;
    }

    if (parser->buf == 0 && parser->size > 0) {
        /* parser was not initialized properly (or at all) */
        return SB_EINVAL;
    }

    offset = parser->offset;

    if (offset >= parser->size) {
        /* end of stream reached */
        return SB_ENOENT;
    }

    if (parser->size - offset < SB_I_TLV_HEADER_LENGTH) {
        /* the header of the entry is truncated */
        return SB_ECORRUPTED;
    }

    tag = sb_parse_uint8(parser->buf, &offset);
    length = sb_parse_uint16(parser->buf, &offset);

    if (parser->size - offset < (size_t)length) {
        /* the value of the entry would extend past the end of the buffer */
        return SB_ECORRUPTED;
    }

    entry->tag = tag;
    entry->value = parser->buf + offset;
    entry->length = length;

    parser->offset = offset + length;

    return SB_SUCCESS;
}

void sb_tlv_parser_destroy(sb_tlv_parser_t* parser)
{
    if (parser == 0) {
        return;
    }

    parser->buf = 0;
    parser->size = 0;
    parser->offset = 0;
}
