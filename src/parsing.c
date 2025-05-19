/*
 * This file is part of libskybrush.
 *
 * Copyright 2020-2025 CollMot Robotics Ltd.
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

#include "parsing.h"

/**
 * Parses an unsigned 8-bit integer from a buffer.
 *
 * The offset is automatically advanced after reading the integer.
 */
uint8_t sb_parse_uint8(const uint8_t* buf, size_t* offset)
{
    uint8_t result = buf[*offset];
    *offset += 1;
    return result;
}

/**
 * Parses a signed 16-bit little-endian integer from a buffer.
 *
 * The offset is automatically advanced after reading the integer.
 */
int16_t sb_parse_int16(const uint8_t* buf, size_t* offset)
{
    return (int16_t)(sb_parse_uint16(buf, offset));
}

/**
 * Parses a signed 32-bit little-endian integer from a buffer.
 *
 * The offset is automatically advanced after reading the integer.
 */
int32_t sb_parse_int32(const uint8_t* buf, size_t* offset)
{
    return (int32_t)(sb_parse_uint32(buf, offset));
}

/**
 * Parses an unsigned 16-bit little-endian integer from a buffer.
 *
 * The offset is automatically advanced after reading the integer.
 */
uint16_t sb_parse_uint16(const uint8_t* buf, size_t* offset)
{
    uint32_t result;

    result = buf[*offset + 1];
    result = (result << 8) + buf[*offset];

    *offset += 2;

    return result;
}

/**
 * Parses an unsigned 32-bit little-endian integer from a buffer.
 *
 * The offset is automatically advanced after reading the integer.
 */
uint32_t sb_parse_uint32(const uint8_t* buf, size_t* offset)
{
    uint32_t result;

    result = buf[*offset + 3];
    result = (result << 8) + buf[*offset + 2];
    result = (result << 8) + buf[*offset + 1];
    result = (result << 8) + buf[*offset];

    *offset += 4;

    return result;
}

/**
 * Parses an unsigned 32-bit integer from a buffer, encoded as a variable-length
 * integers.
 *
 * Variable length integers are encoded incrementally; first we copy the
 * seven least significant bits of the unsigned integer to a byte and set the
 * MSB of the byte to 1 if and only if there are more non-zero bits in the
 * integer. Then we shift the unsigned integer down by 7 bits and repeat.
 *
 * The offset is automatically advanced after reading the integer.
 *
 * @param  buf     the buffer that contains the integer to parse
 * @param  num_bytes  the number of bytes in the buffer
 * @param  offset  the offset into the buffer; it will be advanced automatically
 *                 after reading the integer
 * @param  result  the parsed integer will be returned here
 * @return \c SB_SUCCESS if the parsing was successful, \c SB_OVERFLOW when the
 *         parsed number does not fit into a 32-bit unsigned integer
 */
sb_error_t sb_parse_varuint32(const uint8_t* buf, const size_t num_bytes, size_t* offset, uint32_t* result)
{
    uint32_t value = 0;
    uint8_t byte;
    uint8_t num_bits = 0;
    uint8_t bits_left = 32;

    while (1) {
        if (*offset >= num_bytes) {
            /* End of buffer reached */
            return SB_EPARSE;
        }

        byte = buf[*offset];
        (*offset)++;

        if (bits_left < 7 && (byte >> bits_left) > 0) {
            break;
        }

        value = value + (((uint32_t)(byte & 0x7f)) << num_bits);
        if (!(byte & 0x80)) {
            /* Normal exit */
            *result = value;
            return SB_SUCCESS;
        }

        num_bits += 7;
        bits_left -= 7;
        if (num_bits > 31) {
            break;
        }
    }

    /* If we are here, it means that there was an overflow */
    while (1) {
        if (!(byte & 0x80)) {
            return SB_EOVERFLOW;
        }

        if (*offset >= num_bytes) {
            /* End of buffer reached */
            return SB_EPARSE;
        }

        byte = buf[*offset];
        (*offset)++;
    }
}

/**
 * Writes a signed 16-bit little-endian integer to a buffer.
 *
 * The offset is automatically advanced after writing the integer.
 */
void sb_write_int16(uint8_t* buf, size_t* offset, int16_t value)
{
    sb_write_uint16(buf, offset, value);
}

/**
 * Writes a signed 32-bit little-endian integer to a buffer.
 *
 * The offset is automatically advanced after writing the integer.
 */
void sb_write_int32(uint8_t* buf, size_t* offset, int32_t value)
{
    sb_write_uint32(buf, offset, value);
}

/**
 * Writes an unsigned 16-bit little-endian integer to a buffer.
 *
 * The offset is automatically advanced after writing the integer.
 */
void sb_write_uint16(uint8_t* buf, size_t* offset, uint16_t value)
{
    buf[*offset] = value & 0xff;
    buf[(*offset) + 1] = value >> 8;
    *offset += 2;
}

/**
 * Writes an unsigned 16-bit little-endian integer to a buffer.
 *
 * The offset is automatically advanced after writing the integer.
 */
void sb_write_uint32(uint8_t* buf, size_t* offset, uint32_t value)
{
    buf[*offset] = value & 0xff;
    value >>= 8;

    buf[(*offset) + 1] = value & 0xff;
    value >>= 8;

    buf[(*offset) + 2] = value & 0xff;
    value >>= 8;

    buf[(*offset) + 3] = value & 0xff;

    *offset += 4;
}
