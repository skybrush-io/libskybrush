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

#include "./utils.h"

#include "../parsing.h"

/**
 * @brief Returns the number of expected coordinates given the header bits.
 */
uint8_t sb_i_get_num_coords(uint8_t header_bits)
{
    return 1 << (header_bits & 0x03);
}

/**
 * Parses an angle from a memory block.
 *
 * The offset is automatically advanced after reading the angle.
 */
float sb_i_parse_angle(const uint8_t* buffer, size_t* offset)
{
    int16_t angle = sb_parse_int16(buffer, offset) % 3600;

    if (angle < 0) {
        angle += 3600;
    }

    return angle / 10.0f;
}

/**
 * Parses a coordinate from a memory block, scaling it up with the given
 * scaling factor as needed.
 *
 * The offset is automatically advanced after reading the coordinate.
 */

float sb_i_parse_coordinate(const uint8_t* buffer, size_t* offset, float scale)
{
    return sb_parse_int16(buffer, offset) * scale;
}
