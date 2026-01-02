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

#ifndef SKYBRUSH_I_TRAJECTORY_UTILS_H
#define SKYBRUSH_I_TRAJECTORY_UTILS_H

#include <skybrush/decls.h>
#include <stddef.h>
#include <stdint.h>

__BEGIN_DECLS

uint8_t sb_i_get_num_coords(uint8_t header_bits);
float sb_i_parse_angle(const uint8_t* buf, size_t* offset);
float sb_i_parse_coordinate(const uint8_t* buf, size_t* offset, float scale);

__END_DECLS

#endif
