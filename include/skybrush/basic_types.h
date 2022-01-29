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

#ifndef SKYBRUSH_BASIC_TYPES_H
#define SKYBRUSH_BASIC_TYPES_H

#include <skybrush/decls.h>

#include <stdint.h>

__BEGIN_DECLS

/**
 * Type alias for booleans.
 */
typedef uint8_t sb_bool_t;

/**
 * Lower and upper limits of some quantity.
 */
typedef struct
{
    float min;
    float max;
} sb_interval_t;

/**
 * A simple 3D vector with an extra yaw component.
 */
typedef struct
{
    float x;
    float y;
    float z;
    float yaw;
} sb_vector3_with_yaw_t;

/**
 * Bounding box of a set of 3D points.
 */
typedef struct
{
    sb_interval_t x;
    sb_interval_t y;
    sb_interval_t z;
} sb_bounding_box_t;

__END_DECLS

#endif
