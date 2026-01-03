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

#ifndef SKYBRUSH_BASIC_TYPES_H
#define SKYBRUSH_BASIC_TYPES_H

#include <skybrush/decls.h>

#include <stdint.h>

__BEGIN_DECLS

/**
 * @file basic_types.h
 * @brief Basic data types used throughout the library
 */

/**
 * Type alias for booleans.
 */
typedef uint8_t sb_bool_t;

/**
 * Lower and upper limits of some quantity.
 */
typedef struct
{
    /** The lower limit, inclusive. */
    float min;

    /** The upper limit, inclusive. */
    float max;
} sb_interval_t;

/**
 * A simple 2D vector.
 */
typedef struct
{
    /** The X coordinate of the vector */
    float x;

    /** The Y coordinate of the vector */
    float y;
} sb_vector2_t;

/**
 * A simple 3D vector.
 */
typedef struct
{
    /** The X coordinate of the vector */
    float x;

    /** The Y coordinate of the vector */
    float y;

    /** The Z coordinate of the vector */
    float z;
} sb_vector3_t;

/**
 * A simple 3D vector with an extra yaw component.
 */
typedef struct
{
    /** The X coordinate of the vector */
    float x;

    /** The Y coordinate of the vector */
    float y;

    /** The Z coordinate of the vector */
    float z;

    /** The yaw angle corresponding to the vector */
    float yaw;
} sb_vector3_with_yaw_t;

/**
 * Bounding box of a set of 3D points.
 */
typedef struct
{
    /** The minimum and maximum X coordinates of the bounding box */
    sb_interval_t x;

    /** The minimum and maximum Y coordinates of the bounding box */
    sb_interval_t y;

    /** The minimum and maximum Z coordinates of the bounding box */
    sb_interval_t z;
} sb_bounding_box_t;

__END_DECLS

#endif
