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

#ifndef SKYBRUSH_COLORS_H
#define SKYBRUSH_COLORS_H

#include <skybrush/basic_types.h>
#include <skybrush/decls.h>

__BEGIN_DECLS

/**
 * @file colors.h
 * @brief RGB color handling.
 */

/**
 * Typedef for an RGB color.
 */
typedef struct sb_rgb_color_s {
    uint8_t red; /**< The red component of the color */
    uint8_t green; /**< The green component of the color */
    uint8_t blue; /**< The blue component of the color */
} sb_rgb_color_t;

/**
 * Constant for the black color.
 */
extern const sb_rgb_color_t SB_COLOR_BLACK;

/**
 * Constant for the white color.
 */
extern const sb_rgb_color_t SB_COLOR_WHITE;

sb_rgb_color_t sb_rgb_color_decode_rgb565(uint16_t color);
uint16_t sb_rgb_color_encode_rgb565(sb_rgb_color_t color);
sb_bool_t sb_rgb_color_equals(sb_rgb_color_t first, sb_rgb_color_t second);
sb_rgb_color_t sb_rgb_color_linear_interpolation(
    sb_rgb_color_t first, sb_rgb_color_t second, float ratio);
sb_rgb_color_t sb_rgb_color_make(uint8_t red, uint8_t green, uint8_t blue);

__END_DECLS

#endif
