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
 * Typedef for an RGBW color.
 *
 * The library primarily works in RGB and most of the functions support RGB
 * only. There are special conversion functions that take an existing RGB
 * color and returns an equivalent RGBW color.
 */
typedef struct sb_rgbw_color_s {
    uint8_t red; /**< The red component of the color */
    uint8_t green; /**< The green component of the color */
    uint8_t blue; /**< The blue component of the color */
    uint8_t white; /**< The white component of the color */
} sb_rgbw_color_t;

/**
 * Supported methods for converting an RGB color to an RGBW color.
 */
typedef enum {
    SB_RGBW_CONVERSION_FIXED_VALUE,
    SB_RGBW_CONVERSION_SUBTRACT_MIN,
    SB_RGBW_CONVERSION_USE_REFERENCE
} sb_rgbw_conversion_method_t;

/**
 * Structure to define the full parameter set of an RGB-to-RGBW conversion.
 */
typedef struct {
    sb_rgbw_conversion_method_t method;
    float temperature;
    union {
        uint8_t fixed_value;
        struct {
            float mul[3];
            float div[3];
            float temperature;
        } color_ref;
    } params;
} sb_rgbw_conversion_t;

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
sb_bool_t sb_rgb_color_almost_equals(sb_rgb_color_t first, sb_rgb_color_t second, uint8_t eps);
sb_rgb_color_t sb_rgb_color_linear_interpolation(
    sb_rgb_color_t first, sb_rgb_color_t second, float ratio);
sb_rgb_color_t sb_rgb_color_make(uint8_t red, uint8_t green, uint8_t blue);
sb_rgbw_color_t sb_rgb_color_to_rgbw(sb_rgb_color_t color, sb_rgbw_conversion_t conv);
sb_rgb_color_t sb_rgb_color_from_color_temperature(float temperature);

sb_bool_t sb_rgbw_color_equals(sb_rgbw_color_t first, sb_rgbw_color_t second);
sb_bool_t sb_rgbw_color_almost_equals(sb_rgbw_color_t first, sb_rgbw_color_t second, uint8_t eps);
sb_rgbw_color_t sb_rgbw_color_make(uint8_t red, uint8_t green, uint8_t blue, uint8_t white);

void sb_rgbw_conversion_turn_off(sb_rgbw_conversion_t* conv);
void sb_rgbw_conversion_use_fixed_value(sb_rgbw_conversion_t* conv, uint8_t value);
void sb_rgbw_conversion_use_min_subtraction(sb_rgbw_conversion_t* conv);
void sb_rgbw_conversion_use_color_temperature(sb_rgbw_conversion_t* conv, float temperature);
void sb_rgbw_conversion_use_reference_color(sb_rgbw_conversion_t* conv, sb_rgb_color_t reference);

__END_DECLS

#endif
