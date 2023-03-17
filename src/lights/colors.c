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

#include <skybrush/colors.h>

#include "../utils.h"

const sb_rgb_color_t SB_COLOR_BLACK = { 0, 0, 0 };
const sb_rgb_color_t SB_COLOR_WHITE = { 255, 255, 255 };

/**
 * @brief Decodes a color in RGB565 format
 *
 * @param color  the color in RGB565 format to decode
 * @return the decoded color
 */
sb_rgb_color_t sb_rgb_color_decode_rgb565(uint16_t color)
{
    return sb_rgb_color_make(
        (color & 0xf800) >> 8,
        (color & 0x7e0) >> 3,
        (color & 0x1f) << 3);
}

/**
 * @brief Encodes a color in RGB565 format
 *
 * @param color  the color to encode
 * @return the encoded color
 */
uint16_t sb_rgb_color_encode_rgb565(sb_rgb_color_t color)
{
    uint16_t result = 0;

    result |= ((color.red >> 3) & 0x1f) << 11;
    result |= ((color.green >> 2) & 0x3f) << 5;
    result |= ((color.blue >> 3) & 0x1f);

    return result;
}

/**
 * @brief Determines whether two RGB colors are equal
 *
 * @param first   the first color
 * @param second  the second color
 * @return whether the two colors are equal
 */
sb_bool_t sb_rgb_color_equals(sb_rgb_color_t first, sb_rgb_color_t second)
{
    return first.red == second.red && first.green == second.green && first.blue == second.blue;
}

/**
 * \brief Linearly interpolates between two colors
 *
 * \param  first   the first color
 * \param  second  the second color
 * \param  ratio   the interpolation ratio; zero means the first color, 1 means
 *                 the second color. Values less than zero or greater than 1
 *                 are allowed.
 * \return the interpolated color
 */
sb_rgb_color_t sb_rgb_color_linear_interpolation(
    sb_rgb_color_t first, sb_rgb_color_t second, float ratio)
{
    sb_rgb_color_t result;

    result.red = clamp(first.red + (second.red - first.red) * ratio, 0, 255);
    result.green = clamp(first.green + (second.green - first.green) * ratio, 0, 255);
    result.blue = clamp(first.blue + (second.blue - first.blue) * ratio, 0, 255);

    return result;
}

/**
 * @brief Creates an RGB color struct from its components
 *
 * @param red    the red component
 * @param green  the green component
 * @param blue   the blue component
 * @return the RGB color struct that was created
 */
sb_rgb_color_t sb_rgb_color_make(uint8_t red, uint8_t green, uint8_t blue)
{
    sb_rgb_color_t result;

    result.red = red;
    result.green = green;
    result.blue = blue;

    return result;
}

/**
 * @brief Creates an RGBW color struct from its components
 *
 * @param red    the red component
 * @param green  the green component
 * @param blue   the blue component
 * @param white  the white component
 * @return the RGBW color struct that was created
 */
sb_rgbw_color_t sb_rgbw_color_make(uint8_t red, uint8_t green, uint8_t blue, uint8_t white)
{
    sb_rgbw_color_t result;

    result.red = red;
    result.green = green;
    result.blue = blue;
    result.white = white;

    return result;
}

/**
 * @brief Converts an RGB color to an equivalent RGBW color.
 *
 * @param color  the color to convert
 * @param conv   the conversion method and parameters
 * @return the RGBW color that was created from the RGB color.
 */
sb_rgbw_color_t sb_rgb_color_to_rgbw(sb_rgb_color_t color, sb_rgbw_conversion_t conv)
{
    sb_rgbw_color_t result;
    uint8_t value;

    switch (conv.method) {
    case SB_RGBW_CONVERSION_SUBTRACT_MIN:
        value = color.red;
        if (color.green < value) {
            value = color.green;
        }
        if (color.blue < value) {
            value = color.blue;
        }
        result.red = color.red - value;
        result.green = color.green - value;
        result.blue = color.blue - value;
        result.white = value;
        break;

    case SB_RGBW_CONVERSION_OFF:
    case SB_RGBW_CONVERSION_FIXED_VALUE:
    default:
        result.red = color.red;
        result.green = color.green;
        result.blue = color.blue;
        result.white = conv.method == SB_RGBW_CONVERSION_FIXED_VALUE ? conv.params.fixed_value : 0;
        break;
    }

    return result;
}

/**
 * @brief Determines whether two RGBW colors are equal
 *
 * @param first   the first color
 * @param second  the second color
 * @return whether the two colors are equal
 */
sb_bool_t sb_rgbw_color_equals(sb_rgbw_color_t first, sb_rgbw_color_t second)
{
    return (
        first.red == second.red && first.green == second.green && first.blue == second.blue && first.white == second.white);
}
