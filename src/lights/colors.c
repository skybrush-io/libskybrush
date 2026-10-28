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

#include <math.h>
#include <skybrush/colors.h>
#include <stdlib.h>

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
 * @brief Determines whether two RGB colors are equal with a tolerance
 *
 * @param first   the first color
 * @param second  the second color
 * @param eps     the tolerance level, i.e. the maximum allowed difference
 *        between the individual components of the two colors
 * @return whether the two colors are equal (with tolerance)
 */
sb_bool_t sb_rgb_color_almost_equals(sb_rgb_color_t first, sb_rgb_color_t second, uint8_t eps)
{
    return (
        abs(first.red - second.red) <= eps && abs(first.green - second.green) <= eps && abs(first.blue - second.blue) <= eps);
}

/**
 * Converts a float to a color component, truncating it towards zero. Values
 * below 0 or above 255 saturate, and NaN becomes zero; converting these to an
 * integer type directly would be undefined behaviour.
 */
static uint8_t sb_i_color_component(float value)
{
    /* Written so that NaN becomes zero as well */
    if (!(value > 0)) {
        return 0;
    }
    return value > 255 ? 255 : (uint8_t)value;
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

    result.red = sb_i_color_component(first.red + (second.red - first.red) * ratio);
    result.green = sb_i_color_component(first.green + (second.green - first.green) * ratio);
    result.blue = sb_i_color_component(first.blue + (second.blue - first.blue) * ratio);

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

    switch (conv.method) {
    case SB_RGBW_CONVERSION_SUBTRACT_MIN: {
        uint8_t value;
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
    }

    case SB_RGBW_CONVERSION_FIXED_VALUE:
    default:
        result.red = color.red;
        result.green = color.green;
        result.blue = color.blue;
        result.white = conv.params.fixed_value;
        break;

    case SB_RGBW_CONVERSION_USE_REFERENCE: {
        const uint8_t values[3] = { color.red, color.green, color.blue };
        uint8_t* results[3] = { &result.red, &result.green, &result.blue };
        float scaled;
        float min_scaled = INFINITY;
        float correction;
        uint8_t i;

        /* Code adapted from: https://www.dmurph.com/posts/2021/1/cabinet-light-3.html
         *
         * Each channel that the white LED emits limits the white level to the
         * amount of white that the channel can account for. A channel with a
         * multiplier of zero is not emitted by the white LED and does not
         * limit it. */
        for (i = 0; i < 3; i++) {
            if (conv.params.color_ref.mul[i] > 0) {
                scaled = values[i] * conv.params.color_ref.mul[i];
                if (scaled < min_scaled) {
                    min_scaled = scaled;
                }
            }
        }

        if (isinf(min_scaled)) {
            /* The white LED emits nothing; it is not used */
            result.white = 0;
        } else {
            result.white = min_scaled <= 0 ? 0 : (min_scaled <= 255 ? ((uint8_t)min_scaled) : 255);
        }

        for (i = 0; i < 3; i++) {
            correction = result.white * conv.params.color_ref.div[i];
            *results[i] = values[i] > correction ? values[i] - correction : 0;
        }
    }
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

/**
 * @brief Determines whether two RGBW colors are equal with a tolerance
 *
 * @param first   the first color
 * @param second  the second color
 * @param eps     the tolerance level, i.e. the maximum allowed difference
 *        between the individual components of the two colors
 * @return whether the two colors are equal (with tolerance)
 */
sb_bool_t sb_rgbw_color_almost_equals(sb_rgbw_color_t first, sb_rgbw_color_t second, uint8_t eps)
{
    return (
        abs(first.red - second.red) <= eps && abs(first.green - second.green) <= eps && abs(first.blue - second.blue) <= eps && abs(first.white - second.white) <= eps);
}

/**
 * @brief Sets up the RGBW conversion object so it always uses zero for the white channel.
 *
 * @param  conv  the conversion object to set up
 */
void sb_rgbw_conversion_turn_off(sb_rgbw_conversion_t* conv)
{
    sb_rgbw_conversion_use_fixed_value(conv, 0);
}

/**
 * @brief Sets up the RGBW conversion object so it always uses a fixed value for the white channel.
 *
 * @param  conv  the conversion object to set up
 * @param  value the fixed value for the W channel
 */
void sb_rgbw_conversion_use_fixed_value(sb_rgbw_conversion_t* conv, uint8_t value)
{
    conv->method = SB_RGBW_CONVERSION_FIXED_VALUE;
    conv->params.fixed_value = value;
}

/**
 * @brief Sets up the RGBW conversion object to use the min-subtraction method.
 *
 * The algorithm assumes a perfect white LED on the white channel and simply
 * subtracts the minimum of the red, green and blue channels from all these
 * three channels after assigning the remainder to the white channel.
 *
 * @param conv  the conversion object to set up
 */
void sb_rgbw_conversion_use_min_subtraction(sb_rgbw_conversion_t* conv)
{
    conv->method = SB_RGBW_CONVERSION_SUBTRACT_MIN;
}

/**
 * @brief Sets up the RGBW conversion object with a color temperature.
 *
 * The RGBW converter will assume the the white LED produces a color equivalent
 * to the radiation of a black body at the given temperature. E.g., 4500K
 * corresponds to a typical "warm white" color.
 *
 * @param conv         the conversion object to set up
 * @param temperature  the color temperature in Kelvins
 */
void sb_rgbw_conversion_use_color_temperature(sb_rgbw_conversion_t* conv, float temperature)
{
    /* Nothing to do if the conversion is already set up with the same
     * temperature. A conversion set up with a reference color records a
     * temperature of zero, so zero never counts as the same temperature. */
    if (conv->method == SB_RGBW_CONVERSION_USE_REFERENCE && conv->params.color_ref.temperature != 0 && conv->params.color_ref.temperature == temperature) {
        return;
    }

    sb_rgbw_conversion_use_reference_color(
        conv, sb_rgb_color_from_color_temperature(temperature));
    conv->params.color_ref.temperature = temperature;
}

/**
 * @brief Sets up the RGBW conversion object with a color temperature.
 *
 * The RGBW converter will assume the the white LED produces a color that is
 * equivalent to the color emitted by the combination of the red, green and
 * blue LEDs with the given reference color.
 *
 * It is advised to pass a reference color where the largest component is 255.
 * We will nevertheless normalize the color if this is not the case.
 *
 * @param conv         the conversion object to set up
 * @param reference  the reference color that the white LED emits
 */
void sb_rgbw_conversion_use_reference_color(sb_rgbw_conversion_t* conv, sb_rgb_color_t reference)
{
    float max_value = 1;
    uint8_t i;

    if (reference.red > max_value) {
        max_value = reference.red;
    }
    if (reference.green > max_value) {
        max_value = reference.green;
    }
    if (reference.blue > max_value) {
        max_value = reference.blue;
    }

    conv->method = SB_RGBW_CONVERSION_USE_REFERENCE;

    /* A channel that the reference color lacks gets a multiplier and a
     * divisor of zero: the white LED does not emit it, so it neither limits
     * the white level nor is reduced by it */
    conv->params.color_ref.mul[0] = reference.red >= 1 ? max_value / reference.red : 0;
    conv->params.color_ref.mul[1] = reference.green >= 1 ? max_value / reference.green : 0;
    conv->params.color_ref.mul[2] = reference.blue >= 1 ? max_value / reference.blue : 0;
    conv->params.color_ref.temperature = 0; /* not based on temperature */

    for (i = 0; i < 3; i++) {
        conv->params.color_ref.div[i] = conv->params.color_ref.mul[i] > 0 ? 1.0f / conv->params.color_ref.mul[i] : 0;
    }
}

/**
 * @brief Calculates the color with which an ideal black body radiates at the given temperature.
 *
 * This function uses an approximation; see here:
 * https://tannerhelland.com/2012/09/18/convert-temperature-rgb-algorithm-code.html
 *
 * Another option would be to use a lookup table similar to this one, but this
 * would increase the size of the library significantly:
 * http://www.vendian.org/mncharity/dir3/blackbody/UnstableURLs/bbr_color.html
 *
 * @param temperature   the temperature. Must be between 1000 and 40000 Kelvin.
 * @return the color of the black body radiation at the given temperature, in RGB space
 */
sb_rgb_color_t sb_rgb_color_from_color_temperature(float temperature)
{
    float temp_div = (temperature < 1000) ? 10 : (temperature > 40000 ? 400 : temperature / 100);
    sb_rgb_color_t result;
    float value;

    /* red */
    if (temp_div <= 66) {
        result.red = 255;
    } else {
        value = 329.698727446f * powf(temp_div - 60, -0.1332047592f);
        result.red = sb_i_color_component(value);
    }

    /* green */
    if (temp_div <= 66) {
        value = 99.4708025861f * logf(temp_div) - 161.1195681661f;
    } else {
        value = 288.1221695283f * powf(temp_div - 60, -0.0755148492f);
    }
    result.green = sb_i_color_component(value);

    /* blue */
    if (temp_div >= 66) {
        result.blue = 255;
    } else {
        value = 138.5177312231f * logf(temp_div - 10) - 305.0447927307f;
        result.blue = sb_i_color_component(value);
    }

    return result;
}
