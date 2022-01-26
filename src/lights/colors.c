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
 * @brief Determines whether two colors are equal
 *
 * @param first   the first color
 * @param second  the second color
 * @return whether the two colors are equal
 */
sb_bool_t sb_rgb_color_equals(
    sb_rgb_color_t first, sb_rgb_color_t second)
{
    return first.red == second.red && first.green == second.green && first.blue == second.blue;
}

/**
 * @brief Linearly interpolates between two RGB colors
 *
 * @param first   the first color
 * @param second  the second color
 * @param ratio   the interpolation ratio; 0 will return the first color, 1
 *        will return the second color
 * @return the interpolated color
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
