#include <skybrush/colors.h>

#include "../utils.h"

const sb_rgb_color_t SB_COLOR_BLACK = {0, 0, 0};
const sb_rgb_color_t SB_COLOR_WHITE = {255, 255, 255};

sb_bool_t sb_rgb_color_equals(
    sb_rgb_color_t first, sb_rgb_color_t second)
{
  return first.red == second.red && first.green == second.green &&
         first.blue == second.blue;
}

sb_rgb_color_t sb_rgb_color_linear_interpolation(
    sb_rgb_color_t first, sb_rgb_color_t second, float ratio)
{
  sb_rgb_color_t result;

  result.red = clamp(first.red + (second.red - first.red) * ratio, 0, 255);
  result.green = clamp(first.green + (second.green - first.green) * ratio, 0, 255);
  result.blue = clamp(first.blue + (second.blue - first.blue) * ratio, 0, 255);

  return result;
}
