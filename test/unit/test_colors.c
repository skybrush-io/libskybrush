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

#include <skybrush/colors.h>

#include "unity.h"

void setUp(void)
{
}

void tearDown(void)
{
}

void test_decode_rgb565(void)
{
    sb_rgb_color_t color;

    color = sb_rgb_color_decode_rgb565(0xf800);
    TEST_ASSERT_TRUE(sb_rgb_color_equals(sb_rgb_color_make(248, 0, 0), color));

    color = sb_rgb_color_decode_rgb565(0x07e0);
    TEST_ASSERT_TRUE(sb_rgb_color_equals(sb_rgb_color_make(0, 252, 0), color));

    color = sb_rgb_color_decode_rgb565(0x001f);
    TEST_ASSERT_TRUE(sb_rgb_color_equals(sb_rgb_color_make(0, 0, 248), color));

    color = sb_rgb_color_decode_rgb565(0xfc08);
    TEST_ASSERT_TRUE(sb_rgb_color_equals(sb_rgb_color_make(248, 128, 64), color));
}

void test_encode_rgb565(void)
{
    sb_rgb_color_t color;

    color = sb_rgb_color_make(255, 0, 0);
    TEST_ASSERT_EQUAL_UINT16(0xf800, sb_rgb_color_encode_rgb565(color));

    color = sb_rgb_color_make(0, 255, 0);
    TEST_ASSERT_EQUAL_UINT16(0x07e0, sb_rgb_color_encode_rgb565(color));

    color = sb_rgb_color_make(0, 0, 255);
    TEST_ASSERT_EQUAL_UINT16(0x001f, sb_rgb_color_encode_rgb565(color));

    color = sb_rgb_color_make(255, 128, 64);
    TEST_ASSERT_EQUAL_UINT16(0xfc08, sb_rgb_color_encode_rgb565(color));
}

void test_rgb_equals(void)
{
    sb_rgb_color_t red = { 255, 0, 0 };
    sb_rgb_color_t another_red = { 255, 0, 0 };
    sb_rgb_color_t white = { 255, 255, 255 };

    TEST_ASSERT_TRUE(sb_rgb_color_equals(red, another_red));
    TEST_ASSERT_TRUE(sb_rgb_color_equals(white, SB_COLOR_WHITE));

    TEST_ASSERT_FALSE(sb_rgb_color_equals(red, SB_COLOR_BLACK));
    TEST_ASSERT_FALSE(sb_rgb_color_equals(SB_COLOR_WHITE, SB_COLOR_BLACK));
}

void test_rgbw_equals(void)
{
    sb_rgbw_color_t red = { 255, 0, 0 };
    sb_rgbw_color_t another_red = { 255, 0, 0 };
    sb_rgbw_color_t black = { 0, 0, 0, 0 };
    sb_rgbw_color_t white = { 0, 0, 0, 255 };
    sb_rgbw_color_t another_white = { 0, 0, 0, 255 };
    sb_rgbw_color_t white_with_rgb_only = { 255, 255, 255, 0 };

    TEST_ASSERT_TRUE(sb_rgbw_color_equals(red, another_red));
    TEST_ASSERT_TRUE(sb_rgbw_color_equals(white, another_white));

    TEST_ASSERT_FALSE(sb_rgbw_color_equals(red, black));
    TEST_ASSERT_FALSE(sb_rgbw_color_equals(white, black));
    TEST_ASSERT_FALSE(sb_rgbw_color_equals(white, white_with_rgb_only));
}

void test_rgb_from_color_temperature(void)
{
    /* Approximation inexact around 1000K but then it gets better.
     * Expected values are from:
     * http://www.vendian.org/mncharity/dir3/blackbody/UnstableURLs/bbr_color.html
     */
    TEST_ASSERT_TRUE(sb_rgb_color_almost_equals(
        sb_rgb_color_from_color_temperature(1000),
        sb_rgb_color_make(255, 56, 0),
        12));
    TEST_ASSERT_TRUE(sb_rgb_color_almost_equals(
        sb_rgb_color_from_color_temperature(2000),
        sb_rgb_color_make(255, 137, 18),
        5));
    TEST_ASSERT_TRUE(sb_rgb_color_almost_equals(
        sb_rgb_color_from_color_temperature(3000),
        sb_rgb_color_make(255, 180, 107),
        5));
    TEST_ASSERT_TRUE(sb_rgb_color_almost_equals(
        sb_rgb_color_from_color_temperature(4000),
        sb_rgb_color_make(255, 209, 163),
        5));
    TEST_ASSERT_TRUE(sb_rgb_color_almost_equals(
        sb_rgb_color_from_color_temperature(5000),
        sb_rgb_color_make(255, 228, 206),
        5));
    TEST_ASSERT_TRUE(sb_rgb_color_almost_equals(
        sb_rgb_color_from_color_temperature(6000),
        sb_rgb_color_make(255, 243, 239),
        5));
    TEST_ASSERT_TRUE(sb_rgb_color_almost_equals(
        sb_rgb_color_from_color_temperature(7000),
        sb_rgb_color_make(245, 243, 255),
        5));
    TEST_ASSERT_TRUE(sb_rgb_color_almost_equals(
        sb_rgb_color_from_color_temperature(8000),
        sb_rgb_color_make(227, 233, 255),
        7));
    TEST_ASSERT_TRUE(sb_rgb_color_almost_equals(
        sb_rgb_color_from_color_temperature(9000),
        sb_rgb_color_make(214, 225, 255),
        5));
    TEST_ASSERT_TRUE(sb_rgb_color_almost_equals(
        sb_rgb_color_from_color_temperature(10000),
        sb_rgb_color_make(204, 219, 255),
        5));
}

void test_rgbw_conversion(void)
{
    sb_rgb_color_t color = { 128, 192, 254 };
    sb_rgbw_color_t converted;
    sb_rgbw_conversion_t conv;

    /* "off" method (i.e. no white channel) */
    sb_rgbw_conversion_turn_off(&conv);
    TEST_ASSERT_TRUE(sb_rgbw_color_equals(
        sb_rgbw_color_make(128, 192, 254, 0),
        sb_rgb_color_to_rgbw(color, conv)));

    /* "fixed value" method (white channel set to a fixed value) */
    sb_rgbw_conversion_use_fixed_value(&conv, 123);
    TEST_ASSERT_TRUE(sb_rgbw_color_equals(
        sb_rgbw_color_make(128, 192, 254, 123),
        sb_rgb_color_to_rgbw(color, conv)));

    /* assume that W is perfect white and use min(R, G, B) */
    sb_rgbw_conversion_use_min_subtraction(&conv);
    TEST_ASSERT_TRUE(sb_rgbw_color_equals(
        sb_rgbw_color_make(0, 64, 126, 128),
        sb_rgb_color_to_rgbw(color, conv)));
    color.green = 64;
    TEST_ASSERT_TRUE(sb_rgbw_color_equals(
        sb_rgbw_color_make(64, 0, 190, 64),
        sb_rgb_color_to_rgbw(color, conv)));
    color.blue = 32;
    TEST_ASSERT_TRUE(sb_rgbw_color_equals(
        sb_rgbw_color_make(96, 32, 0, 32),
        sb_rgb_color_to_rgbw(color, conv)));
    color.green = 192;
    color.blue = 254;

    /* test with a reference color */
    sb_rgbw_conversion_use_reference_color(&conv, sb_rgb_color_make(254, 127, 127));
    converted = sb_rgb_color_to_rgbw(color, conv);
    TEST_ASSERT_TRUE(sb_rgbw_color_equals(sb_rgbw_color_make(0, 128, 190, 128), converted));

    sb_rgbw_conversion_use_reference_color(&conv, sb_rgb_color_make(127, 254, 127));
    converted = sb_rgb_color_to_rgbw(color, conv);
    TEST_ASSERT_TRUE(sb_rgbw_color_equals(sb_rgbw_color_make(32, 0, 158, 192), converted));

    sb_rgbw_conversion_use_reference_color(&conv, sb_rgb_color_make(127, 127, 254));
    converted = sb_rgb_color_to_rgbw(color, conv);
    TEST_ASSERT_TRUE(sb_rgbw_color_equals(sb_rgbw_color_make(1, 65, 0, 254), converted));

    sb_rgbw_conversion_use_reference_color(&conv, sb_rgb_color_make(255, 219, 186));
    converted = sb_rgb_color_to_rgbw(SB_COLOR_WHITE, conv);
    TEST_ASSERT_TRUE(sb_rgbw_color_equals(sb_rgbw_color_make(0, 36, 69, 255), converted));

    /* test with a reference color of perfect white and check whether the
     * results are identical to the naive min(R, G, B) method */
    sb_rgbw_conversion_use_reference_color(&conv, sb_rgb_color_make(255, 255, 255));
    TEST_ASSERT_TRUE(sb_rgbw_color_equals(
        sb_rgbw_color_make(0, 64, 126, 128),
        sb_rgb_color_to_rgbw(color, conv)));
    color.green = 64;
    TEST_ASSERT_TRUE(sb_rgbw_color_equals(
        sb_rgbw_color_make(64, 0, 190, 64),
        sb_rgb_color_to_rgbw(color, conv)));
    color.blue = 32;
    TEST_ASSERT_TRUE(sb_rgbw_color_equals(
        sb_rgbw_color_make(96, 32, 0, 32),
        sb_rgb_color_to_rgbw(color, conv)));
    color.green = 192;
    color.blue = 254;

    /* test a white LED with a warm white color temperature of 3000K */
    sb_rgbw_conversion_use_color_temperature(&conv, 3000);
    TEST_ASSERT_TRUE(sb_rgbw_color_equals(
        sb_rgbw_color_make(0, 103, 199, 128),
        sb_rgb_color_to_rgbw(color, conv)));

    /* test a white LED with a cool white color temperature of 6000K */
    sb_rgbw_conversion_use_color_temperature(&conv, 6000);
    TEST_ASSERT_TRUE(sb_rgbw_color_equals(
        sb_rgbw_color_make(0, 68, 135, 128),
        sb_rgb_color_to_rgbw(color, conv)));
}

int main(int argc, char* argv[])
{
    UNITY_BEGIN();

    RUN_TEST(test_decode_rgb565);
    RUN_TEST(test_encode_rgb565);
    RUN_TEST(test_rgb_equals);
    RUN_TEST(test_rgb_from_color_temperature);
    RUN_TEST(test_rgbw_equals);
    RUN_TEST(test_rgbw_conversion);

    return UNITY_END();
}
