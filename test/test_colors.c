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

#include "unity.h"

void setUp()
{
}

void tearDown()
{
}

void test_decode_rgb565()
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

void test_encode_rgb565()
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

void test_rgb_equals()
{
    sb_rgb_color_t red = { 255, 0, 0 };
    sb_rgb_color_t another_red = { 255, 0, 0 };
    sb_rgb_color_t white = { 255, 255, 255 };

    TEST_ASSERT_TRUE(sb_rgb_color_equals(red, another_red));
    TEST_ASSERT_TRUE(sb_rgb_color_equals(white, SB_COLOR_WHITE));

    TEST_ASSERT_FALSE(sb_rgb_color_equals(red, SB_COLOR_BLACK));
    TEST_ASSERT_FALSE(sb_rgb_color_equals(SB_COLOR_WHITE, SB_COLOR_BLACK));
}

void test_rgbw_equals()
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

void test_rgbw_conversion()
{
    // sb_rgb_color_t reference = { 255, 219, 186 };
    sb_rgb_color_t color = { 128, 192, 255 };
    sb_rgbw_conversion_t conv;

    conv.method = SB_RGBW_CONVERSION_OFF;
    conv.params.fixed_value = 123;
    TEST_ASSERT_TRUE(sb_rgbw_color_equals(
        sb_rgbw_color_make(128, 192, 255, 0),
        sb_rgb_color_to_rgbw(color, conv)));

    conv.method = SB_RGBW_CONVERSION_FIXED_VALUE;
    conv.params.fixed_value = 123;
    TEST_ASSERT_TRUE(sb_rgbw_color_equals(
        sb_rgbw_color_make(128, 192, 255, 123),
        sb_rgb_color_to_rgbw(color, conv)));

    conv.method = SB_RGBW_CONVERSION_SUBTRACT_MIN;
    conv.params.fixed_value = 123;
    TEST_ASSERT_TRUE(sb_rgbw_color_equals(
        sb_rgbw_color_make(0, 64, 127, 128),
        sb_rgb_color_to_rgbw(color, conv)));
    color.green = 64;
    TEST_ASSERT_TRUE(sb_rgbw_color_equals(
        sb_rgbw_color_make(64, 0, 191, 64),
        sb_rgb_color_to_rgbw(color, conv)));
    color.blue = 32;
    TEST_ASSERT_TRUE(sb_rgbw_color_equals(
        sb_rgbw_color_make(96, 32, 0, 32),
        sb_rgb_color_to_rgbw(color, conv)));
    color.green = 192;
    color.blue = 255;
}

int main(int argc, char* argv[])
{
    UNITY_BEGIN();

    RUN_TEST(test_decode_rgb565);
    RUN_TEST(test_encode_rgb565);
    RUN_TEST(test_rgb_equals);
    RUN_TEST(test_rgbw_equals);
    RUN_TEST(test_rgbw_conversion);

    return UNITY_END();
}
