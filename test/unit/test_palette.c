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

#include <skybrush/palette.h>
#include <stdlib.h>
#include <string.h>

#include "unity.h"

void setUp(void)
{
}

void tearDown(void)
{
}

void test_init_empty(void)
{
    sb_color_palette_t palette;

    TEST_ASSERT_EQUAL(SB_EINVAL, sb_color_palette_init(0));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_color_palette_init(&palette));

    TEST_ASSERT_EQUAL(0, palette.num_colors);
    TEST_ASSERT(!sb_buffer_is_view(&palette.buffer));

    /* an empty palette yields the black color for any index */
    TEST_ASSERT_TRUE(sb_rgb_color_equals(SB_COLOR_BLACK, sb_color_palette_get_color(&palette, 0)));
    TEST_ASSERT_TRUE(sb_rgb_color_equals(SB_COLOR_BLACK, sb_color_palette_get_color(&palette, 42)));

    sb_color_palette_destroy(&palette);
}

void test_clear(void)
{
    sb_color_palette_t palette;
    uint8_t bytes[3] = { 0xff, 0x00, 0x00 };

    TEST_ASSERT_EQUAL(SB_EINVAL, sb_color_palette_clear(0));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_color_palette_init(&palette));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_color_palette_update_from_buffer(&palette, bytes, sizeof(bytes)));
    TEST_ASSERT_EQUAL(1, palette.num_colors);
    TEST_ASSERT(sb_buffer_is_view(&palette.buffer));

    /* clearing the palette must replace the borrowed view with an owned,
     * empty buffer */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_color_palette_clear(&palette));
    TEST_ASSERT_EQUAL(0, palette.num_colors);
    TEST_ASSERT(!sb_buffer_is_view(&palette.buffer));
    TEST_ASSERT_TRUE(sb_rgb_color_equals(SB_COLOR_BLACK, sb_color_palette_get_color(&palette, 0)));

    sb_color_palette_destroy(&palette);
}

void test_update_from_buffer(void)
{
    sb_color_palette_t palette;
    uint8_t bytes[9] = {
        0xff, 0x00, 0x00, /* red */
        0x00, 0xff, 0x00, /* green */
        0x00, 0x00, 0xff /* blue */
    };

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_color_palette_init(&palette));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_color_palette_update_from_buffer(&palette, bytes, sizeof(bytes)));

    TEST_ASSERT_EQUAL(3, palette.num_colors);
    TEST_ASSERT(sb_buffer_is_view(&palette.buffer));

    TEST_ASSERT_TRUE(
        sb_rgb_color_equals(sb_rgb_color_make(255, 0, 0), sb_color_palette_get_color(&palette, 0)));
    TEST_ASSERT_TRUE(
        sb_rgb_color_equals(sb_rgb_color_make(0, 255, 0), sb_color_palette_get_color(&palette, 1)));
    TEST_ASSERT_TRUE(
        sb_rgb_color_equals(sb_rgb_color_make(0, 0, 255), sb_color_palette_get_color(&palette, 2)));

    /* indices out of range yield the black color */
    TEST_ASSERT_TRUE(sb_rgb_color_equals(SB_COLOR_BLACK, sb_color_palette_get_color(&palette, 3)));
    TEST_ASSERT_TRUE(sb_rgb_color_equals(SB_COLOR_BLACK, sb_color_palette_get_color(&palette, 100)));

    sb_color_palette_destroy(&palette);
}

void test_update_from_buffer_uses_a_borrowed_view(void)
{
    sb_color_palette_t palette;
    uint8_t bytes[3] = { 0xff, 0x00, 0x00 };

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_color_palette_init(&palette));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_color_palette_update_from_buffer(&palette, bytes, sizeof(bytes)));

    /* the palette does not take a copy; the caller keeps the ownership of
     * the buffer, so modifying the buffer is visible through the palette */
    bytes[1] = 0xff;

    TEST_ASSERT_EQUAL(1, palette.num_colors);
    TEST_ASSERT_TRUE(
        sb_rgb_color_equals(sb_rgb_color_make(255, 255, 0), sb_color_palette_get_color(&palette, 0)));

    sb_color_palette_destroy(&palette);
}

void test_update_from_buffer_null_buffer_with_zero_size(void)
{
    sb_color_palette_t palette;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_color_palette_init(&palette));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_color_palette_update_from_buffer(&palette, 0, 0));

    /* the palette becomes empty and owns its (empty) internal buffer */
    TEST_ASSERT_EQUAL(0, palette.num_colors);
    TEST_ASSERT(!sb_buffer_is_view(&palette.buffer));
    TEST_ASSERT_TRUE(sb_rgb_color_equals(SB_COLOR_BLACK, sb_color_palette_get_color(&palette, 0)));

    sb_color_palette_destroy(&palette);
}

void test_update_from_bytes_takes_ownership(void)
{
    sb_color_palette_t palette;
    uint8_t* bytes = malloc(6);

    TEST_ASSERT_NOT_NULL(bytes);
    bytes[0] = 0xff;
    bytes[1] = 0x00;
    bytes[2] = 0x00; /* red */
    bytes[3] = 0x0f;
    bytes[4] = 0x1e;
    bytes[5] = 0x2d; /* some custom color */

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_color_palette_init(&palette));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_color_palette_update_from_bytes(&palette, bytes, 6));

    /* the palette has taken over the buffer, so it must not be a view and
     * the caller must not free it any more; destroying the palette releases
     * the buffer */
    TEST_ASSERT_EQUAL(2, palette.num_colors);
    TEST_ASSERT(!sb_buffer_is_view(&palette.buffer));

    TEST_ASSERT_TRUE(
        sb_rgb_color_equals(sb_rgb_color_make(255, 0, 0), sb_color_palette_get_color(&palette, 0)));
    TEST_ASSERT_TRUE(
        sb_rgb_color_equals(sb_rgb_color_make(0x0f, 0x1e, 0x2d), sb_color_palette_get_color(&palette, 1)));

    sb_color_palette_destroy(&palette);
}

void test_update_replaces_previous_content(void)
{
    sb_color_palette_t palette;
    uint8_t first[3] = { 0xff, 0x00, 0x00 };
    uint8_t second[6] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05 };

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_color_palette_init(&palette));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_color_palette_update_from_buffer(&palette, first, sizeof(first)));
    TEST_ASSERT_EQUAL(1, palette.num_colors);

    /* updating the palette again must release the previous buffer and
     * replace it with the new one */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_color_palette_update_from_buffer(&palette, second, sizeof(second)));
    TEST_ASSERT_EQUAL(2, palette.num_colors);
    TEST_ASSERT_TRUE(
        sb_rgb_color_equals(sb_rgb_color_make(0x00, 0x01, 0x02), sb_color_palette_get_color(&palette, 0)));
    TEST_ASSERT_TRUE(
        sb_rgb_color_equals(sb_rgb_color_make(0x03, 0x04, 0x05), sb_color_palette_get_color(&palette, 1)));

    sb_color_palette_destroy(&palette);
}

void test_update_invalid_args(void)
{
    sb_color_palette_t palette;
    uint8_t bytes[7] = { 0, 1, 2, 3, 4, 5, 6 };

    TEST_ASSERT_EQUAL(SB_EINVAL, sb_color_palette_update_from_buffer(0, bytes, sizeof(bytes)));
    TEST_ASSERT_EQUAL(SB_EINVAL, sb_color_palette_update_from_bytes(0, bytes, sizeof(bytes)));

    /* the number of bytes must be a multiple of three */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_color_palette_init(&palette));
    TEST_ASSERT_EQUAL(SB_EINVAL, sb_color_palette_update_from_buffer(&palette, bytes, 1));
    TEST_ASSERT_EQUAL(SB_EINVAL, sb_color_palette_update_from_buffer(&palette, bytes, 4));
    TEST_ASSERT_EQUAL(SB_EINVAL, sb_color_palette_update_from_buffer(&palette, bytes, 7));
    TEST_ASSERT_EQUAL(SB_EINVAL, sb_color_palette_update_from_bytes(&palette, bytes, 1));
    TEST_ASSERT_EQUAL(SB_EINVAL, sb_color_palette_update_from_bytes(&palette, bytes, 4));
    TEST_ASSERT_EQUAL(SB_EINVAL, sb_color_palette_update_from_bytes(&palette, bytes, 7));

    /* a null buffer is invalid, except for update_from_buffer() with zero size */
    TEST_ASSERT_EQUAL(SB_EINVAL, sb_color_palette_update_from_buffer(&palette, 0, 3));
    TEST_ASSERT_EQUAL(SB_EINVAL, sb_color_palette_update_from_bytes(&palette, 0, 3));
    TEST_ASSERT_EQUAL(SB_EINVAL, sb_color_palette_update_from_bytes(&palette, 0, 0));

    /* update_from_bytes() cannot take over a zero-size buffer */
    TEST_ASSERT_EQUAL(SB_EINVAL, sb_color_palette_update_from_bytes(&palette, bytes, 0));

    /* the palette must not be modified on failure */
    TEST_ASSERT_EQUAL(0, palette.num_colors);
    TEST_ASSERT_TRUE(sb_rgb_color_equals(SB_COLOR_BLACK, sb_color_palette_get_color(&palette, 0)));

    sb_color_palette_destroy(&palette);
}

void test_get_color_invalid_palette(void)
{
    TEST_ASSERT_TRUE(sb_rgb_color_equals(SB_COLOR_BLACK, sb_color_palette_get_color(0, 0)));
    TEST_ASSERT_TRUE(sb_rgb_color_equals(SB_COLOR_BLACK, sb_color_palette_get_color(0, 42)));
}

void test_destroy(void)
{
    sb_color_palette_t palette;
    uint8_t bytes[3] = { 0xff, 0x00, 0x00 };

    /* destroying a null palette is a no-op */
    sb_color_palette_destroy(0);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_color_palette_init(&palette));
    sb_color_palette_destroy(&palette);

    /* the palette must be reset to a state where it cannot be used */
    TEST_ASSERT_EQUAL(0, palette.num_colors);
    TEST_ASSERT_TRUE(sb_rgb_color_equals(SB_COLOR_BLACK, sb_color_palette_get_color(&palette, 0)));

    /* calling destroy multiple times is safe */
    sb_color_palette_destroy(&palette);
    sb_color_palette_destroy(&palette);

    /* also safe for a palette backed by a borrowed view */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_color_palette_init(&palette));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_color_palette_update_from_buffer(&palette, bytes, sizeof(bytes)));
    sb_color_palette_destroy(&palette);
    sb_color_palette_destroy(&palette);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_init_empty);
    RUN_TEST(test_clear);
    RUN_TEST(test_update_from_buffer);
    RUN_TEST(test_update_from_buffer_uses_a_borrowed_view);
    RUN_TEST(test_update_from_buffer_null_buffer_with_zero_size);
    RUN_TEST(test_update_from_bytes_takes_ownership);
    RUN_TEST(test_update_replaces_previous_content);
    RUN_TEST(test_update_invalid_args);
    RUN_TEST(test_get_color_invalid_palette);
    RUN_TEST(test_destroy);

    return UNITY_END();
}
