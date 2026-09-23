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

#include <skybrush/formats/binary.h>
#include <skybrush/gcs_light_control.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "unity.h"
#include "utils.h"

void setUp(void)
{
}

void tearDown(void)
{
}

/* Convenience macro to encode a single TLV entry into a buffer at a given
 * offset and to advance the offset accordingly */
#define WRITE_ENTRY(buf, offset, tag, value, value_length)           \
    do {                                                             \
        size_t i_;                                                   \
        (buf)[(offset)++] = (tag);                                   \
        (buf)[(offset)++] = (uint8_t)((value_length) & 0xff);        \
        (buf)[(offset)++] = (uint8_t)(((value_length) >> 8) & 0xff); \
        for (i_ = 0; i_ < (value_length); i_++) {                    \
            (buf)[(offset)++] = (value)[i_];                         \
        }                                                            \
    } while (0)

/* Convenience macro to encode an X-Y coordinate pair tag into a buffer at
 * a given offset and to advance the offset accordingly */
#define WRITE_COORDS(buf, offset, x, y)                                 \
    do {                                                                \
        (buf)[(offset)++] = SB_GCS_LIGHT_CONTROL_SETUP_TAG_COORDINATES; \
        (buf)[(offset)++] = 0x04;                                       \
        (buf)[(offset)++] = 0x00;                                       \
        (buf)[(offset)++] = (uint8_t)((x) & 0xff);                      \
        (buf)[(offset)++] = (uint8_t)(((x) >> 8) & 0xff);               \
        (buf)[(offset)++] = (uint8_t)((y) & 0xff);                      \
        (buf)[(offset)++] = (uint8_t)(((y) >> 8) & 0xff);               \
    } while (0)

/* Convenience macro to encode a palette tag into a buffer at a given
 * offset and to advance the offset accordingly */
#define WRITE_PALETTE(buf, offset, value, value_length)              \
    do {                                                             \
        size_t i_;                                                   \
        (buf)[(offset)++] = SB_GCS_LIGHT_CONTROL_SETUP_TAG_PALETTE;  \
        (buf)[(offset)++] = (uint8_t)((value_length) & 0xff);        \
        (buf)[(offset)++] = (uint8_t)(((value_length) >> 8) & 0xff); \
        for (i_ = 0; i_ < (value_length); i_++) {                    \
            (buf)[(offset)++] = (value)[i_];                         \
        }                                                            \
    } while (0)

/* Convenience macro to encode a matrix size tag into a buffer at a given
 * offset and to advance the offset accordingly */
#define WRITE_SIZE(buf, offset, x, y)                                   \
    do {                                                                \
        (buf)[(offset)++] = SB_GCS_LIGHT_CONTROL_SETUP_TAG_MATRIX_SIZE; \
        (buf)[(offset)++] = 0x04;                                       \
        (buf)[(offset)++] = 0x00;                                       \
        (buf)[(offset)++] = (uint8_t)((x) & 0xff);                      \
        (buf)[(offset)++] = (uint8_t)(((x) >> 8) & 0xff);               \
        (buf)[(offset)++] = (uint8_t)((y) & 0xff);                      \
        (buf)[(offset)++] = (uint8_t)(((y) >> 8) & 0xff);               \
    } while (0)

static void assert_defaults(const sb_gcs_light_control_setup_t* setup)
{
    TEST_ASSERT_EQUAL_INT16(0, setup->coords.x);
    TEST_ASSERT_EQUAL_INT16(0, setup->coords.y);
    TEST_ASSERT_EQUAL_INT16(0, setup->size.x);
    TEST_ASSERT_EQUAL_INT16(0, setup->size.y);
    TEST_ASSERT_EQUAL(0, setup->palette.num_colors);
    TEST_ASSERT_TRUE(
        sb_rgb_color_equals(SB_COLOR_BLACK, sb_color_palette_get_color(&setup->palette, 0)));
}

void test_init(void)
{
    sb_gcs_light_control_setup_t setup;

    TEST_ASSERT_EQUAL(SB_EINVAL, sb_gcs_light_control_setup_init(0));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_gcs_light_control_setup_init(&setup));

    assert_defaults(&setup);

    sb_gcs_light_control_setup_destroy(&setup);
}

void test_clear(void)
{
    sb_gcs_light_control_setup_t setup;
    uint8_t body[13];
    size_t offset = 0;
    uint8_t palette[3] = { 0xff, 0x00, 0x00 };

    WRITE_COORDS(body, offset, 10, 20);
    WRITE_PALETTE(body, offset, palette, sizeof(palette));

    TEST_ASSERT_EQUAL(SB_EINVAL, sb_gcs_light_control_setup_clear(0));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_gcs_light_control_setup_init(&setup));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_gcs_light_control_setup_update_from_buffer(&setup, body, offset));
    TEST_ASSERT_EQUAL(1, setup.palette.num_colors);

    /* clearing the setup must release the borrowed palette view as well */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_gcs_light_control_setup_clear(&setup));
    assert_defaults(&setup);
    TEST_ASSERT(!sb_buffer_is_view(&setup.palette.buffer));

    sb_gcs_light_control_setup_destroy(&setup);
}

void test_update_from_buffer(void)
{
    sb_gcs_light_control_setup_t setup;
    uint8_t palette[6] = { 0xff, 0x00, 0x00, 0x00, 0x00, 0xff };
    uint8_t body[7 + 7 + 9];
    size_t offset = 0;

    WRITE_COORDS(body, offset, 10, 20);
    WRITE_SIZE(body, offset, 100, 200);
    WRITE_PALETTE(body, offset, palette, sizeof(palette));

    TEST_ASSERT_EQUAL(SB_EINVAL, sb_gcs_light_control_setup_update_from_buffer(0, body, sizeof(body)));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_gcs_light_control_setup_init(&setup));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_gcs_light_control_setup_update_from_buffer(&setup, body, offset));

    TEST_ASSERT_EQUAL_INT16(10, setup.coords.x);
    TEST_ASSERT_EQUAL_INT16(20, setup.coords.y);

    TEST_ASSERT_EQUAL_INT16(100, setup.size.x);
    TEST_ASSERT_EQUAL_INT16(200, setup.size.y);

    TEST_ASSERT_EQUAL(2, setup.palette.num_colors);
    TEST_ASSERT(sb_buffer_is_view(&setup.palette.buffer));
    TEST_ASSERT_TRUE(
        sb_rgb_color_equals(sb_rgb_color_make(255, 0, 0), sb_color_palette_get_color(&setup.palette, 0)));
    TEST_ASSERT_TRUE(
        sb_rgb_color_equals(sb_rgb_color_make(0, 0, 255), sb_color_palette_get_color(&setup.palette, 1)));

    sb_gcs_light_control_setup_destroy(&setup);
}

void test_update_from_buffer_uses_a_borrowed_view(void)
{
    sb_gcs_light_control_setup_t setup;
    uint8_t palette[3] = { 0xff, 0x00, 0x00 };
    uint8_t body[9];
    size_t offset = 0;

    WRITE_PALETTE(body, offset, palette, sizeof(palette));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_gcs_light_control_setup_init(&setup));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_gcs_light_control_setup_update_from_buffer(&setup, body, offset));

    /* modifying the body of the block is visible through the palette
     * because the palette borrows a view into the body */
    body[3 + 1] = 0xff;

    TEST_ASSERT_TRUE(
        sb_rgb_color_equals(sb_rgb_color_make(255, 255, 0), sb_color_palette_get_color(&setup.palette, 0)));

    sb_gcs_light_control_setup_destroy(&setup);
}

void test_update_from_bytes_takes_ownership(void)
{
    sb_gcs_light_control_setup_t setup;
    uint8_t palette[3] = { 0xff, 0x00, 0x00 };
    uint8_t* body = malloc(13);
    size_t offset = 0;

    TEST_ASSERT_NOT_NULL(body);
    WRITE_COORDS(body, offset, 1000, 4094);
    WRITE_PALETTE(body, offset, palette, sizeof(palette));

    TEST_ASSERT_EQUAL(SB_EINVAL, sb_gcs_light_control_setup_update_from_bytes(0, body, offset));

    /* the body was released by the call above so we need a new one */
    body = malloc(13);
    TEST_ASSERT_NOT_NULL(body);
    offset = 0;
    WRITE_COORDS(body, offset, 1000, 4094);
    WRITE_PALETTE(body, offset, palette, sizeof(palette));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_gcs_light_control_setup_init(&setup));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_gcs_light_control_setup_update_from_bytes(&setup, body, offset));

    /* the palette must have its own copy because the body of the block does
     * not outlive the update function */
    TEST_ASSERT_EQUAL_INT16(1000, setup.coords.x);
    TEST_ASSERT_EQUAL_INT16(4094, setup.coords.y);
    TEST_ASSERT_EQUAL(1, setup.palette.num_colors);
    TEST_ASSERT(!sb_buffer_is_view(&setup.palette.buffer));

    sb_gcs_light_control_setup_destroy(&setup);
}

void test_update_with_empty_body(void)
{
    sb_gcs_light_control_setup_t setup;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_gcs_light_control_setup_init(&setup));

    /* update the setup with some values first */
    {
        uint8_t palette[3] = { 0xff, 0x00, 0x00 };
        uint8_t body[13];
        size_t offset = 0;

        WRITE_COORDS(body, offset, 7, 8);
        WRITE_PALETTE(body, offset, palette, sizeof(palette));

        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_gcs_light_control_setup_update_from_buffer(&setup, body, offset));
    }

    /* an empty body resets the setup to its default values */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_gcs_light_control_setup_update_from_buffer(&setup, 0, 0));
    assert_defaults(&setup);

    sb_gcs_light_control_setup_destroy(&setup);
}

void test_update_with_missing_tags(void)
{
    sb_gcs_light_control_setup_t setup;
    uint8_t palette[3] = { 0xff, 0x00, 0x00 };
    uint8_t body[9];
    size_t offset;

    /* body with an X-Y coordinate pair only */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_gcs_light_control_setup_init(&setup));

    offset = 0;
    WRITE_COORDS(body, offset, 5, 6);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_gcs_light_control_setup_update_from_buffer(&setup, body, offset));

    TEST_ASSERT_EQUAL_INT16(5, setup.coords.x);
    TEST_ASSERT_EQUAL_INT16(6, setup.coords.y);
    TEST_ASSERT_EQUAL(0, setup.palette.num_colors);

    /* body with a palette only */
    offset = 0;
    WRITE_PALETTE(body, offset, palette, sizeof(palette));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_gcs_light_control_setup_update_from_buffer(&setup, body, offset));

    TEST_ASSERT_EQUAL_INT16(0, setup.coords.x);
    TEST_ASSERT_EQUAL_INT16(0, setup.coords.y);
    TEST_ASSERT_EQUAL(0, setup.size.x);
    TEST_ASSERT_EQUAL_INT16(0, setup.size.y);
    TEST_ASSERT_EQUAL(1, setup.palette.num_colors);

    /* body with a matrix size only */
    offset = 0;
    WRITE_SIZE(body, offset, 100, 200);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_gcs_light_control_setup_update_from_buffer(&setup, body, offset));

    TEST_ASSERT_EQUAL_INT16(0, setup.coords.x);
    TEST_ASSERT_EQUAL_INT16(0, setup.coords.y);
    TEST_ASSERT_EQUAL_INT16(100, setup.size.x);
    TEST_ASSERT_EQUAL_INT16(200, setup.size.y);
    TEST_ASSERT_EQUAL(0, setup.palette.num_colors);

    sb_gcs_light_control_setup_destroy(&setup);
}

void test_update_with_empty_palette_tag(void)
{
    sb_gcs_light_control_setup_t setup;
    uint8_t body[3] = { SB_GCS_LIGHT_CONTROL_SETUP_TAG_PALETTE, 0x00, 0x00 };

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_gcs_light_control_setup_init(&setup));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_gcs_light_control_setup_update_from_buffer(&setup, body, sizeof(body)));

    assert_defaults(&setup);

    sb_gcs_light_control_setup_destroy(&setup);
}

void test_update_skips_unknown_tags(void)
{
    sb_gcs_light_control_setup_t setup;
    uint8_t palette[3] = { 0x12, 0x34, 0x56 };
    uint8_t body[5 + 7 + 3 + 6];
    size_t offset = 0;

    /* unknown tag with a two-byte value */
    body[offset++] = 0xfe;
    body[offset++] = 0x02;
    body[offset++] = 0x00;
    body[offset++] = 0xaa;
    body[offset++] = 0xbb;

    WRITE_COORDS(body, offset, 1, 2);

    /* another unknown tag, this time with a zero-length value */
    body[offset++] = 0xff;
    body[offset++] = 0x00;
    body[offset++] = 0x00;

    WRITE_PALETTE(body, offset, palette, sizeof(palette));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_gcs_light_control_setup_init(&setup));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_gcs_light_control_setup_update_from_buffer(&setup, body, offset));

    TEST_ASSERT_EQUAL_INT16(1, setup.coords.x);
    TEST_ASSERT_EQUAL_INT16(2, setup.coords.y);
    TEST_ASSERT_EQUAL(1, setup.palette.num_colors);
    TEST_ASSERT_TRUE(
        sb_rgb_color_equals(sb_rgb_color_make(0x12, 0x34, 0x56), sb_color_palette_get_color(&setup.palette, 0)));

    sb_gcs_light_control_setup_destroy(&setup);
}

void test_update_with_duplicate_tags_last_one_wins(void)
{
    sb_gcs_light_control_setup_t setup;
    uint8_t palette1[3] = { 0xff, 0x00, 0x00 };
    uint8_t palette2[3] = { 0x00, 0xff, 0x00 };
    uint8_t body[7 + 7 + 7 + 7 + 9 + 9];
    size_t offset = 0;

    WRITE_COORDS(body, offset, 1, 2);
    WRITE_COORDS(body, offset, 3, 4);

    WRITE_SIZE(body, offset, 10, 11);
    WRITE_SIZE(body, offset, 12, 13);

    WRITE_PALETTE(body, offset, palette1, sizeof(palette1));
    WRITE_PALETTE(body, offset, palette2, sizeof(palette2));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_gcs_light_control_setup_init(&setup));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_gcs_light_control_setup_update_from_buffer(&setup, body, offset));

    TEST_ASSERT_EQUAL_INT16(3, setup.coords.x);
    TEST_ASSERT_EQUAL_INT16(4, setup.coords.y);

    TEST_ASSERT_EQUAL_INT16(12, setup.size.x);
    TEST_ASSERT_EQUAL_INT16(13, setup.size.y);

    TEST_ASSERT_EQUAL(1, setup.palette.num_colors);
    TEST_ASSERT_TRUE(
        sb_rgb_color_equals(sb_rgb_color_make(0, 255, 0), sb_color_palette_get_color(&setup.palette, 0)));

    sb_gcs_light_control_setup_destroy(&setup);
}

void test_update_with_out_of_range_values(void)
{
    sb_gcs_light_control_setup_t setup;
    uint8_t body[14];
    size_t offset;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_gcs_light_control_setup_init(&setup));

    /* the largest allowed value (4095) is still okay for both the
     * coordinates and the matrix size */
    offset = 0;
    WRITE_COORDS(body, offset, 4095, 4095);
    WRITE_SIZE(body, offset, 4095, 4095);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_gcs_light_control_setup_update_from_buffer(&setup, body, offset));

    TEST_ASSERT_EQUAL_INT16(4095, setup.coords.x);
    TEST_ASSERT_EQUAL_INT16(4095, setup.coords.y);
    TEST_ASSERT_EQUAL_INT16(4095, setup.size.x);
    TEST_ASSERT_EQUAL_INT16(4095, setup.size.y);

    /* negative coordinates are not allowed */
    offset = 0;
    WRITE_COORDS(body, offset, -1, -32768);
    TEST_ASSERT_EQUAL(SB_ECORRUPTED, sb_gcs_light_control_setup_update_from_buffer(&setup, body, offset));

    offset = 0;
    WRITE_COORDS(body, offset, 4095, -1);
    TEST_ASSERT_EQUAL(SB_ECORRUPTED, sb_gcs_light_control_setup_update_from_buffer(&setup, body, offset));

    /* coordinates of 4096 or above are not allowed */
    offset = 0;
    WRITE_COORDS(body, offset, 4096, 0);
    TEST_ASSERT_EQUAL(SB_ECORRUPTED, sb_gcs_light_control_setup_update_from_buffer(&setup, body, offset));

    /* negative matrix size is not allowed */
    offset = 0;
    WRITE_SIZE(body, offset, -1, 0);
    TEST_ASSERT_EQUAL(SB_ECORRUPTED, sb_gcs_light_control_setup_update_from_buffer(&setup, body, offset));

    offset = 0;
    WRITE_SIZE(body, offset, 4095, -1);
    TEST_ASSERT_EQUAL(SB_ECORRUPTED, sb_gcs_light_control_setup_update_from_buffer(&setup, body, offset));

    /* matrix size of 4096 or above is not allowed */
    offset = 0;
    WRITE_SIZE(body, offset, 0, 4096);
    TEST_ASSERT_EQUAL(SB_ECORRUPTED, sb_gcs_light_control_setup_update_from_buffer(&setup, body, offset));

    /* the setup must be left unmodified in each of the failed cases above */
    TEST_ASSERT_EQUAL_INT16(4095, setup.coords.x);
    TEST_ASSERT_EQUAL_INT16(4095, setup.coords.y);
    TEST_ASSERT_EQUAL_INT16(4095, setup.size.x);
    TEST_ASSERT_EQUAL_INT16(4095, setup.size.y);

    sb_gcs_light_control_setup_destroy(&setup);
}

void test_update_with_corrupted_body(void)
{
    sb_gcs_light_control_setup_t setup;
    uint8_t good_palette[3] = { 0xff, 0x00, 0x00 };
    uint8_t good_body[7 + 9 + 7];
    size_t good_offset = 0;
    uint8_t body[16];
    size_t offset;

    WRITE_COORDS(good_body, good_offset, 10, 20);
    WRITE_SIZE(good_body, good_offset, 30, 40);
    WRITE_PALETTE(good_body, good_offset, good_palette, sizeof(good_palette));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_gcs_light_control_setup_init(&setup));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_gcs_light_control_setup_update_from_buffer(&setup, good_body, good_offset));

    /* X-Y coordinate pair with an invalid length (3 bytes instead of 4) */
    offset = 0;
    body[offset++] = SB_GCS_LIGHT_CONTROL_SETUP_TAG_COORDINATES;
    body[offset++] = 0x03;
    body[offset++] = 0x00;
    body[offset++] = 0x01;
    body[offset++] = 0x00;
    body[offset++] = 0x02;
    TEST_ASSERT_EQUAL(SB_ECORRUPTED, sb_gcs_light_control_setup_update_from_buffer(&setup, body, offset));

    /* X-Y coordinate pair with an invalid length (5 bytes instead of 4) */
    offset = 0;
    WRITE_ENTRY(body, offset, SB_GCS_LIGHT_CONTROL_SETUP_TAG_COORDINATES, "\x01\x00\x02\x00\x03", 5);
    TEST_ASSERT_EQUAL(SB_ECORRUPTED, sb_gcs_light_control_setup_update_from_buffer(&setup, body, offset));

    /* palette with a length that is not a multiple of three */
    offset = 0;
    WRITE_ENTRY(body, offset, SB_GCS_LIGHT_CONTROL_SETUP_TAG_PALETTE, "\x01\x02\x03\x04", 4);
    TEST_ASSERT_EQUAL(SB_ECORRUPTED, sb_gcs_light_control_setup_update_from_buffer(&setup, body, offset));

    /* matrix size with an invalid length (3 bytes instead of 4) */
    offset = 0;
    WRITE_ENTRY(body, offset, SB_GCS_LIGHT_CONTROL_SETUP_TAG_MATRIX_SIZE, "\x01\x00\x02", 3);
    TEST_ASSERT_EQUAL(SB_ECORRUPTED, sb_gcs_light_control_setup_update_from_buffer(&setup, body, offset));

    /* truncated tag-length-value header */
    offset = 0;
    body[offset++] = SB_GCS_LIGHT_CONTROL_SETUP_TAG_COORDINATES;
    body[offset++] = 0x04;
    TEST_ASSERT_EQUAL(SB_ECORRUPTED, sb_gcs_light_control_setup_update_from_buffer(&setup, body, offset));

    /* null buffer with a positive size */
    TEST_ASSERT_EQUAL(SB_EINVAL, sb_gcs_light_control_setup_update_from_buffer(&setup, 0, 4));

    /* the setup must be left unmodified in each of the failed cases above */
    TEST_ASSERT_EQUAL_INT16(10, setup.coords.x);
    TEST_ASSERT_EQUAL_INT16(20, setup.coords.y);
    TEST_ASSERT_EQUAL_INT16(30, setup.size.x);
    TEST_ASSERT_EQUAL_INT16(40, setup.size.y);
    TEST_ASSERT_EQUAL(1, setup.palette.num_colors);
    TEST_ASSERT_TRUE(
        sb_rgb_color_equals(sb_rgb_color_make(255, 0, 0), sb_color_palette_get_color(&setup.palette, 0)));

    sb_gcs_light_control_setup_destroy(&setup);
}

void test_update_from_binary_file_in_memory(void)
{
    sb_gcs_light_control_setup_t setup;
    uint8_t palette[3] = { 0x0f, 0x1e, 0x2d };
    uint8_t file[5 + 3 + 13];
    size_t offset = 0;
    size_t body_offset;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_gcs_light_control_setup_init(&setup));

    /* header of the Skybrush binary show file */
    offset = make_skyb_header(file, 0);

    /* GCS light control setup block */
    body_offset = offset + 3;
    file[offset++] = SB_BINARY_BLOCK_GCS_LIGHT_CONTROL_SETUP;
    file[offset++] = 13; /* length of the body */
    file[offset++] = 0x00;

    WRITE_COORDS(file, body_offset, 123, 456);
    WRITE_PALETTE(file, body_offset, palette, sizeof(palette));

    TEST_ASSERT_EQUAL(body_offset, offset + 13);

    TEST_ASSERT_EQUAL(
        SB_SUCCESS, sb_gcs_light_control_setup_update_from_binary_file_in_memory(&setup, file, sizeof(file)));

    TEST_ASSERT_EQUAL_INT16(123, setup.coords.x);
    TEST_ASSERT_EQUAL_INT16(456, setup.coords.y);

    TEST_ASSERT_EQUAL(1, setup.palette.num_colors);
    TEST_ASSERT(sb_buffer_is_view(&setup.palette.buffer));
    TEST_ASSERT_TRUE(
        sb_rgb_color_equals(sb_rgb_color_make(0x0f, 0x1e, 0x2d), sb_color_palette_get_color(&setup.palette, 0)));

    sb_gcs_light_control_setup_destroy(&setup);
}

void test_update_from_binary_file(void)
{
    sb_gcs_light_control_setup_t setup;
    uint8_t palette[3] = { 0x0f, 0x1e, 0x2d };
    uint8_t file[5 + 3 + 13];
    size_t offset = 0;
    size_t body_offset;
    FILE* fp;
    int fd;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_gcs_light_control_setup_init(&setup));

    /* header of the Skybrush binary show file */
    offset = make_skyb_header(file, 0);

    /* GCS light control setup block */
    body_offset = offset + 3;
    file[offset++] = SB_BINARY_BLOCK_GCS_LIGHT_CONTROL_SETUP;
    file[offset++] = 13; /* length of the body */
    file[offset++] = 0x00;

    WRITE_COORDS(file, body_offset, 123, 456);
    WRITE_PALETTE(file, body_offset, palette, sizeof(palette));

    TEST_ASSERT_EQUAL(body_offset, offset + 13);

    fp = tmpfile();
    TEST_ASSERT_NOT_NULL(fp);
    TEST_ASSERT_EQUAL(sizeof(file), fwrite(file, 1, sizeof(file), fp));
    fflush(fp);

    fd = fileno(fp);
    TEST_ASSERT_GREATER_OR_EQUAL(0, fd);

    /* the file descriptor must be rewound to the beginning of the file
     * because the parser reads from the current position of the descriptor */
    TEST_ASSERT_EQUAL(0, lseek(fd, 0, SEEK_SET));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_gcs_light_control_setup_update_from_binary_file(&setup, fd));

    /* the palette must have its own copy because the body of the block was
     * released after parsing; closing the file must not invalidate it */
    fclose(fp);

    TEST_ASSERT_EQUAL_INT16(123, setup.coords.x);
    TEST_ASSERT_EQUAL_INT16(456, setup.coords.y);

    TEST_ASSERT_EQUAL(1, setup.palette.num_colors);
    TEST_ASSERT(!sb_buffer_is_view(&setup.palette.buffer));
    TEST_ASSERT_TRUE(
        sb_rgb_color_equals(sb_rgb_color_make(0x0f, 0x1e, 0x2d), sb_color_palette_get_color(&setup.palette, 0)));

    sb_gcs_light_control_setup_destroy(&setup);
}

void test_update_from_binary_file_without_setup_block(void)
{
    sb_gcs_light_control_setup_t setup;
    uint8_t file[5 + 3 + 3];
    size_t offset;

    /* header of the Skybrush binary show file, followed by a comment block
     * with the text "abc" so that the file is not empty */
    offset = make_skyb_header(file, 0);
    file[offset++] = SB_BINARY_BLOCK_COMMENT;
    file[offset++] = 0x03;
    file[offset++] = 0x00;
    file[offset++] = 0x61;
    file[offset++] = 0x62;
    file[offset++] = 0x63;

    TEST_ASSERT_EQUAL(sizeof(file), offset);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_gcs_light_control_setup_init(&setup));

    /* fill the setup with some values first so that we can check that it
     * is left unmodified when the block is not found */
    {
        uint8_t palette[3] = { 0xff, 0x00, 0x00 };
        uint8_t body[13];
        size_t offset = 0;

        WRITE_COORDS(body, offset, 7, 8);
        WRITE_PALETTE(body, offset, palette, sizeof(palette));

        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_gcs_light_control_setup_update_from_buffer(&setup, body, offset));
    }

    TEST_ASSERT_EQUAL(
        SB_ENOENT, sb_gcs_light_control_setup_update_from_binary_file_in_memory(&setup, file, sizeof(file)));

    TEST_ASSERT_EQUAL_INT16(7, setup.coords.x);
    TEST_ASSERT_EQUAL_INT16(8, setup.coords.y);
    TEST_ASSERT_EQUAL(1, setup.palette.num_colors);

    sb_gcs_light_control_setup_destroy(&setup);
}

/* Loads the contents of a fixture file from the test/fixtures directory
 * into a newly allocated buffer */
static uint8_t* load_fixture(const char* fname, size_t* size)
{
    FILE* fp = fopen(fname, "rb");
    uint8_t* buf;

    TEST_ASSERT_NOT_NULL(fp);

    TEST_ASSERT_EQUAL(0, fseek(fp, 0, SEEK_END));
    *size = (size_t)ftell(fp);
    TEST_ASSERT_GREATER_THAN(0, *size);
    TEST_ASSERT_EQUAL(0, fseek(fp, 0, SEEK_SET));

    buf = malloc(*size);
    TEST_ASSERT_NOT_NULL(buf);
    TEST_ASSERT_EQUAL(*size, fread(buf, 1, *size, fp));

    fclose(fp);

    return buf;
}

void test_update_from_binary_file_fixture(void)
{
    sb_gcs_light_control_setup_t setup;
    uint8_t* buf;
    size_t size;

    buf = load_fixture("fixtures/hover_3m_with_gcs_control_block.skyb", &size);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_gcs_light_control_setup_init(&setup));
    TEST_ASSERT_EQUAL(
        SB_SUCCESS, sb_gcs_light_control_setup_update_from_binary_file_in_memory(&setup, buf, size));

    TEST_ASSERT_EQUAL_INT16(8, setup.size.x);
    TEST_ASSERT_EQUAL_INT16(8, setup.size.y);

    TEST_ASSERT_EQUAL_INT16(2, setup.coords.x);
    TEST_ASSERT_EQUAL_INT16(3, setup.coords.y);

    TEST_ASSERT_EQUAL(0, sb_color_palette_size(&setup.palette));
    TEST_ASSERT_TRUE(sb_color_palette_is_empty(&setup.palette));

    /* the palette borrows a view into the buffer, so the setup must be
     * destroyed before the buffer is released */
    sb_gcs_light_control_setup_destroy(&setup);
    free(buf);
}

void test_destroy(void)
{
    sb_gcs_light_control_setup_t setup;

    /* destroying a null setup object is a no-op */
    sb_gcs_light_control_setup_destroy(0);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_gcs_light_control_setup_init(&setup));
    sb_gcs_light_control_setup_destroy(&setup);

    /* the setup object must be reset to a state where it cannot be used */
    assert_defaults(&setup);

    /* calling destroy multiple times is safe */
    sb_gcs_light_control_setup_destroy(&setup);
    sb_gcs_light_control_setup_destroy(&setup);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_init);
    RUN_TEST(test_clear);
    RUN_TEST(test_update_from_buffer);
    RUN_TEST(test_update_from_buffer_uses_a_borrowed_view);
    RUN_TEST(test_update_from_bytes_takes_ownership);
    RUN_TEST(test_update_with_empty_body);
    RUN_TEST(test_update_with_missing_tags);
    RUN_TEST(test_update_with_empty_palette_tag);
    RUN_TEST(test_update_skips_unknown_tags);
    RUN_TEST(test_update_with_duplicate_tags_last_one_wins);
    RUN_TEST(test_update_with_out_of_range_values);
    RUN_TEST(test_update_with_corrupted_body);
    RUN_TEST(test_update_from_binary_file_in_memory);
    RUN_TEST(test_update_from_binary_file);
    RUN_TEST(test_update_from_binary_file_fixture);
    RUN_TEST(test_update_from_binary_file_without_setup_block);
    RUN_TEST(test_destroy);

    return UNITY_END();
}
