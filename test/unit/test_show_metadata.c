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
#include <skybrush/metadata.h>
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

/* Convenience macro to encode a unique show ID tag into a buffer at a
 * given offset and to advance the offset accordingly */
#define WRITE_SHOW_ID(buf, offset, value) \
    WRITE_ENTRY(buf, offset, SB_SHOW_METADATA_TAG_SHOW_ID, value, SB_SHOW_METADATA_SHOW_ID_LENGTH)

/* Convenience macro to encode a drone index tag into a buffer at a given
 * offset and to advance the offset accordingly */
#define WRITE_DRONE_INDEX(buf, offset, value)                 \
    do {                                                      \
        (buf)[(offset)++] = SB_SHOW_METADATA_TAG_DRONE_INDEX; \
        (buf)[(offset)++] = 0x02;                             \
        (buf)[(offset)++] = 0x00;                             \
        (buf)[(offset)++] = (uint8_t)((value) & 0xff);        \
        (buf)[(offset)++] = (uint8_t)(((value) >> 8) & 0xff); \
    } while (0)

/* Asserts that a show metadata object holds the default values, i.e. an
 * all-zero show ID and a drone index of zero */
static void assert_defaults(const sb_show_metadata_t* metadata)
{
    static const uint8_t zero_id[SB_SHOW_METADATA_SHOW_ID_LENGTH] = { 0, 0, 0, 0 };
    uint8_t show_id[SB_SHOW_METADATA_SHOW_ID_LENGTH];
    uint16_t drone_index;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_get_show_id(metadata, show_id));
    TEST_ASSERT_EQUAL_MEMORY(zero_id, show_id, sizeof(show_id));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_get_drone_index(metadata, &drone_index));
    TEST_ASSERT_EQUAL_UINT16(0, drone_index);
}

void test_init(void)
{
    sb_show_metadata_t metadata;

    TEST_ASSERT_EQUAL(SB_EINVAL, sb_show_metadata_init(0));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_init(&metadata));

    assert_defaults(&metadata);

    sb_show_metadata_destroy(&metadata);
}

void test_clear(void)
{
    sb_show_metadata_t metadata;
    uint8_t show_id[SB_SHOW_METADATA_SHOW_ID_LENGTH] = { 1, 2, 3, 4 };
    uint8_t body[7 + 5];
    size_t offset = 0;

    WRITE_SHOW_ID(body, offset, show_id);
    WRITE_DRONE_INDEX(body, offset, 300);

    TEST_ASSERT_EQUAL(SB_EINVAL, sb_show_metadata_clear(0));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_init(&metadata));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_update_from_buffer(&metadata, body, offset));
    {
        uint8_t result[SB_SHOW_METADATA_SHOW_ID_LENGTH];
        uint16_t drone_index;
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_get_show_id(&metadata, result));
        TEST_ASSERT_EQUAL_MEMORY(show_id, result, sizeof(show_id));
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_get_drone_index(&metadata, &drone_index));
        TEST_ASSERT_EQUAL_UINT16(300, drone_index);
    }

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_clear(&metadata));
    assert_defaults(&metadata);

    sb_show_metadata_destroy(&metadata);
}

void test_update_from_buffer(void)
{
    sb_show_metadata_t metadata;
    uint8_t show_id[SB_SHOW_METADATA_SHOW_ID_LENGTH] = { 0xde, 0xad, 0xbe, 0xef };
    uint8_t result[SB_SHOW_METADATA_SHOW_ID_LENGTH];
    uint8_t body[7 + 5];
    uint16_t drone_index;
    size_t offset = 0;

    WRITE_SHOW_ID(body, offset, show_id);
    WRITE_DRONE_INDEX(body, offset, 0xbeef);

    TEST_ASSERT_EQUAL(
        SB_EINVAL, sb_show_metadata_update_from_buffer(0, body, sizeof(body)));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_init(&metadata));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_update_from_buffer(&metadata, body, offset));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_get_show_id(&metadata, result));
    TEST_ASSERT_EQUAL_MEMORY(show_id, result, sizeof(show_id));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_get_drone_index(&metadata, &drone_index));
    TEST_ASSERT_EQUAL_UINT16(0xbeef, drone_index);

    sb_show_metadata_destroy(&metadata);
}

void test_update_from_buffer_null_arguments(void)
{
    sb_show_metadata_t metadata;
    uint8_t body[1] = { 0 };

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_init(&metadata));

    /* null buffer with a positive size */
    TEST_ASSERT_EQUAL(SB_EINVAL, sb_show_metadata_update_from_buffer(&metadata, 0, 1));
    TEST_ASSERT_EQUAL(SB_EINVAL, sb_show_metadata_update_from_buffer(0, body, sizeof(body)));

    sb_show_metadata_destroy(&metadata);
}

void test_update_with_empty_body(void)
{
    sb_show_metadata_t metadata;
    uint8_t show_id[SB_SHOW_METADATA_SHOW_ID_LENGTH] = { 1, 2, 3, 4 };
    uint8_t body[7 + 5];
    uint16_t drone_index;
    size_t offset = 0;

    WRITE_SHOW_ID(body, offset, show_id);
    WRITE_DRONE_INDEX(body, offset, 42);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_init(&metadata));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_update_from_buffer(&metadata, body, offset));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_get_drone_index(&metadata, &drone_index));
    TEST_ASSERT_EQUAL_UINT16(42, drone_index);

    /* an empty body resets the object to its default values */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_update_from_buffer(&metadata, 0, 0));
    assert_defaults(&metadata);

    sb_show_metadata_destroy(&metadata);
}

void test_update_with_missing_tags(void)
{
    sb_show_metadata_t metadata;
    uint8_t show_id[SB_SHOW_METADATA_SHOW_ID_LENGTH] = { 9, 8, 7, 6 };
    uint8_t result[SB_SHOW_METADATA_SHOW_ID_LENGTH];
    uint8_t body[7 + 5];
    uint16_t drone_index;
    size_t offset;

    /* body with a unique show ID only */
    offset = 0;
    WRITE_SHOW_ID(body, offset, show_id);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_init(&metadata));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_update_from_buffer(&metadata, body, offset));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_get_show_id(&metadata, result));
    TEST_ASSERT_EQUAL_MEMORY(show_id, result, sizeof(show_id));

    /* the index of the drone is missing from the block, so it must have
     * its default value */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_get_drone_index(&metadata, &drone_index));
    TEST_ASSERT_EQUAL_UINT16(0, drone_index);

    /* body with the index of the drone only */
    offset = 0;
    WRITE_DRONE_INDEX(body, offset, 7);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_update_from_buffer(&metadata, body, offset));

    /* the show ID is missing from the block, so it must have its default
     * value */
    {
        static const uint8_t zero_id[SB_SHOW_METADATA_SHOW_ID_LENGTH] = { 0, 0, 0, 0 };
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_get_show_id(&metadata, result));
        TEST_ASSERT_EQUAL_MEMORY(zero_id, result, sizeof(zero_id));
    }

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_get_drone_index(&metadata, &drone_index));
    TEST_ASSERT_EQUAL_UINT16(7, drone_index);

    sb_show_metadata_destroy(&metadata);
}

void test_update_skips_unknown_tags(void)
{
    sb_show_metadata_t metadata;
    uint8_t show_id[SB_SHOW_METADATA_SHOW_ID_LENGTH] = { 0x12, 0x34, 0x56, 0x78 };
    uint8_t result[SB_SHOW_METADATA_SHOW_ID_LENGTH];
    uint8_t body[5 + 7 + 3 + 5];
    size_t offset = 0;

    /* unknown tag with a two-byte value */
    body[offset++] = 0xfe;
    body[offset++] = 0x02;
    body[offset++] = 0x00;
    body[offset++] = 0xaa;
    body[offset++] = 0xbb;

    WRITE_SHOW_ID(body, offset, show_id);

    /* another unknown tag, this time with a zero-length value */
    body[offset++] = 0xff;
    body[offset++] = 0x00;
    body[offset++] = 0x00;

    WRITE_DRONE_INDEX(body, offset, 65535);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_init(&metadata));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_update_from_buffer(&metadata, body, offset));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_get_show_id(&metadata, result));
    TEST_ASSERT_EQUAL_MEMORY(show_id, result, sizeof(show_id));

    {
        uint16_t drone_index;
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_get_drone_index(&metadata, &drone_index));
        TEST_ASSERT_EQUAL_UINT16(65535, drone_index);
    }

    sb_show_metadata_destroy(&metadata);
}

void test_update_with_duplicate_tags_last_one_wins(void)
{
    sb_show_metadata_t metadata;
    uint8_t show_id1[SB_SHOW_METADATA_SHOW_ID_LENGTH] = { 1, 1, 1, 1 };
    uint8_t show_id2[SB_SHOW_METADATA_SHOW_ID_LENGTH] = { 2, 2, 2, 2 };
    uint8_t result[SB_SHOW_METADATA_SHOW_ID_LENGTH];
    uint8_t body[7 + 7 + 5 + 5];
    uint16_t drone_index;
    size_t offset = 0;

    WRITE_SHOW_ID(body, offset, show_id1);
    WRITE_SHOW_ID(body, offset, show_id2);
    WRITE_DRONE_INDEX(body, offset, 1);
    WRITE_DRONE_INDEX(body, offset, 4711);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_init(&metadata));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_update_from_buffer(&metadata, body, offset));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_get_show_id(&metadata, result));
    TEST_ASSERT_EQUAL_MEMORY(show_id2, result, sizeof(show_id2));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_get_drone_index(&metadata, &drone_index));
    TEST_ASSERT_EQUAL_UINT16(4711, drone_index);

    sb_show_metadata_destroy(&metadata);
}

void test_update_with_corrupted_body(void)
{
    sb_show_metadata_t metadata;
    uint8_t good_show_id[SB_SHOW_METADATA_SHOW_ID_LENGTH] = { 0x11, 0x22, 0x33, 0x44 };
    uint8_t good_body[7 + 5];
    uint8_t body[8];
    size_t good_offset = 0;
    size_t offset;

    WRITE_SHOW_ID(good_body, good_offset, good_show_id);
    WRITE_DRONE_INDEX(good_body, good_offset, 13);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_init(&metadata));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_update_from_buffer(&metadata, good_body, good_offset));

    /* unique show ID with an invalid length (3 bytes instead of 4) */
    offset = 0;
    WRITE_ENTRY(body, offset, SB_SHOW_METADATA_TAG_SHOW_ID, "\x01\x02\x03", 3);
    TEST_ASSERT_EQUAL(SB_ECORRUPTED, sb_show_metadata_update_from_buffer(&metadata, body, offset));

    /* unique show ID with an invalid length (5 bytes instead of 4) */
    offset = 0;
    WRITE_ENTRY(body, offset, SB_SHOW_METADATA_TAG_SHOW_ID, "\x01\x02\x03\x04\x05", 5);
    TEST_ASSERT_EQUAL(SB_ECORRUPTED, sb_show_metadata_update_from_buffer(&metadata, body, offset));

    /* drone index with an invalid length (1 byte instead of 2) */
    offset = 0;
    WRITE_ENTRY(body, offset, SB_SHOW_METADATA_TAG_DRONE_INDEX, "\x01", 1);
    TEST_ASSERT_EQUAL(SB_ECORRUPTED, sb_show_metadata_update_from_buffer(&metadata, body, offset));

    /* drone index with an invalid length (3 bytes instead of 2) */
    offset = 0;
    WRITE_ENTRY(body, offset, SB_SHOW_METADATA_TAG_DRONE_INDEX, "\x01\x02\x03", 3);
    TEST_ASSERT_EQUAL(SB_ECORRUPTED, sb_show_metadata_update_from_buffer(&metadata, body, offset));

    /* truncated tag-length-value header */
    offset = 0;
    body[offset++] = SB_SHOW_METADATA_TAG_SHOW_ID;
    body[offset++] = 0x04;
    TEST_ASSERT_EQUAL(SB_ECORRUPTED, sb_show_metadata_update_from_buffer(&metadata, body, offset));

    /* the metadata must be left unmodified in each of the failed cases above */
    {
        uint8_t result[SB_SHOW_METADATA_SHOW_ID_LENGTH];
        uint16_t drone_index;

        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_get_show_id(&metadata, result));
        TEST_ASSERT_EQUAL_MEMORY(good_show_id, result, sizeof(good_show_id));

        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_get_drone_index(&metadata, &drone_index));
        TEST_ASSERT_EQUAL_UINT16(13, drone_index);
    }

    sb_show_metadata_destroy(&metadata);
}

void test_update_from_binary_file_in_memory(void)
{
    sb_show_metadata_t metadata;
    uint8_t show_id[SB_SHOW_METADATA_SHOW_ID_LENGTH] = { 0xca, 0xfe, 0xba, 0xbe };
    uint8_t result[SB_SHOW_METADATA_SHOW_ID_LENGTH];
    uint8_t file[5 + 3 + 7 + 5];
    uint16_t drone_index;
    size_t offset = 0;
    size_t body_offset;

    /* header of the Skybrush binary show file */
    offset = make_skyb_header(file, 0);

    /* show metadata block */
    body_offset = offset + 3;
    file[offset++] = SB_BINARY_BLOCK_SHOW_METADATA;
    file[offset++] = 12; /* length of the body */
    file[offset++] = 0x00;

    /* the body of the block must end exactly at the end of the file */
    TEST_ASSERT_EQUAL(sizeof(file), offset + 12);

    WRITE_SHOW_ID(file, body_offset, show_id);
    WRITE_DRONE_INDEX(file, body_offset, 1234);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_init(&metadata));
    TEST_ASSERT_EQUAL(
        SB_SUCCESS, sb_show_metadata_update_from_binary_file_in_memory(&metadata, file, sizeof(file)));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_get_show_id(&metadata, result));
    TEST_ASSERT_EQUAL_MEMORY(show_id, result, sizeof(show_id));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_get_drone_index(&metadata, &drone_index));
    TEST_ASSERT_EQUAL_UINT16(1234, drone_index);

    sb_show_metadata_destroy(&metadata);
}

void test_update_from_binary_file(void)
{
    sb_show_metadata_t metadata;
    uint8_t show_id[SB_SHOW_METADATA_SHOW_ID_LENGTH] = { 0x0f, 0x1e, 0x2d, 0x3c };
    uint8_t result[SB_SHOW_METADATA_SHOW_ID_LENGTH];
    uint8_t file[5 + 3 + 7 + 5];
    uint16_t drone_index;
    size_t offset = 0;
    size_t body_offset;
    FILE* fp;
    int fd;

    /* header of the Skybrush binary show file */
    offset = make_skyb_header(file, 0);

    /* show metadata block */
    body_offset = offset + 3;
    file[offset++] = SB_BINARY_BLOCK_SHOW_METADATA;
    file[offset++] = 12;
    file[offset++] = 0x00;

    /* the body of the block must end exactly at the end of the file */
    TEST_ASSERT_EQUAL(sizeof(file), offset + 12);

    WRITE_SHOW_ID(file, body_offset, show_id);
    WRITE_DRONE_INDEX(file, body_offset, 999);

    fp = tmpfile();
    TEST_ASSERT_NOT_NULL(fp);
    TEST_ASSERT_EQUAL(sizeof(file), fwrite(file, 1, sizeof(file), fp));
    fflush(fp);

    fd = fileno(fp);
    TEST_ASSERT_GREATER_OR_EQUAL(0, fd);

    /* the file descriptor must be rewound to the beginning of the file
     * because the parser reads from the current position of the descriptor */
    TEST_ASSERT_EQUAL(0, lseek(fd, 0, SEEK_SET));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_init(&metadata));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_update_from_binary_file(&metadata, fd));

    fclose(fp);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_get_show_id(&metadata, result));
    TEST_ASSERT_EQUAL_MEMORY(show_id, result, sizeof(show_id));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_get_drone_index(&metadata, &drone_index));
    TEST_ASSERT_EQUAL_UINT16(999, drone_index);

    sb_show_metadata_destroy(&metadata);
}

void test_update_from_binary_file_without_metadata_block(void)
{
    sb_show_metadata_t metadata;
    uint8_t show_id[SB_SHOW_METADATA_SHOW_ID_LENGTH] = { 4, 3, 2, 1 };
    uint8_t result[SB_SHOW_METADATA_SHOW_ID_LENGTH];
    uint8_t file[5 + 3 + 3];
    uint16_t drone_index;
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

    /* fill the object with some values first so that we can check that it
     * is left unmodified when the block is not found */
    {
        uint8_t body[7 + 5];
        size_t offset = 0;

        WRITE_SHOW_ID(body, offset, show_id);
        WRITE_DRONE_INDEX(body, offset, 7);

        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_init(&metadata));
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_update_from_buffer(&metadata, body, offset));
    }

    TEST_ASSERT_EQUAL(
        SB_ENOENT, sb_show_metadata_update_from_binary_file_in_memory(&metadata, file, sizeof(file)));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_get_show_id(&metadata, result));
    TEST_ASSERT_EQUAL_MEMORY(show_id, result, sizeof(show_id));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_get_drone_index(&metadata, &drone_index));
    TEST_ASSERT_EQUAL_UINT16(7, drone_index);

    sb_show_metadata_destroy(&metadata);
}

void test_destroy(void)
{
    sb_show_metadata_t metadata;

    /* destroying a null object is a no-op */
    sb_show_metadata_destroy(0);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_init(&metadata));
    sb_show_metadata_destroy(&metadata);

    /* the object must be reset to a state where it cannot be used */
    assert_defaults(&metadata);

    /* calling destroy multiple times is safe */
    sb_show_metadata_destroy(&metadata);
    sb_show_metadata_destroy(&metadata);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_init);
    RUN_TEST(test_clear);
    RUN_TEST(test_update_from_buffer);
    RUN_TEST(test_update_from_buffer_null_arguments);
    RUN_TEST(test_update_with_empty_body);
    RUN_TEST(test_update_with_missing_tags);
    RUN_TEST(test_update_skips_unknown_tags);
    RUN_TEST(test_update_with_duplicate_tags_last_one_wins);
    RUN_TEST(test_update_with_corrupted_body);
    RUN_TEST(test_update_from_binary_file_in_memory);
    RUN_TEST(test_update_from_binary_file);
    RUN_TEST(test_update_from_binary_file_without_metadata_block);
    RUN_TEST(test_destroy);

    return UNITY_END();
}
