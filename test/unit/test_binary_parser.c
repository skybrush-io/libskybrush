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

#include <stdio.h>
#include <unistd.h>

#include <skybrush/formats/binary.h>

#include "unity.h"

void setUp(void)
{
}

void tearDown(void)
{
}

void test_open_file(void)
{
    sb_binary_file_parser_t parser;
    FILE* fp;
    int fd;

    fp = fopen("fixtures/test.skyb", "rb");
    TEST_ASSERT(fp);

    fd = fileno(fp);
    TEST_ASSERT(fd >= 0);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_binary_file_parser_init_from_file(&parser, fd));

    TEST_ASSERT_EQUAL(1, sb_binary_file_parser_get_version(&parser));

    sb_binary_file_parser_destroy(&parser);
    fclose(fp);
}

void test_read_blocks_from_parser(sb_binary_file_parser_t* parser)
{
    sb_binary_block_t block;
    char buf[32];

    /* first block: trajectory */

    TEST_ASSERT_TRUE(sb_binary_file_is_current_block_valid(parser));

    block = sb_binary_file_get_current_block(parser);
    TEST_ASSERT_EQUAL(SB_BINARY_BLOCK_TRAJECTORY, block.type);
    TEST_ASSERT_EQUAL(36, block.length);
    TEST_ASSERT_EQUAL(8, block.start_of_body);

    /* seeking to next block */

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_binary_file_seek_to_next_block(parser));

    /* second block: comment */

    TEST_ASSERT_TRUE(sb_binary_file_is_current_block_valid(parser));

    block = sb_binary_file_get_current_block(parser);
    TEST_ASSERT_EQUAL(SB_BINARY_BLOCK_COMMENT, block.type);
    TEST_ASSERT_EQUAL(19, block.length);
    TEST_ASSERT_EQUAL(47, block.start_of_body);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_binary_file_read_current_block(parser, (uint8_t*)buf));
    buf[block.length] = 0;
    TEST_ASSERT_EQUAL_STRING("this is a test file", buf);

    /* seeking to next block */

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_binary_file_seek_to_next_block(parser));

    /* third block: light program */

    TEST_ASSERT_TRUE(sb_binary_file_is_current_block_valid(parser));

    block = sb_binary_file_get_current_block(parser);
    TEST_ASSERT_EQUAL(SB_BINARY_BLOCK_LIGHT_PROGRAM, block.type);
    TEST_ASSERT_EQUAL(27, block.length);
    TEST_ASSERT_EQUAL(69, block.start_of_body);

    /* seeking to next block */

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_binary_file_seek_to_next_block(parser));

    /* fourth block: yaw control */

    TEST_ASSERT_TRUE(sb_binary_file_is_current_block_valid(parser));

    block = sb_binary_file_get_current_block(parser);
    TEST_ASSERT_EQUAL(SB_BINARY_BLOCK_YAW_CONTROL, block.type);
    TEST_ASSERT_EQUAL(11, block.length);
    TEST_ASSERT_EQUAL(99, block.start_of_body);

    /* trying to read past EOF */

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_binary_file_seek_to_next_block(parser));
    TEST_ASSERT_FALSE(sb_binary_file_is_current_block_valid(parser));

    TEST_ASSERT_EQUAL(SB_EREAD, sb_binary_file_seek_to_next_block(parser));
}

void test_read_blocks_from_file(void)
{
    sb_binary_file_parser_t parser;
    FILE* fp;
    int fd;

    fp = fopen("fixtures/test.skyb", "rb");
    TEST_ASSERT(fp);

    fd = fileno(fp);
    TEST_ASSERT(fd >= 0);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_binary_file_parser_init_from_file(&parser, fd));
    test_read_blocks_from_parser(&parser);
    sb_binary_file_parser_destroy(&parser);

    fclose(fp);
}

void test_read_blocks_from_memory(void)
{
    sb_binary_file_parser_t parser;
    FILE* fp;
    uint8_t buf[4096];
    size_t nbytes;

    fp = fopen("fixtures/test.skyb", "rb");
    TEST_ASSERT(fp);
    nbytes = fread(buf, sizeof(uint8_t), sizeof(buf) / sizeof(uint8_t), fp);
    TEST_ASSERT(nbytes > 0);
    fclose(fp);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_binary_file_parser_init_from_buffer(&parser, buf, nbytes));
    test_read_blocks_from_parser(&parser);
    sb_binary_file_parser_destroy(&parser);
}

void test_find_first_block_by_type(void)
{
    sb_binary_file_parser_t parser;
    FILE* fp;
    int fd;

    fp = fopen("fixtures/test.skyb", "rb");
    TEST_ASSERT(fp);

    fd = fileno(fp);
    TEST_ASSERT(fd >= 0);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_binary_file_parser_init_from_file(&parser, fd));

    TEST_ASSERT_EQUAL(SB_ENOENT,
        sb_binary_file_find_first_block_by_type(
            &parser, SB_BINARY_BLOCK_NONE));

    TEST_ASSERT_EQUAL(SB_SUCCESS,
        sb_binary_file_find_first_block_by_type(
            &parser, SB_BINARY_BLOCK_COMMENT));
    TEST_ASSERT_EQUAL(SB_BINARY_BLOCK_COMMENT,
        sb_binary_file_get_current_block(&parser).type);

    TEST_ASSERT_EQUAL(SB_SUCCESS,
        sb_binary_file_find_first_block_by_type(
            &parser, SB_BINARY_BLOCK_LIGHT_PROGRAM));
    TEST_ASSERT_EQUAL(SB_BINARY_BLOCK_LIGHT_PROGRAM,
        sb_binary_file_get_current_block(&parser).type);

    TEST_ASSERT_EQUAL(SB_SUCCESS,
        sb_binary_file_find_first_block_by_type(
            &parser, SB_BINARY_BLOCK_TRAJECTORY));
    TEST_ASSERT_EQUAL(SB_BINARY_BLOCK_TRAJECTORY,
        sb_binary_file_get_current_block(&parser).type);

    TEST_ASSERT_EQUAL(SB_SUCCESS,
        sb_binary_file_find_first_block_by_type(
            &parser, SB_BINARY_BLOCK_YAW_CONTROL));
    TEST_ASSERT_EQUAL(SB_BINARY_BLOCK_YAW_CONTROL,
        sb_binary_file_get_current_block(&parser).type);

    sb_binary_file_parser_destroy(&parser);
    fclose(fp);
}

void test_read_truncated_block_from_memory(void)
{
    /* Regression test: a block that declares more bytes than the in-memory
     * buffer holds used to be returned as a view extending past the buffer */
    uint8_t buf[] = {
        's', 'k', 'y', 'b', 1,
        SB_BINARY_BLOCK_COMMENT, 5, 0, 'a', 'b'
    };
    sb_binary_file_parser_t parser;
    uint8_t* block_buf;
    size_t size;
    sb_bool_t owned;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_binary_file_parser_init_from_buffer(&parser, buf, sizeof(buf)));
    TEST_ASSERT_EQUAL(SB_BINARY_BLOCK_COMMENT, sb_binary_file_get_current_block(&parser).type);
    TEST_ASSERT_EQUAL(SB_EREAD, sb_binary_file_read_current_block_ex(&parser, &block_buf, &size, &owned));
    sb_binary_file_parser_destroy(&parser);
}

void test_read_block_past_end_from_memory(void)
{
    uint8_t buf[] = {
        's', 'k', 'y', 'b', 1,
        SB_BINARY_BLOCK_COMMENT, 2, 0, 'a', 'b'
    };
    sb_binary_file_parser_t parser;
    uint8_t* block_buf;
    size_t size;
    sb_bool_t owned;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_binary_file_parser_init_from_buffer(&parser, buf, sizeof(buf)));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_binary_file_read_current_block_ex(&parser, &block_buf, &size, &owned));
    TEST_ASSERT_EQUAL(2, size);
    TEST_ASSERT_FALSE(owned);
    TEST_ASSERT_EQUAL_MEMORY("ab", block_buf, 2);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_binary_file_seek_to_next_block(&parser));
    TEST_ASSERT_FALSE(sb_binary_file_is_current_block_valid(&parser));
    TEST_ASSERT_EQUAL(SB_EREAD, sb_binary_file_read_current_block_ex(&parser, &block_buf, &size, &owned));
    sb_binary_file_parser_destroy(&parser);
}

void test_skip_truncated_block(void)
{
    /* Regression test: skipping a block that extends beyond the end of the
     * file used to succeed for file descriptors (and report that there are
     * no more blocks) while it failed for in-memory buffers */
    uint8_t buf[] = {
        's', 'k', 'y', 'b', 1,
        SB_BINARY_BLOCK_COMMENT, 5, 0, 'a', 'b'
    };
    sb_binary_file_parser_t parser;
    FILE* fp;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_binary_file_parser_init_from_buffer(&parser, buf, sizeof(buf)));
    TEST_ASSERT_EQUAL(SB_EREAD, sb_binary_file_find_first_block_by_type(&parser, SB_BINARY_BLOCK_TRAJECTORY));
    sb_binary_file_parser_destroy(&parser);

    fp = tmpfile();
    TEST_ASSERT_NOT_NULL(fp);
    TEST_ASSERT_EQUAL(sizeof(buf), fwrite(buf, 1, sizeof(buf), fp));
    TEST_ASSERT_EQUAL(0, fflush(fp));
    TEST_ASSERT_EQUAL(0, lseek(fileno(fp), 0, SEEK_SET));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_binary_file_parser_init_from_file(&parser, fileno(fp)));
    TEST_ASSERT_EQUAL(SB_EREAD, sb_binary_file_find_first_block_by_type(&parser, SB_BINARY_BLOCK_TRAJECTORY));
    sb_binary_file_parser_destroy(&parser);

    fclose(fp);
}

int main(int argc, char* argv[])
{
    UNITY_BEGIN();

    RUN_TEST(test_open_file);
    RUN_TEST(test_read_blocks_from_file);
    RUN_TEST(test_read_blocks_from_memory);
    RUN_TEST(test_find_first_block_by_type);
    RUN_TEST(test_read_truncated_block_from_memory);
    RUN_TEST(test_read_block_past_end_from_memory);
    RUN_TEST(test_skip_truncated_block);

    return UNITY_END();
}
