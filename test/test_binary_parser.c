#include <skybrush/formats/binary.h>

#include "unity.h"

void setUp()
{
}

void tearDown()
{
}

void test_open_file()
{
    sb_binary_file_parser_t parser;
    FILE *fp;

    fp = fopen("fixtures/test.skyb", "rb");

    TEST_ASSERT(fp);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_binary_file_parser_init(&parser, fp));
    TEST_ASSERT_EQUAL(1, sb_binary_file_parser_get_version(&parser));

    sb_binary_file_parser_destroy(&parser);
    fclose(fp);
}

void test_read_blocks()
{
    sb_binary_file_parser_t parser;
    sb_binary_block_t block;
    FILE *fp;
    char buf[32];

    fp = fopen("fixtures/test.skyb", "rb");

    TEST_ASSERT(fp);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_binary_file_parser_init(&parser, fp));

    /* first block: trajectory */

    TEST_ASSERT_TRUE(sb_binary_file_is_current_block_valid(&parser));

    block = sb_binary_file_get_current_block(&parser);
    TEST_ASSERT_EQUAL(SB_BINARY_BLOCK_TRAJECTORY, block.type);
    TEST_ASSERT_EQUAL(36, block.length);
    TEST_ASSERT_EQUAL(8, block.start_of_body);

    /* seeking to next block */

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_binary_file_seek_to_next_block(&parser));

    /* second block: comment */

    TEST_ASSERT_TRUE(sb_binary_file_is_current_block_valid(&parser));

    block = sb_binary_file_get_current_block(&parser);
    TEST_ASSERT_EQUAL(SB_BINARY_BLOCK_COMMENT, block.type);
    TEST_ASSERT_EQUAL(19, block.length);
    TEST_ASSERT_EQUAL(47, block.start_of_body);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_binary_file_read_current_block(&parser, (uint8_t *)buf));
    buf[block.length] = 0;
    TEST_ASSERT_EQUAL_STRING("this is a test file", buf);

    /* trying to read past EOF */

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_binary_file_seek_to_next_block(&parser));
    TEST_ASSERT_FALSE(sb_binary_file_is_current_block_valid(&parser));

    TEST_ASSERT_EQUAL(SB_EREAD, sb_binary_file_seek_to_next_block(&parser));

    sb_binary_file_parser_destroy(&parser);
    fclose(fp);
}

void test_find_first_block_by_type()
{
    sb_binary_file_parser_t parser;
    FILE *fp;

    fp = fopen("fixtures/test.skyb", "rb");

    TEST_ASSERT(fp);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_binary_file_parser_init(&parser, fp));

    TEST_ASSERT_EQUAL(SB_SUCCESS,
                      sb_binary_file_find_first_block_by_type(
                          &parser, SB_BINARY_BLOCK_COMMENT));
    TEST_ASSERT_EQUAL(SB_BINARY_BLOCK_COMMENT,
                      sb_binary_file_get_current_block(&parser).type);

    TEST_ASSERT_EQUAL(SB_FAILURE,
                      sb_binary_file_find_first_block_by_type(
                          &parser, SB_BINARY_BLOCK_LIGHT_PROGRAM));

    TEST_ASSERT_EQUAL(SB_SUCCESS,
                      sb_binary_file_find_first_block_by_type(
                          &parser, SB_BINARY_BLOCK_TRAJECTORY));
    TEST_ASSERT_EQUAL(SB_BINARY_BLOCK_TRAJECTORY,
                      sb_binary_file_get_current_block(&parser).type);

    sb_binary_file_parser_destroy(&parser);
    fclose(fp);
}

int main(int argc, char *argv[])
{
    UNITY_BEGIN();

    RUN_TEST(test_open_file);
    RUN_TEST(test_read_blocks);
    RUN_TEST(test_find_first_block_by_type);

    return UNITY_END();
}