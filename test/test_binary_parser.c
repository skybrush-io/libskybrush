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

void test_read_blocks_from_parser(sb_binary_file_parser_t *parser)
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

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_binary_file_read_current_block(parser, (uint8_t *)buf));
    buf[block.length] = 0;
    TEST_ASSERT_EQUAL_STRING("this is a test file", buf);

    /* trying to read past EOF */

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_binary_file_seek_to_next_block(parser));
    TEST_ASSERT_FALSE(sb_binary_file_is_current_block_valid(parser));

    TEST_ASSERT_EQUAL(SB_EREAD, sb_binary_file_seek_to_next_block(parser));
}

void test_read_blocks_from_file()
{
    sb_binary_file_parser_t parser;
    FILE *fp;
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

void test_read_blocks_from_memory()
{
    sb_binary_file_parser_t parser;
    FILE *fp;
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

void test_find_first_block_by_type()
{
    sb_binary_file_parser_t parser;
    FILE *fp;
    int fd;

    fp = fopen("fixtures/test.skyb", "rb");
    TEST_ASSERT(fp);

    fd = fileno(fp);
    TEST_ASSERT(fd >= 0);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_binary_file_parser_init_from_file(&parser, fd));

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
    RUN_TEST(test_read_blocks_from_file);
    RUN_TEST(test_read_blocks_from_memory);
    RUN_TEST(test_find_first_block_by_type);

    return UNITY_END();
}