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

#include <skybrush/formats/tlv.h>
#include <string.h>

#include "unity.h"

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

void test_init(void)
{
    sb_tlv_parser_t parser;
    uint8_t buf[1] = { 0x00 };

    TEST_ASSERT_EQUAL(SB_EINVAL, sb_tlv_parser_init(0, buf, sizeof(buf)));

    /* null buffer is okay if the size is zero */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_tlv_parser_init(&parser, 0, 0));
    TEST_ASSERT_EQUAL(SB_ENOENT, sb_tlv_parser_next(&parser, &(sb_tlv_entry_t) { 0 }));
    sb_tlv_parser_destroy(&parser);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_tlv_parser_init(&parser, buf, sizeof(buf)));
    TEST_ASSERT_EQUAL(SB_ECORRUPTED, sb_tlv_parser_next(&parser, &(sb_tlv_entry_t) { 0 }));
    sb_tlv_parser_destroy(&parser);
}

void test_init_null_buffer_with_nonzero_size(void)
{
    sb_tlv_parser_t parser;
    sb_tlv_entry_t entry;

    /* a null buffer with a positive size is invalid */
    TEST_ASSERT_EQUAL(SB_EINVAL, sb_tlv_parser_init(&parser, 0, 1));
    TEST_ASSERT_EQUAL(SB_EINVAL, sb_tlv_parser_init(&parser, 0, 100));

    /* the parser must not be modified on failure */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_tlv_parser_init(&parser, 0, 0));
    TEST_ASSERT_EQUAL(SB_EINVAL, sb_tlv_parser_init(&parser, 0, 100));
    TEST_ASSERT_EQUAL(SB_ENOENT, sb_tlv_parser_next(&parser, &entry));
    sb_tlv_parser_destroy(&parser);

    /* a parser that was tampered with so that it has a null buffer but a
     * positive size must be rejected by next() as well */
    parser.buf = 0;
    parser.size = 5;
    parser.offset = 0;
    TEST_ASSERT_EQUAL(SB_EINVAL, sb_tlv_parser_next(&parser, &entry));

    sb_tlv_parser_destroy(&parser);
}

void test_destroy(void)
{
    sb_tlv_parser_t parser;
    sb_tlv_entry_t entry;
    uint8_t buf[3] = { 0x01, 0x00, 0x00 };

    /* destroying a null parser is a no-op */
    sb_tlv_parser_destroy(0);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_tlv_parser_init(&parser, buf, sizeof(buf)));
    sb_tlv_parser_destroy(&parser);

    /* the parser must be reset to a state where it cannot be used */
    TEST_ASSERT_EQUAL(0, parser.buf);
    TEST_ASSERT_EQUAL(0, parser.size);
    TEST_ASSERT_EQUAL(0, parser.offset);
    TEST_ASSERT_EQUAL(SB_ENOENT, sb_tlv_parser_next(&parser, &entry));

    /* calling destroy multiple times is safe */
    sb_tlv_parser_destroy(&parser);
    sb_tlv_parser_destroy(&parser);
}

void test_next_invalid_args(void)
{
    sb_tlv_parser_t parser;
    sb_tlv_entry_t entry;
    uint8_t buf[3] = { 0x01, 0x00, 0x00 };

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_tlv_parser_init(&parser, buf, sizeof(buf)));

    TEST_ASSERT_EQUAL(SB_EINVAL, sb_tlv_parser_next(0, &entry));
    TEST_ASSERT_EQUAL(SB_EINVAL, sb_tlv_parser_next(&parser, 0));

    sb_tlv_parser_destroy(&parser);
}

void test_empty_stream(void)
{
    sb_tlv_parser_t parser;
    sb_tlv_entry_t entry;
    uint8_t buf[8] = { 0 }; /* content does not matter, size does */

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_tlv_parser_init(&parser, buf, 0));
    TEST_ASSERT_EQUAL(SB_ENOENT, sb_tlv_parser_next(&parser, &entry));
    sb_tlv_parser_destroy(&parser);
}

void test_single_entry(void)
{
    sb_tlv_parser_t parser;
    sb_tlv_entry_t entry;
    uint8_t value[4] = { 0xde, 0xad, 0xbe, 0xef };
    uint8_t buf[3 + sizeof(value)];
    size_t offset = 0;

    WRITE_ENTRY(buf, offset, 0x42, value, sizeof(value));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_tlv_parser_init(&parser, buf, sizeof(buf)));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_tlv_parser_next(&parser, &entry));

    TEST_ASSERT_EQUAL_HEX8(0x42, entry.tag);
    TEST_ASSERT_EQUAL(sizeof(value), entry.length);
    TEST_ASSERT_EQUAL_PTR(buf + 3, entry.value);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(value, entry.value, sizeof(value));

    /* no more entries */
    TEST_ASSERT_EQUAL(SB_ENOENT, sb_tlv_parser_next(&parser, &entry));
    TEST_ASSERT_EQUAL(SB_ENOENT, sb_tlv_parser_next(&parser, &entry));

    sb_tlv_parser_destroy(&parser);
}

void test_multiple_entries(void)
{
    sb_tlv_parser_t parser;
    sb_tlv_entry_t entry;
    uint8_t value1[2] = { 0x01, 0x02 };
    uint8_t value2[5] = { 0x0a, 0x0b, 0x0c, 0x0d, 0x0e };
    uint8_t value3[1] = { 0xff };
    uint8_t buf[3 + 2 + 3 + 5 + 3 + 1];
    size_t offset = 0;

    WRITE_ENTRY(buf, offset, 0x10, value1, sizeof(value1));
    WRITE_ENTRY(buf, offset, 0x20, value2, sizeof(value2));
    WRITE_ENTRY(buf, offset, 0x30, value3, sizeof(value3));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_tlv_parser_init(&parser, buf, sizeof(buf)));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_tlv_parser_next(&parser, &entry));
    TEST_ASSERT_EQUAL_HEX8(0x10, entry.tag);
    TEST_ASSERT_EQUAL(2, entry.length);
    TEST_ASSERT_EQUAL_PTR(buf + 3, entry.value);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(value1, entry.value, sizeof(value1));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_tlv_parser_next(&parser, &entry));
    TEST_ASSERT_EQUAL_HEX8(0x20, entry.tag);
    TEST_ASSERT_EQUAL(5, entry.length);
    TEST_ASSERT_EQUAL_PTR(buf + 8, entry.value);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(value2, entry.value, sizeof(value2));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_tlv_parser_next(&parser, &entry));
    TEST_ASSERT_EQUAL_HEX8(0x30, entry.tag);
    TEST_ASSERT_EQUAL(1, entry.length);
    TEST_ASSERT_EQUAL_PTR(buf + 16, entry.value);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(value3, entry.value, sizeof(value3));

    TEST_ASSERT_EQUAL(SB_ENOENT, sb_tlv_parser_next(&parser, &entry));

    sb_tlv_parser_destroy(&parser);
}

void test_zero_length_entry(void)
{
    sb_tlv_parser_t parser;
    sb_tlv_entry_t entry;
    uint8_t buf[3] = { 0x7f, 0x00, 0x00 };

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_tlv_parser_init(&parser, buf, sizeof(buf)));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_tlv_parser_next(&parser, &entry));
    TEST_ASSERT_EQUAL_HEX8(0x7f, entry.tag);
    TEST_ASSERT_EQUAL(0, entry.length);

    TEST_ASSERT_EQUAL(SB_ENOENT, sb_tlv_parser_next(&parser, &entry));

    sb_tlv_parser_destroy(&parser);
}

void test_length_is_little_endian(void)
{
    sb_tlv_parser_t parser;
    sb_tlv_entry_t entry;
    uint8_t buf[3 + 300];

    memset(buf + 3, 0xab, 300);
    buf[0] = 0x01;
    buf[1] = 0x2c; /* 300 = 0x012c, little-endian: 0x2c, 0x01 */
    buf[2] = 0x01;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_tlv_parser_init(&parser, buf, sizeof(buf)));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_tlv_parser_next(&parser, &entry));
    TEST_ASSERT_EQUAL(300, entry.length);
    TEST_ASSERT_EQUAL_PTR(buf + 3, entry.value);

    TEST_ASSERT_EQUAL(SB_ENOENT, sb_tlv_parser_next(&parser, &entry));

    sb_tlv_parser_destroy(&parser);
}

void test_truncated_header(void)
{
    sb_tlv_parser_t parser;
    sb_tlv_entry_t entry;
    uint8_t buf[2] = { 0x01, 0x00 };

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_tlv_parser_init(&parser, buf, sizeof(buf)));
    TEST_ASSERT_EQUAL(SB_ECORRUPTED, sb_tlv_parser_next(&parser, &entry));

    /* single byte only */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_tlv_parser_init(&parser, buf, 1));
    TEST_ASSERT_EQUAL(SB_ECORRUPTED, sb_tlv_parser_next(&parser, &entry));

    sb_tlv_parser_destroy(&parser);
}

void test_length_overrun(void)
{
    sb_tlv_parser_t parser;
    sb_tlv_entry_t entry;
    uint8_t buf[5] = { 0x01, 0x03, 0x00, 0xaa, 0xbb }; /* claims 3 bytes, has 2 */

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_tlv_parser_init(&parser, buf, sizeof(buf)));
    TEST_ASSERT_EQUAL(SB_ECORRUPTED, sb_tlv_parser_next(&parser, &entry));

    sb_tlv_parser_destroy(&parser);
}

void test_truncated_second_entry(void)
{
    sb_tlv_parser_t parser;
    sb_tlv_entry_t entry;
    uint8_t buf[4] = { 0x01, 0x00, 0x00, 0x02 }; /* first entry okay, second is truncated */

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_tlv_parser_init(&parser, buf, sizeof(buf)));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_tlv_parser_next(&parser, &entry));
    TEST_ASSERT_EQUAL_HEX8(0x01, entry.tag);
    TEST_ASSERT_EQUAL(0, entry.length);

    TEST_ASSERT_EQUAL(SB_ECORRUPTED, sb_tlv_parser_next(&parser, &entry));

    sb_tlv_parser_destroy(&parser);
}

void test_entries_with_unknown_tags_can_be_skipped(void)
{
    sb_tlv_parser_t parser;
    sb_tlv_entry_t entry;
    uint8_t value[2] = { 0x01, 0x02 };
    uint8_t buf[3 + 2 + 3 + 2];
    size_t offset = 0;

    WRITE_ENTRY(buf, offset, 0xfe, value, sizeof(value)); /* unknown tag */
    WRITE_ENTRY(buf, offset, 0x01, value, sizeof(value)); /* known tag */

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_tlv_parser_init(&parser, buf, sizeof(buf)));

    /* the parser returns every entry, including the unknown one; it is up
     * to the caller to skip it */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_tlv_parser_next(&parser, &entry));
    TEST_ASSERT_EQUAL_HEX8(0xfe, entry.tag);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_tlv_parser_next(&parser, &entry));
    TEST_ASSERT_EQUAL_HEX8(0x01, entry.tag);
    TEST_ASSERT_EQUAL(2, entry.length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(value, entry.value, sizeof(value));

    TEST_ASSERT_EQUAL(SB_ENOENT, sb_tlv_parser_next(&parser, &entry));

    sb_tlv_parser_destroy(&parser);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_init);
    RUN_TEST(test_init_null_buffer_with_nonzero_size);
    RUN_TEST(test_destroy);
    RUN_TEST(test_next_invalid_args);
    RUN_TEST(test_empty_stream);
    RUN_TEST(test_single_entry);
    RUN_TEST(test_multiple_entries);
    RUN_TEST(test_zero_length_entry);
    RUN_TEST(test_length_is_little_endian);
    RUN_TEST(test_truncated_header);
    RUN_TEST(test_length_overrun);
    RUN_TEST(test_truncated_second_entry);
    RUN_TEST(test_entries_with_unknown_tags_can_be_skipped);

    return UNITY_END();
}
