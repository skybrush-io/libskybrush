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

#include "../src/parsing.h"
#include <float.h>

#include "unity.h"

void setUp()
{
}

void tearDown()
{
}

void test_parse_int16()
{
    uint8_t buf[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0xff, 0xfe };
    size_t offset;

    offset = 0;
    TEST_ASSERT_EQUAL(0x0201, sb_parse_int16(buf, &offset));
    TEST_ASSERT_EQUAL(2, offset);

    offset = 1;
    TEST_ASSERT_EQUAL(0x0302, sb_parse_int16(buf, &offset));
    TEST_ASSERT_EQUAL(3, offset);

    offset = 3;
    TEST_ASSERT_EQUAL(0x0504, sb_parse_int16(buf, &offset));
    TEST_ASSERT_EQUAL(5, offset);

    TEST_ASSERT_EQUAL(-257, sb_parse_int16(buf, &offset));
    TEST_ASSERT_EQUAL(7, offset);
}

void test_parse_uint16()
{
    uint8_t buf[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0xff, 0xfe };
    size_t offset;

    offset = 0;
    TEST_ASSERT_EQUAL(0x0201, sb_parse_uint16(buf, &offset));
    TEST_ASSERT_EQUAL(2, offset);

    offset = 1;
    TEST_ASSERT_EQUAL(0x0302, sb_parse_uint16(buf, &offset));
    TEST_ASSERT_EQUAL(3, offset);

    offset = 3;
    TEST_ASSERT_EQUAL(0x0504, sb_parse_uint16(buf, &offset));
    TEST_ASSERT_EQUAL(5, offset);

    TEST_ASSERT_EQUAL(0xfeff, sb_parse_uint16(buf, &offset));
    TEST_ASSERT_EQUAL(7, offset);
}

void test_parse_int32()
{
    uint8_t buf[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0xff, 0xfe };
    size_t offset;

    offset = 0;
    TEST_ASSERT_EQUAL(0x04030201, sb_parse_int32(buf, &offset));
    TEST_ASSERT_EQUAL(4, offset);

    offset = 1;
    TEST_ASSERT_EQUAL(0x05040302, sb_parse_int32(buf, &offset));
    TEST_ASSERT_EQUAL(5, offset);

    offset = 3;
    TEST_ASSERT_EQUAL(-16841468, sb_parse_int32(buf, &offset));
    TEST_ASSERT_EQUAL(7, offset);
}

void test_parse_uint32()
{
    uint8_t buf[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0xff, 0xfe };
    size_t offset;

    offset = 0;
    TEST_ASSERT_EQUAL(0x04030201, sb_parse_uint32(buf, &offset));
    TEST_ASSERT_EQUAL(4, offset);

    offset = 1;
    TEST_ASSERT_EQUAL(0x05040302, sb_parse_uint32(buf, &offset));
    TEST_ASSERT_EQUAL(5, offset);

    offset = 3;
    TEST_ASSERT_EQUAL(0xfeff0504, sb_parse_uint32(buf, &offset));
    TEST_ASSERT_EQUAL(7, offset);
}

void test_parse_varuint32()
{
    uint8_t buf[] = { 0x00, 0x01, 0x40, 0x7f, 0x80, 0x02, 0xa7, 0x82, 0x04, 0xff, 0xff, 0xff, 0xff, 0x0d, 0xff, 0xff, 0xff, 0xff, 0x0f };
    uint8_t overflow_buf[] = { 0x80, 0x80, 0x80, 0x80, 0x10, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00 };
    uint8_t partial_buf_1[] = { 0x80, 0x80 };
    uint8_t partial_buf_2[] = { 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80 };
    size_t offset;
    uint32_t value;

    offset = 0;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_parse_varuint32(buf, sizeof(buf), &offset, &value));
    TEST_ASSERT_EQUAL(1, offset);
    TEST_ASSERT_EQUAL(0, value);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_parse_varuint32(buf, sizeof(buf), &offset, &value));
    TEST_ASSERT_EQUAL(2, offset);
    TEST_ASSERT_EQUAL(1, value);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_parse_varuint32(buf, sizeof(buf), &offset, &value));
    TEST_ASSERT_EQUAL(3, offset);
    TEST_ASSERT_EQUAL(0x40, value);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_parse_varuint32(buf, sizeof(buf), &offset, &value));
    TEST_ASSERT_EQUAL(4, offset);
    TEST_ASSERT_EQUAL(0x7f, value);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_parse_varuint32(buf, sizeof(buf), &offset, &value));
    TEST_ASSERT_EQUAL(6, offset);
    TEST_ASSERT_EQUAL(0x100, value);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_parse_varuint32(buf, sizeof(buf), &offset, &value));
    TEST_ASSERT_EQUAL(9, offset);
    TEST_ASSERT_EQUAL(0x10127, value);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_parse_varuint32(buf, sizeof(buf), &offset, &value));
    TEST_ASSERT_EQUAL(14, offset);
    TEST_ASSERT_EQUAL(0xdfffffff, value);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_parse_varuint32(buf, sizeof(buf), &offset, &value));
    TEST_ASSERT_EQUAL(19, offset);
    TEST_ASSERT_EQUAL(0xffffffff, value);

    offset = 0;
    TEST_ASSERT_EQUAL(SB_EOVERFLOW, sb_parse_varuint32(overflow_buf, sizeof(overflow_buf), &offset, &value));
    TEST_ASSERT_EQUAL(5, offset);
    TEST_ASSERT_EQUAL(SB_EOVERFLOW, sb_parse_varuint32(overflow_buf, sizeof(overflow_buf), &offset, &value));
    TEST_ASSERT_EQUAL(12, offset);

    offset = 0;
    TEST_ASSERT_EQUAL(SB_EPARSE, sb_parse_varuint32(partial_buf_1, sizeof(partial_buf_1), &offset, &value));
    TEST_ASSERT_EQUAL(2, offset);

    offset = 0;
    TEST_ASSERT_EQUAL(SB_EPARSE, sb_parse_varuint32(partial_buf_2, sizeof(partial_buf_2), &offset, &value));
    TEST_ASSERT_EQUAL(7, offset);
}

int main(int argc, char* argv[])
{
    UNITY_BEGIN();

    RUN_TEST(test_parse_int16);
    RUN_TEST(test_parse_int32);
    RUN_TEST(test_parse_uint16);
    RUN_TEST(test_parse_uint32);
    RUN_TEST(test_parse_varuint32);

    return UNITY_END();
}
