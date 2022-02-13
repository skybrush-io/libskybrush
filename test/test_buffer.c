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

#include <skybrush/buffer.h>
#include <string.h>

#include "unity.h"

void setUp()
{
}

void tearDown()
{
}

void test_init_destroy()
{
    sb_buffer_t buf;
    size_t i;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_init(&buf, 8));
    TEST_ASSERT(!sb_buffer_is_view(&buf));
    TEST_ASSERT_EQUAL(8, sb_buffer_size(&buf));
    TEST_ASSERT_GREATER_OR_EQUAL(8, sb_buffer_capacity(&buf));
    for (i = 0; i < 8; i++) {
        TEST_ASSERT_EQUAL(0, SB_BUFFER(buf)[i]);
    }
    sb_buffer_destroy(&buf);
}

void test_init_destroy_zero_size()
{
    sb_buffer_t buf;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_init(&buf, 0));
    TEST_ASSERT(!sb_buffer_is_view(&buf));
    TEST_ASSERT_EQUAL(0, sb_buffer_size(&buf));
    TEST_ASSERT_GREATER_OR_EQUAL(1, sb_buffer_capacity(&buf));
    sb_buffer_destroy(&buf);
}

void test_clear_and_prune()
{
    sb_buffer_t buf;
    size_t capacity;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_init(&buf, 8));
    TEST_ASSERT_EQUAL(8, sb_buffer_size(&buf));
    capacity = sb_buffer_capacity(&buf);

    sb_buffer_clear(&buf);

    TEST_ASSERT_EQUAL(0, sb_buffer_size(&buf));
    TEST_ASSERT_EQUAL(capacity, sb_buffer_capacity(&buf));

    sb_buffer_prune(&buf);

    TEST_ASSERT_EQUAL(0, sb_buffer_size(&buf));
    TEST_ASSERT_EQUAL(1, sb_buffer_capacity(&buf));

    sb_buffer_destroy(&buf);
}

void test_resize_same_or_larger()
{
    sb_buffer_t buf;
    size_t i, capacity;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_init(&buf, 8));
    TEST_ASSERT_EQUAL(8, sb_buffer_size(&buf));
    capacity = sb_buffer_capacity(&buf);
    TEST_ASSERT_GREATER_OR_EQUAL(8, capacity);

    SB_BUFFER(buf)
    [0] = 42;
    SB_BUFFER(buf)
    [1] = 84;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_resize(&buf, capacity + 4));
    TEST_ASSERT_EQUAL(capacity + 4, sb_buffer_size(&buf));
    TEST_ASSERT_GREATER_OR_EQUAL(capacity + 4, sb_buffer_capacity(&buf));

    TEST_ASSERT_EQUAL(42, SB_BUFFER(buf)[0]);
    TEST_ASSERT_EQUAL(84, SB_BUFFER(buf)[1]);
    for (i = 2; i < sb_buffer_size(&buf); i++) {
        TEST_ASSERT_EQUAL(0, SB_BUFFER(buf)[i]);
    }

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_resize(&buf, capacity + 4));
    TEST_ASSERT_EQUAL(capacity + 4, sb_buffer_size(&buf));
    TEST_ASSERT_GREATER_OR_EQUAL(capacity + 4, sb_buffer_capacity(&buf));

    TEST_ASSERT_EQUAL(42, SB_BUFFER(buf)[0]);
    TEST_ASSERT_EQUAL(84, SB_BUFFER(buf)[1]);
    for (i = 2; i < sb_buffer_size(&buf); i++) {
        TEST_ASSERT_EQUAL(0, SB_BUFFER(buf)[i]);
    }

    sb_buffer_prune(&buf);

    TEST_ASSERT_EQUAL(capacity + 4, sb_buffer_size(&buf));
    TEST_ASSERT_EQUAL(capacity + 4, sb_buffer_capacity(&buf));

    TEST_ASSERT_EQUAL(42, SB_BUFFER(buf)[0]);
    TEST_ASSERT_EQUAL(84, SB_BUFFER(buf)[1]);
    for (i = 2; i < sb_buffer_size(&buf); i++) {
        TEST_ASSERT_EQUAL(0, SB_BUFFER(buf)[i]);
    }

    sb_buffer_destroy(&buf);
}

void test_resize_smaller()
{
    sb_buffer_t buf;
    size_t i, capacity;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_init(&buf, 8));
    TEST_ASSERT_EQUAL(8, sb_buffer_size(&buf));
    capacity = sb_buffer_capacity(&buf);
    TEST_ASSERT_GREATER_OR_EQUAL(8, capacity);

    SB_BUFFER(buf)
    [0] = 42;
    SB_BUFFER(buf)
    [1] = 84;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_resize(&buf, 4));
    TEST_ASSERT_EQUAL(4, sb_buffer_size(&buf));
    TEST_ASSERT_EQUAL(capacity, sb_buffer_capacity(&buf));

    sb_buffer_prune(&buf);

    TEST_ASSERT_EQUAL(4, sb_buffer_size(&buf));
    TEST_ASSERT_EQUAL(4, sb_buffer_capacity(&buf));

    TEST_ASSERT_EQUAL(42, SB_BUFFER(buf)[0]);
    TEST_ASSERT_EQUAL(84, SB_BUFFER(buf)[1]);
    for (i = 2; i < sb_buffer_size(&buf); i++) {
        TEST_ASSERT_EQUAL(0, SB_BUFFER(buf)[i]);
    }

    sb_buffer_destroy(&buf);
}

void test_resize_too_large()
{
    sb_buffer_t buf;
    const char* s = "dummy";

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_init(&buf, 8));
    TEST_ASSERT_EQUAL(8, sb_buffer_size(&buf));

    /* this will trigger an overflow when calculating the new size */
    TEST_ASSERT_EQUAL(SB_ENOMEM, sb_buffer_append_bytes(&buf, s, SIZE_MAX));

    /* this will test another code path */
    sb_buffer_clear(&buf);
    sb_buffer_prune(&buf);
    TEST_ASSERT_EQUAL(SB_ENOMEM, sb_buffer_append_bytes(&buf, s, SIZE_MAX));
}

void test_fill()
{
    sb_buffer_t buf;
    size_t i;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_init(&buf, 16));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_resize(&buf, 8));

    /* now the capacity is at least 16, but the size is only 8. Filling the
     * buffer should fill the first 8 bytes but not the rest */
    sb_buffer_fill(&buf, 42);
    for (i = 0; i < 8; i++) {
        TEST_ASSERT_EQUAL(42, SB_BUFFER(buf)[i]);
    }
    for (i = 8; i < 16; i++) {
        TEST_ASSERT_EQUAL(0, SB_BUFFER(buf)[i]);
    }

    sb_buffer_fill(&buf, 7);
    for (i = 0; i < 8; i++) {
        TEST_ASSERT_EQUAL(7, SB_BUFFER(buf)[i]);
    }
    for (i = 8; i < 16; i++) {
        TEST_ASSERT_EQUAL(0, SB_BUFFER(buf)[i]);
    }

    sb_buffer_destroy(&buf);
}

void test_append()
{
    sb_buffer_t buf, other;
    const char* str = "hello world";
    const char* str2 = "Hey ";

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_init(&buf, 0));
    TEST_ASSERT_EQUAL(0, sb_buffer_size(&buf));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_append_byte(&buf, str[0]));
    TEST_ASSERT_EQUAL(1, sb_buffer_size(&buf));
    TEST_ASSERT_EQUAL(str[0], SB_BUFFER(buf)[0]);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_append_bytes(&buf, str + 1, strlen(str) - 1));
    TEST_ASSERT_EQUAL(11, sb_buffer_size(&buf));
    TEST_ASSERT_EQUAL(0, memcmp(str, SB_BUFFER(buf), strlen(str)));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_init(&other, 0));
    TEST_ASSERT_EQUAL(0, sb_buffer_size(&other));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_append_bytes(&other, str2, strlen(str2)));
    TEST_ASSERT_EQUAL(4, sb_buffer_size(&other));
    TEST_ASSERT_EQUAL(0, memcmp(str2, SB_BUFFER(other), strlen(str2)));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_concat(&other, &buf));
    TEST_ASSERT_EQUAL(15, sb_buffer_size(&other));
    TEST_ASSERT_EQUAL(0, memcmp(str2, SB_BUFFER(other), strlen(str2)));
    TEST_ASSERT_EQUAL(0, memcmp(str, SB_BUFFER(other) + 4, strlen(str)));

    sb_buffer_destroy(&other);
    sb_buffer_destroy(&buf);
}

void test_append_zero_length()
{
    sb_buffer_t buf;
    const char* str = "hello world";

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_init(&buf, 0));
    TEST_ASSERT_EQUAL(0, sb_buffer_size(&buf));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_append_bytes(&buf, str, strlen(str)));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_append_bytes(&buf, str, 0));
    TEST_ASSERT_EQUAL(11, sb_buffer_size(&buf));
    TEST_ASSERT_EQUAL(0, memcmp(str, SB_BUFFER(buf), strlen(str)));

    sb_buffer_destroy(&buf);
}

void test_init_view()
{
    sb_buffer_t buf;
    char* str = "hello world";

    sb_buffer_init_view(&buf, str, strlen(str));
    TEST_ASSERT(sb_buffer_is_view(&buf));
    TEST_ASSERT_EQUAL(strlen(str), sb_buffer_size(&buf));
    TEST_ASSERT_EQUAL(0, memcmp(str, SB_BUFFER(buf), strlen(str)));
    TEST_ASSERT_EQUAL(str, SB_BUFFER(buf));
}

void test_init_view_cannot_grow_or_shrink()
{
    sb_buffer_t buf;
    char* str = "hello world";

    sb_buffer_init_view(&buf, str, strlen(str));
    TEST_ASSERT_EQUAL(SB_FAILURE, sb_buffer_resize(&buf, sb_buffer_size(&buf) + 1));
    TEST_ASSERT_EQUAL(SB_FAILURE, sb_buffer_resize(&buf, sb_buffer_size(&buf) - 1));
    TEST_ASSERT_EQUAL(SB_FAILURE, sb_buffer_clear(&buf));
}

void test_init_from_bytes()
{
    sb_buffer_t buf;
    const char* str = "hello world";

    char* str2 = malloc(32);
    strcpy(str2, str);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_init_from_bytes(&buf, str2, strlen(str2)));
    /* str2 ownership taken by 'buf' */

    TEST_ASSERT(!sb_buffer_is_view(&buf));
    TEST_ASSERT_EQUAL(strlen(str), sb_buffer_size(&buf));
    TEST_ASSERT_EQUAL(0, memcmp(str, SB_BUFFER(buf), strlen(str)));
    TEST_ASSERT_EQUAL(str2, SB_BUFFER(buf));

    sb_buffer_destroy(&buf);
}

void test_init_from_bytes_zero_size()
{
    sb_buffer_t buf;
    char* str = "hello world";

    TEST_ASSERT_EQUAL(SB_EINVAL, sb_buffer_init_from_bytes(&buf, str, 0));
}

int main(int argc, char* argv[])
{
    UNITY_BEGIN();

    RUN_TEST(test_init_destroy);
    RUN_TEST(test_init_destroy_zero_size);
    RUN_TEST(test_clear_and_prune);
    RUN_TEST(test_resize_same_or_larger);
    RUN_TEST(test_resize_smaller);
    RUN_TEST(test_resize_too_large);
    RUN_TEST(test_fill);
    RUN_TEST(test_append);
    RUN_TEST(test_append_zero_length);

    RUN_TEST(test_init_view);
    RUN_TEST(test_init_view_cannot_grow_or_shrink);

    RUN_TEST(test_init_from_bytes);
    RUN_TEST(test_init_from_bytes_zero_size);

    return UNITY_END();
}
