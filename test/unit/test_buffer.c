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

#include <skybrush/buffer.h>
#include <string.h>

#include "unity.h"

void setUp(void)
{
}

void tearDown(void)
{
}

void test_init_destroy(void)
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

void test_init_destroy_zero_size(void)
{
    sb_buffer_t buf;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_init(&buf, 0));
    TEST_ASSERT(!sb_buffer_is_view(&buf));
    TEST_ASSERT_EQUAL(0, sb_buffer_size(&buf));
    TEST_ASSERT_GREATER_OR_EQUAL(1, sb_buffer_capacity(&buf));
    sb_buffer_destroy(&buf);
}

void test_clear_and_prune(void)
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

void test_resize_same_or_larger(void)
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

void test_resize_smaller(void)
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

void test_resize_too_large(void)
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

    sb_buffer_destroy(&buf);
}

void test_reserve_same_or_larger(void)
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

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_reserve(&buf, capacity + 4));
    TEST_ASSERT_EQUAL(8, sb_buffer_size(&buf));
    TEST_ASSERT_GREATER_OR_EQUAL(capacity + 4, sb_buffer_capacity(&buf));

    TEST_ASSERT_EQUAL(42, SB_BUFFER(buf)[0]);
    TEST_ASSERT_EQUAL(84, SB_BUFFER(buf)[1]);
    for (i = 2; i < sb_buffer_size(&buf); i++) {
        TEST_ASSERT_EQUAL(0, SB_BUFFER(buf)[i]);
    }

    capacity = sb_buffer_capacity(&buf);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_reserve(&buf, capacity));
    TEST_ASSERT_EQUAL(8, sb_buffer_size(&buf));
    TEST_ASSERT_GREATER_OR_EQUAL(capacity, sb_buffer_capacity(&buf));

    TEST_ASSERT_EQUAL(42, SB_BUFFER(buf)[0]);
    TEST_ASSERT_EQUAL(84, SB_BUFFER(buf)[1]);
    for (i = 2; i < sb_buffer_size(&buf); i++) {
        TEST_ASSERT_EQUAL(0, SB_BUFFER(buf)[i]);
    }

    sb_buffer_destroy(&buf);
}

void test_reserve_smaller(void)
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

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_reserve(&buf, capacity - 2));
    TEST_ASSERT_EQUAL(8, sb_buffer_size(&buf));
    TEST_ASSERT_EQUAL(capacity, sb_buffer_capacity(&buf));

    TEST_ASSERT_EQUAL(42, SB_BUFFER(buf)[0]);
    TEST_ASSERT_EQUAL(84, SB_BUFFER(buf)[1]);
    for (i = 2; i < sb_buffer_size(&buf); i++) {
        TEST_ASSERT_EQUAL(0, SB_BUFFER(buf)[i]);
    }

    sb_buffer_destroy(&buf);
}

void test_fill(void)
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

void test_append(void)
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

void test_append_bytes_one_by_one(void)
{
    /* Appending a single byte to a full buffer must grow the buffer. The
     * buffer starts with a capacity of one byte so every power of two is a
     * point where the buffer is full */
    sb_buffer_t buf;
    int i;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_init(&buf, 0));

    for (i = 0; i < 100; i++) {
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_append_byte(&buf, (uint8_t)i));
        TEST_ASSERT_EQUAL(i + 1, sb_buffer_size(&buf));
        TEST_ASSERT_TRUE(sb_buffer_capacity(&buf) >= sb_buffer_size(&buf));
    }

    for (i = 0; i < 100; i++) {
        TEST_ASSERT_EQUAL_UINT8(i, SB_BUFFER(buf)[i]);
    }

    sb_buffer_destroy(&buf);
}

void test_append_zero_length(void)
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

void test_extend_with_zeros(void)
{
    sb_buffer_t buf;
    const char* str = "hello world";
    size_t i;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_init(&buf, 0));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_append_bytes(&buf, str, strlen(str)));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_extend_with_zeros(&buf, 0));
    TEST_ASSERT_EQUAL(strlen(str), sb_buffer_size(&buf));
    TEST_ASSERT_EQUAL(0, memcmp(str, SB_BUFFER(buf), strlen(str)));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_extend_with_zeros(&buf, 10));
    TEST_ASSERT_EQUAL(strlen(str) + 10, sb_buffer_size(&buf));
    TEST_ASSERT_EQUAL(0, strcmp(str, (char*)SB_BUFFER(buf)));

    for (i = strlen(str); i < sb_buffer_size(&buf); i++) {
        TEST_ASSERT_EQUAL(0, SB_BUFFER(buf)[i]);
    }

    sb_buffer_destroy(&buf);
}

void test_init_view(void)
{
    sb_buffer_t buf;
    char* str = "hello world";

    sb_buffer_init_view(&buf, str, strlen(str));
    TEST_ASSERT(sb_buffer_is_view(&buf));
    TEST_ASSERT_EQUAL(strlen(str), sb_buffer_size(&buf));
    TEST_ASSERT_EQUAL(0, memcmp(str, SB_BUFFER(buf), strlen(str)));
    TEST_ASSERT_EQUAL(str, SB_BUFFER(buf));
}

void test_view_cannot_grow_but_can_shrink(void)
{
    sb_buffer_t buf;
    char* str = "hello world";

    sb_buffer_init_view(&buf, str, strlen(str));
    TEST_ASSERT_EQUAL(SB_EPERM, sb_buffer_resize(&buf, sb_buffer_size(&buf) + 1));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_resize(&buf, sb_buffer_size(&buf) - 1));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_clear(&buf));
}

void test_init_from_bytes(void)
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

void test_init_from_bytes_zero_size(void)
{
    sb_buffer_t buf;
    char* bytes = malloc(1);

    TEST_ASSERT_NOT_NULL(bytes);

    /* The buffer takes ownership of the array even if it is empty */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_init_from_bytes(&buf, bytes, 0));
    TEST_ASSERT(!sb_buffer_is_view(&buf));
    TEST_ASSERT_EQUAL(0, sb_buffer_size(&buf));
    TEST_ASSERT_EQUAL(0, sb_buffer_capacity(&buf));

    /* The buffer can grow from zero capacity */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_append_bytes(&buf, "abc", 3));
    TEST_ASSERT_EQUAL(3, sb_buffer_size(&buf));
    TEST_ASSERT_EQUAL(0, memcmp("abc", SB_BUFFER(buf), 3));

    sb_buffer_destroy(&buf);
}

void test_init_from_bytes_null(void)
{
    sb_buffer_t buf;

    TEST_ASSERT_EQUAL(SB_EINVAL, sb_buffer_init_from_bytes(&buf, NULL, 0));
    TEST_ASSERT_EQUAL(SB_EINVAL, sb_buffer_init_from_bytes(&buf, NULL, 10));
}

void test_ensure_owned(void)
{
    sb_buffer_t buf;
    char* str = "hello world";

    sb_buffer_init_view(&buf, str, strlen(str));

    TEST_ASSERT(sb_buffer_is_view(&buf));
    TEST_ASSERT((void*)str == (void*)SB_BUFFER(buf));
    TEST_ASSERT_EQUAL(strlen(str), sb_buffer_size(&buf));
    TEST_ASSERT_EQUAL(0, memcmp(str, SB_BUFFER(buf), strlen(str)));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_ensure_owned(&buf));
    TEST_ASSERT(!sb_buffer_is_view(&buf));
    TEST_ASSERT_EQUAL(strlen(str), sb_buffer_size(&buf));
    TEST_ASSERT_EQUAL(0, memcmp(str, SB_BUFFER(buf), strlen(str)));
    TEST_ASSERT((void*)str != (void*)SB_BUFFER(buf));

    sb_buffer_destroy(&buf);
}

void test_update_from_owned_buffer(void)
{
    sb_buffer_t src, dest;
    char* str = "hello world";

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_init(&src, 0));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_append_bytes(&src, str, strlen(str)));

    /* dest is an owned buffer with enough capacity; storage should be reused */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_init(&dest, 32));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_update(&dest, &src));
    TEST_ASSERT(!sb_buffer_is_view(&dest));
    TEST_ASSERT_EQUAL(strlen(str), sb_buffer_size(&dest));
    TEST_ASSERT_EQUAL(0, memcmp(str, SB_BUFFER(dest), strlen(str)));
    sb_buffer_destroy(&dest);

    /* dest is an owned buffer that needs to grow */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_init(&dest, 2));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_update(&dest, &src));
    TEST_ASSERT(!sb_buffer_is_view(&dest));
    TEST_ASSERT_EQUAL(strlen(str), sb_buffer_size(&dest));
    TEST_ASSERT_GREATER_OR_EQUAL(strlen(str), sb_buffer_capacity(&dest));
    TEST_ASSERT_EQUAL(0, memcmp(str, SB_BUFFER(dest), strlen(str)));
    sb_buffer_destroy(&dest);

    /* dest is a view */
    char other[16] = "xxxxxxxxxxxxxxxx";
    sb_buffer_init_view(&dest, other, sizeof(other));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_update(&dest, &src));
    TEST_ASSERT(!sb_buffer_is_view(&dest));
    TEST_ASSERT_EQUAL(strlen(str), sb_buffer_size(&dest));
    TEST_ASSERT_EQUAL(0, memcmp(str, SB_BUFFER(dest), strlen(str)));
    TEST_ASSERT((void*)other != (void*)SB_BUFFER(dest));
    TEST_ASSERT_EQUAL(0, memcmp("xxxxxxxxxxxxxxxx", other, sizeof(other)));
    sb_buffer_destroy(&dest);

    /* updating a buffer with itself is a no-op */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_update(&src, &src));
    TEST_ASSERT(!sb_buffer_is_view(&src));
    TEST_ASSERT_EQUAL(strlen(str), sb_buffer_size(&src));
    TEST_ASSERT_EQUAL(0, memcmp(str, SB_BUFFER(src), strlen(str)));

    sb_buffer_destroy(&src);
}

void test_update_from_view(void)
{
    sb_buffer_t src, dest;
    char* str = "hello world";

    sb_buffer_init_view(&src, str, strlen(str));

    /* dest is an owned buffer */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_init(&dest, 8));
    SB_BUFFER(dest)
    [0] = 42;
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_update(&dest, &src));
    TEST_ASSERT(sb_buffer_is_view(&dest));
    TEST_ASSERT_EQUAL((void*)str, (void*)SB_BUFFER(dest));
    TEST_ASSERT_EQUAL(strlen(str), sb_buffer_size(&dest));
    sb_buffer_destroy(&dest); /* must not free the storage of the view */

    /* dest is a view too */
    char other[16] = "xxxxxxxxxxxxxxxx";
    sb_buffer_init_view(&dest, other, sizeof(other));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_update(&dest, &src));
    TEST_ASSERT(sb_buffer_is_view(&dest));
    TEST_ASSERT_EQUAL((void*)str, (void*)SB_BUFFER(dest));
    TEST_ASSERT_EQUAL(strlen(str), sb_buffer_size(&dest));
    sb_buffer_destroy(&dest);

    sb_buffer_destroy(&src);
}

void test_update_from_view_of_dest(void)
{
    sb_buffer_t buf, view;
    char* str = "hello world";

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_init(&buf, 0));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_buffer_append_bytes(&buf, str, strlen(str)));

    sb_buffer_init_view(&view, SB_BUFFER(buf), sb_buffer_size(&buf));
    TEST_ASSERT_EQUAL(SB_EINVAL, sb_buffer_update(&buf, &view));

    /* buf must be left unchanged */
    TEST_ASSERT(!sb_buffer_is_view(&buf));
    TEST_ASSERT_EQUAL(strlen(str), sb_buffer_size(&buf));
    TEST_ASSERT_EQUAL(0, memcmp(str, SB_BUFFER(buf), strlen(str)));

    sb_buffer_destroy(&buf);
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
    RUN_TEST(test_reserve_same_or_larger);
    RUN_TEST(test_reserve_smaller);

    RUN_TEST(test_fill);
    RUN_TEST(test_append);
    RUN_TEST(test_append_bytes_one_by_one);
    RUN_TEST(test_append_zero_length);
    RUN_TEST(test_extend_with_zeros);

    RUN_TEST(test_init_view);
    RUN_TEST(test_view_cannot_grow_but_can_shrink);

    RUN_TEST(test_init_from_bytes);
    RUN_TEST(test_init_from_bytes_zero_size);
    RUN_TEST(test_init_from_bytes_null);

    RUN_TEST(test_ensure_owned);

    RUN_TEST(test_update_from_owned_buffer);
    RUN_TEST(test_update_from_view);
    RUN_TEST(test_update_from_view_of_dest);

    return UNITY_END();
}
