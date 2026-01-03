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

#include "unity.h"

#include <skybrush/screenplay.h>

void setUp(void)
{
    /* nothing to set up */
}

void tearDown(void)
{
    /* nothing to tear down */
}

void test_screenplay_init_sets_defaults_and_allocates(void)
{
    sb_screenplay_t screenplay;
    sb_error_t err;

    /* Initialize the screenplay */
    err = sb_screenplay_init(&screenplay);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    /* chapters storage must be allocated */
    TEST_ASSERT_NOT_NULL(screenplay.chapters);

    /* size must be zero, capacity at least 1 */
    TEST_ASSERT_EQUAL(0u, sb_screenplay_size(&screenplay));
    TEST_ASSERT_TRUE(sb_screenplay_capacity(&screenplay) >= 1u);

    /* getting chapter pointer must return NULL for empty screenplay */
    TEST_ASSERT_NULL(sb_screenplay_get_chapter_ptr(&screenplay, 0));
    TEST_ASSERT_NULL(sb_screenplay_get_chapter_ptr_const(&screenplay, 0));

    /* Clean up */
    sb_screenplay_destroy(&screenplay);

    /* After destroy, size and capacity must be zero */
    TEST_ASSERT_EQUAL(0u, sb_screenplay_size(&screenplay));
    TEST_ASSERT_EQUAL(0u, sb_screenplay_capacity(&screenplay));

    /* get chapter should still return NULL (num_chapters is zero) */
    TEST_ASSERT_NULL(sb_screenplay_get_chapter_ptr(&screenplay, 0));
}

void test_sb_screenplay_get_current_chapter_ptr_empty(void)
{
    sb_screenplay_t screenplay;
    sb_error_t err;
    uint32_t time_msec = 12345u;
    sb_screenplay_chapter_t* ptr;

    err = sb_screenplay_init(&screenplay);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    /* Empty screenplay -> must return NULL and set time_msec to 0 */
    ptr = sb_screenplay_get_current_chapter_ptr(&screenplay, &time_msec);
    TEST_ASSERT_NULL(ptr);
    TEST_ASSERT_EQUAL_UINT32(0u, time_msec);

    sb_screenplay_destroy(&screenplay);
}

void test_sb_screenplay_get_current_chapter_ptr_infinite_first(void)
{
    sb_screenplay_t screenplay;
    sb_error_t err;
    uint32_t time_msec = 5000u;
    const sb_screenplay_chapter_t* ptr;

    err = sb_screenplay_init(&screenplay);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    /* Append a single chapter (default is infinite duration) */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_append_chapter(&screenplay, NULL));
    /* Call with arbitrary time -> must return first chapter and leave time unchanged */
    ptr = sb_screenplay_get_current_chapter_ptr(&screenplay, &time_msec);
    TEST_ASSERT_NOT_NULL(ptr);
    TEST_ASSERT_EQUAL_PTR(sb_screenplay_get_chapter_ptr_const(&screenplay, 0), ptr);
    TEST_ASSERT_EQUAL_UINT32(5000u, time_msec);

    sb_screenplay_destroy(&screenplay);
}

void test_sb_screenplay_get_current_chapter_ptr_finite_offsets_and_overflow(void)
{
    sb_screenplay_t screenplay;
    sb_error_t err;
    uint32_t time_msec;
    sb_screenplay_chapter_t* ptr;

    err = sb_screenplay_init(&screenplay);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    /* Append three chapters and set durations: 1000, 2000, 3000 (all finite) */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_append_chapter(&screenplay, &ptr));
    sb_screenplay_chapter_set_duration_msec(ptr, 1000u);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_append_chapter(&screenplay, &ptr));
    sb_screenplay_chapter_set_duration_msec(ptr, 2000u);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_append_chapter(&screenplay, &ptr));
    sb_screenplay_chapter_set_duration_msec(ptr, 3000u);

    /* time within first chapter */
    time_msec = 500u;
    ptr = sb_screenplay_get_current_chapter_ptr(&screenplay, &time_msec);
    TEST_ASSERT_EQUAL_PTR(sb_screenplay_get_chapter_ptr(&screenplay, 0), ptr);
    TEST_ASSERT_EQUAL_UINT32(500u, time_msec);

    /* time exactly at end of first chapter and start of second */
    time_msec = 1000u;
    ptr = sb_screenplay_get_current_chapter_ptr(&screenplay, &time_msec);
    TEST_ASSERT_EQUAL_PTR(sb_screenplay_get_chapter_ptr(&screenplay, 1), ptr);
    TEST_ASSERT_EQUAL_UINT32(0u, time_msec);

    /* time within second chapter (1500 -> second chapter offset 500) */
    time_msec = 1500u;
    ptr = sb_screenplay_get_current_chapter_ptr(&screenplay, &time_msec);
    TEST_ASSERT_EQUAL_PTR(sb_screenplay_get_chapter_ptr(&screenplay, 1), ptr);
    TEST_ASSERT_EQUAL_UINT32(500u, time_msec);

    /* time exactly at end of all chapters -> must return NULL and set time to 0 */
    time_msec = 6000u; /* 1000 + 2000 + 3000 */
    ptr = sb_screenplay_get_current_chapter_ptr(&screenplay, &time_msec);
    TEST_ASSERT_NULL(ptr);
    TEST_ASSERT_EQUAL_UINT32(0u, time_msec);

    /* time beyond total duration -> NULL and time 0 */
    time_msec = 7000u;
    ptr = sb_screenplay_get_current_chapter_ptr(&screenplay, &time_msec);
    TEST_ASSERT_NULL(ptr);
    TEST_ASSERT_EQUAL_UINT32(0u, time_msec);

    sb_screenplay_destroy(&screenplay);
}

void test_sb_screenplay_get_current_chapter_ptr_with_infinite_later_chapter(void)
{
    sb_screenplay_t screenplay;
    sb_error_t err;
    uint32_t time_msec;
    sb_screenplay_chapter_t* ptr;

    err = sb_screenplay_init(&screenplay);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    /* Append three chapters and set durations: 1000, 2000, infinite */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_append_chapter(&screenplay, &ptr));
    sb_screenplay_chapter_set_duration_msec(ptr, 1000u);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_append_chapter(&screenplay, &ptr));
    sb_screenplay_chapter_set_duration_msec(ptr, 2000u);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_append_chapter(&screenplay, &ptr));
    /* chapter 2 (index 2) remains infinite by default */

    /* time that falls into the infinite third chapter: 1000+2000+500 -> should return third with offset 500 */
    time_msec = 3500u;
    ptr = sb_screenplay_get_current_chapter_ptr(&screenplay, &time_msec);
    TEST_ASSERT_EQUAL_PTR(sb_screenplay_get_chapter_ptr(&screenplay, 2), ptr);
    TEST_ASSERT_EQUAL_UINT32(500u, time_msec);

    sb_screenplay_destroy(&screenplay);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_screenplay_init_sets_defaults_and_allocates);
    RUN_TEST(test_sb_screenplay_get_current_chapter_ptr_empty);
    RUN_TEST(test_sb_screenplay_get_current_chapter_ptr_infinite_first);
    RUN_TEST(test_sb_screenplay_get_current_chapter_ptr_finite_offsets_and_overflow);
    RUN_TEST(test_sb_screenplay_get_current_chapter_ptr_with_infinite_later_chapter);

    return UNITY_END();
}
