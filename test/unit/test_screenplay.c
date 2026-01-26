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

    /* Initialize the screenplay */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_init(&screenplay));

    /* chapters storage must be allocated */
    TEST_ASSERT_NOT_NULL(screenplay.chapters);

    /* size must be zero, capacity at least 1 */
    TEST_ASSERT_EQUAL(0u, sb_screenplay_size(&screenplay));
    TEST_ASSERT_TRUE(sb_screenplay_capacity(&screenplay) >= 1u);

    /* getting chapter pointer must return NULL for empty screenplay */
    TEST_ASSERT_NULL(sb_screenplay_get_chapter_ptr(&screenplay, 0));

    /* Clean up */
    sb_screenplay_destroy(&screenplay);

    /* After destroy, size and capacity must be zero */
    TEST_ASSERT_EQUAL(0u, sb_screenplay_size(&screenplay));
    TEST_ASSERT_EQUAL(0u, sb_screenplay_capacity(&screenplay));

    /* get chapter should still return NULL (num_chapters is zero) */
    TEST_ASSERT_NULL(sb_screenplay_get_chapter_ptr(&screenplay, 0));
}

void test_sb_screenplay_get_chapter_ptr_at_time_msec_empty(void)
{
    sb_screenplay_t screenplay;
    sb_error_t err;
    uint32_t time_msec = 12345u;
    sb_screenplay_chapter_t* ptr;
    ssize_t chapter_index;

    err = sb_screenplay_init(&screenplay);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    /* Empty screenplay -> must return NULL and set time_msec to 0 */
    ptr = sb_screenplay_get_chapter_ptr_at_time_msec(&screenplay, &time_msec, NULL);
    TEST_ASSERT_NULL(ptr);
    TEST_ASSERT_EQUAL_UINT32(0u, time_msec);

    /* Test again with chapter index */
    chapter_index = 42;
    ptr = sb_screenplay_get_chapter_ptr_at_time_msec(&screenplay, &time_msec, &chapter_index);
    TEST_ASSERT_NULL(ptr);
    TEST_ASSERT_EQUAL_UINT32(0u, time_msec);
    TEST_ASSERT_EQUAL(-1, chapter_index);

    sb_screenplay_destroy(&screenplay);
}

void test_sb_screenplay_get_chapter_ptr_at_time_msec_infinite_first(void)
{
    sb_screenplay_t screenplay;
    uint32_t time_msec = 5000u;
    sb_screenplay_chapter_t* ptr;
    ssize_t chapter_index;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_init(&screenplay));

    /* Append a single chapter (default is infinite duration) */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_append_new_chapter(&screenplay, NULL));

    /* Call with arbitrary time -> must return first chapter and leave time unchanged */
    ptr = sb_screenplay_get_chapter_ptr_at_time_msec(&screenplay, &time_msec, NULL);
    TEST_ASSERT_NOT_NULL(ptr);
    TEST_ASSERT_EQUAL_PTR(sb_screenplay_get_chapter_ptr(&screenplay, 0), ptr);
    TEST_ASSERT_EQUAL_UINT32(5000u, time_msec);

    /* Test again with chapter index */
    chapter_index = 42;
    ptr = sb_screenplay_get_chapter_ptr_at_time_msec(&screenplay, &time_msec, &chapter_index);
    TEST_ASSERT_NOT_NULL(ptr);
    TEST_ASSERT_EQUAL_PTR(sb_screenplay_get_chapter_ptr(&screenplay, 0), ptr);
    TEST_ASSERT_EQUAL_UINT32(5000u, time_msec);
    TEST_ASSERT_EQUAL(0, chapter_index);

    sb_screenplay_destroy(&screenplay);
}

void test_sb_screenplay_get_chapter_ptr_at_time_msec_finite_offsets_and_overflow(void)
{
    sb_screenplay_t screenplay;
    uint32_t time_msec;
    sb_screenplay_chapter_t* ptr;
    ssize_t chapter_index;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_init(&screenplay));

    /* Append three chapters and set durations: 1000, 2000, 3000 (all finite) */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_append_new_chapter(&screenplay, &ptr));
    sb_screenplay_chapter_set_duration_msec(ptr, 1000u);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_append_new_chapter(&screenplay, &ptr));
    sb_screenplay_chapter_set_duration_msec(ptr, 2000u);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_append_new_chapter(&screenplay, &ptr));
    sb_screenplay_chapter_set_duration_msec(ptr, 3000u);

    /* time within first chapter */
    time_msec = 500u;
    ptr = sb_screenplay_get_chapter_ptr_at_time_msec(&screenplay, &time_msec, &chapter_index);
    TEST_ASSERT_EQUAL_PTR(sb_screenplay_get_chapter_ptr(&screenplay, 0), ptr);
    TEST_ASSERT_EQUAL_UINT32(500u, time_msec);
    TEST_ASSERT_EQUAL(0, chapter_index);

    /* time exactly at end of first chapter and start of second */
    time_msec = 1000u;
    ptr = sb_screenplay_get_chapter_ptr_at_time_msec(&screenplay, &time_msec, &chapter_index);
    TEST_ASSERT_EQUAL_PTR(sb_screenplay_get_chapter_ptr(&screenplay, 1), ptr);
    TEST_ASSERT_EQUAL_UINT32(0u, time_msec);
    TEST_ASSERT_EQUAL(1, chapter_index);

    /* time within second chapter (1500 -> second chapter offset 500) */
    time_msec = 1500u;
    ptr = sb_screenplay_get_chapter_ptr_at_time_msec(&screenplay, &time_msec, &chapter_index);
    TEST_ASSERT_EQUAL_PTR(sb_screenplay_get_chapter_ptr(&screenplay, 1), ptr);
    TEST_ASSERT_EQUAL_UINT32(500u, time_msec);
    TEST_ASSERT_EQUAL(1, chapter_index);

    /* time exactly at end of all chapters -> must return NULL and set time to 0 */
    time_msec = 6000u; /* 1000 + 2000 + 3000 */
    ptr = sb_screenplay_get_chapter_ptr_at_time_msec(&screenplay, &time_msec, &chapter_index);
    TEST_ASSERT_NULL(ptr);
    TEST_ASSERT_EQUAL_UINT32(0u, time_msec);
    TEST_ASSERT_EQUAL(-1, chapter_index);

    /* time beyond total duration -> NULL and time 0 */
    time_msec = 7000u;
    ptr = sb_screenplay_get_chapter_ptr_at_time_msec(&screenplay, &time_msec, &chapter_index);
    TEST_ASSERT_NULL(ptr);
    TEST_ASSERT_EQUAL_UINT32(0u, time_msec);
    TEST_ASSERT_EQUAL(-1, chapter_index);

    sb_screenplay_destroy(&screenplay);
}

void test_sb_screenplay_get_chapter_ptr_at_time_msec_with_infinite_later_chapter(void)
{
    sb_screenplay_t screenplay;
    uint32_t time_msec;
    sb_screenplay_chapter_t* ptr;
    ssize_t chapter_index;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_init(&screenplay));

    /* Append three chapters and set durations: 1000, 2000, infinite */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_append_new_chapter(&screenplay, &ptr));
    sb_screenplay_chapter_set_duration_msec(ptr, 1000u);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_append_new_chapter(&screenplay, &ptr));
    sb_screenplay_chapter_set_duration_msec(ptr, 2000u);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_append_new_chapter(&screenplay, &ptr));
    /* chapter 2 (index 2) remains infinite by default */

    /* time that falls into the infinite third chapter: 1000+2000+500 -> should return third with offset 500 */
    time_msec = 3500u;
    ptr = sb_screenplay_get_chapter_ptr_at_time_msec(&screenplay, &time_msec, &chapter_index);
    TEST_ASSERT_EQUAL_PTR(sb_screenplay_get_chapter_ptr(&screenplay, 2), ptr);
    TEST_ASSERT_EQUAL_UINT32(500u, time_msec);
    TEST_ASSERT_EQUAL(2, chapter_index);

    sb_screenplay_destroy(&screenplay);
}

/* New tests for removing the last chapter, including empty-case behavior */
void test_sb_screenplay_remove_last_chapter(void)
{
    sb_screenplay_t screenplay;
    sb_screenplay_chapter_t* ptr;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_init(&screenplay));

    /* Removing from an empty screenplay should return SB_EEMPTY and leave size 0 */
    TEST_ASSERT_EQUAL(SB_EEMPTY, sb_screenplay_remove_last_chapter(&screenplay));
    TEST_ASSERT_EQUAL(0u, sb_screenplay_size(&screenplay));

    /* Append two chapters */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_append_new_chapter(&screenplay, &ptr));
    TEST_ASSERT_EQUAL(1u, sb_screenplay_size(&screenplay));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_append_new_chapter(&screenplay, &ptr));
    TEST_ASSERT_EQUAL(2u, sb_screenplay_size(&screenplay));

    /* Remove last chapter -> success and size decrements */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_remove_last_chapter(&screenplay));
    TEST_ASSERT_EQUAL(1u, sb_screenplay_size(&screenplay));

    /* Remove last chapter again -> success and size becomes zero */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_remove_last_chapter(&screenplay));
    TEST_ASSERT_EQUAL(0u, sb_screenplay_size(&screenplay));

    /* Removing now should return SB_EEMPTY again */
    TEST_ASSERT_EQUAL(SB_EEMPTY, sb_screenplay_remove_last_chapter(&screenplay));
    TEST_ASSERT_EQUAL(0u, sb_screenplay_size(&screenplay));

    sb_screenplay_destroy(&screenplay);
}

/* Test updating a screenplay from a binary show file that is loaded
 * entirely in memory. The test keeps the buffer alive until the screenplay is
 * destroyed because the chapter (and its trajectory) may reference the buffer.
 */
void test_screenplay_update_from_binary_file_in_memory(void)
{
    sb_screenplay_t screenplay;
    sb_screenplay_chapter_t* chapter;
    FILE* fp;
    size_t num_bytes;
    uint8_t* buf = NULL;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_init(&screenplay));

    /* open fixture and read into memory */
    fp = fopen("fixtures/test.skyb", "rb");
    if (fp == NULL) {
        perror("fixtures/test.skyb");
        abort();
    }

    buf = (uint8_t*)malloc(65536);
    TEST_ASSERT_NOT_NULL(buf);

    num_bytes = fread(buf, sizeof(uint8_t), 65536, fp);
    if (ferror(fp)) {
        perror(NULL);
        fclose(fp);
        free(buf);
        abort();
    }

    fclose(fp);

    /* update screenplay from in-memory binary show */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_update_from_binary_file_in_memory(&screenplay, buf, num_bytes));

    /* check chapter count */
    TEST_ASSERT_EQUAL(1, sb_screenplay_size(&screenplay));

    chapter = sb_screenplay_get_chapter_ptr(&screenplay, 0);
    TEST_ASSERT_NOT_NULL(chapter);

    /* trajectory, light program and yaw control data must be loaded */
    TEST_ASSERT_NOT_NULL(sb_screenplay_chapter_get_trajectory(chapter));
    TEST_ASSERT_NOT_NULL(sb_screenplay_chapter_get_light_program(chapter));
    TEST_ASSERT_NOT_NULL(sb_screenplay_chapter_get_yaw_control(chapter));

    /* no events in file so no event list must be associated to the chapter */
    TEST_ASSERT_NULL(sb_screenplay_chapter_get_events(chapter));

    /* duration must be infinite and time axis must be reset */
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, sb_screenplay_chapter_get_duration_msec(chapter));
    TEST_ASSERT_EQUAL(0, sb_time_axis_num_segments(sb_screenplay_chapter_get_time_axis(chapter)));

    /* update screenplay from null data */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_update_from_binary_file_in_memory(&screenplay, 0, 0));
    TEST_ASSERT_TRUE(sb_screenplay_is_empty(&screenplay));

    /* cleanup: destroy screenplay while buffer is still valid, then free buffer */
    sb_screenplay_destroy(&screenplay);

    free(buf);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_screenplay_init_sets_defaults_and_allocates);
    RUN_TEST(test_sb_screenplay_get_chapter_ptr_at_time_msec_empty);
    RUN_TEST(test_sb_screenplay_get_chapter_ptr_at_time_msec_infinite_first);
    RUN_TEST(test_sb_screenplay_get_chapter_ptr_at_time_msec_finite_offsets_and_overflow);
    RUN_TEST(test_sb_screenplay_get_chapter_ptr_at_time_msec_with_infinite_later_chapter);
    RUN_TEST(test_sb_screenplay_remove_last_chapter);
    RUN_TEST(test_screenplay_update_from_binary_file_in_memory);

    return UNITY_END();
}
