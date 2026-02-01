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

static uint8_t* load_fixture_to_buffer(const char* fname, size_t* num_bytes)
{
    FILE* fp;
    uint8_t* buf = NULL;

    fp = fopen(fname, "rb");
    if (fp == NULL) {
        perror(fname);
        abort();
    }

    buf = (uint8_t*)malloc(65536);
    TEST_ASSERT_NOT_NULL(buf);

    *num_bytes = fread(buf, sizeof(uint8_t), 65536, fp);
    if (ferror(fp)) {
        perror(NULL);
        fclose(fp);
        free(buf);
        abort();
    }

    fclose(fp);

    return buf;
}

void test_screenplay_init_sets_defaults_and_allocates(void)
{
    sb_screenplay_t screenplay;

    /* Initialize the screenplay */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_init(&screenplay));

    /* scenes storage must be allocated */
    TEST_ASSERT_NOT_NULL(screenplay.scenes);

    /* size must be zero, capacity at least 1 */
    TEST_ASSERT_EQUAL(0u, sb_screenplay_size(&screenplay));
    TEST_ASSERT_TRUE(sb_screenplay_capacity(&screenplay) >= 1u);

    /* getting scene pointer must return NULL for empty screenplay */
    TEST_ASSERT_NULL(sb_screenplay_get_scene_ptr(&screenplay, 0));

    /* Clean up */
    sb_screenplay_destroy(&screenplay);

    /* After destroy, size and capacity must be zero */
    TEST_ASSERT_EQUAL(0u, sb_screenplay_size(&screenplay));
    TEST_ASSERT_EQUAL(0u, sb_screenplay_capacity(&screenplay));

    /* get scene should still return NULL (num_scenes is zero) */
    TEST_ASSERT_NULL(sb_screenplay_get_scene_ptr(&screenplay, 0));
}

void test_sb_screenplay_get_scene_ptr_at_time_msec_empty(void)
{
    sb_screenplay_t screenplay;
    sb_error_t err;
    uint32_t time_msec = 12345u;
    sb_screenplay_scene_t* ptr;
    ssize_t scene_index;

    err = sb_screenplay_init(&screenplay);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    /* Empty screenplay -> must return NULL and set time_msec to 0 */
    ptr = sb_screenplay_get_scene_ptr_at_time_msec(&screenplay, &time_msec, NULL);
    TEST_ASSERT_NULL(ptr);
    TEST_ASSERT_EQUAL_UINT32(0u, time_msec);

    /* Test again with scene index */
    scene_index = 42;
    ptr = sb_screenplay_get_scene_ptr_at_time_msec(&screenplay, &time_msec, &scene_index);
    TEST_ASSERT_NULL(ptr);
    TEST_ASSERT_EQUAL_UINT32(0u, time_msec);
    TEST_ASSERT_EQUAL(-1, scene_index);

    sb_screenplay_destroy(&screenplay);
}

void test_sb_screenplay_get_scene_ptr_at_time_msec_infinite_first(void)
{
    sb_screenplay_t screenplay;
    uint32_t time_msec = 5000u;
    sb_screenplay_scene_t* ptr;
    ssize_t scene_index;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_init(&screenplay));

    /* Append a single scene (default is infinite duration) */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_append_new_scene(&screenplay, NULL));

    /* Call with arbitrary time -> must return first scene and leave time unchanged */
    ptr = sb_screenplay_get_scene_ptr_at_time_msec(&screenplay, &time_msec, NULL);
    TEST_ASSERT_NOT_NULL(ptr);
    TEST_ASSERT_EQUAL_PTR(sb_screenplay_get_scene_ptr(&screenplay, 0), ptr);
    TEST_ASSERT_EQUAL_UINT32(5000u, time_msec);

    /* Test again with scene index */
    scene_index = 42;
    ptr = sb_screenplay_get_scene_ptr_at_time_msec(&screenplay, &time_msec, &scene_index);
    TEST_ASSERT_NOT_NULL(ptr);
    TEST_ASSERT_EQUAL_PTR(sb_screenplay_get_scene_ptr(&screenplay, 0), ptr);
    TEST_ASSERT_EQUAL_UINT32(5000u, time_msec);
    TEST_ASSERT_EQUAL(0, scene_index);

    sb_screenplay_destroy(&screenplay);
}

void test_sb_screenplay_get_scene_ptr_at_time_msec_finite_offsets_and_overflow(void)
{
    sb_screenplay_t screenplay;
    uint32_t time_msec;
    sb_screenplay_scene_t* ptr;
    ssize_t scene_index;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_init(&screenplay));

    /* Append three scenes and set durations: 1000, 2000, 3000 (all finite) */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_append_new_scene(&screenplay, &ptr));
    sb_screenplay_scene_set_duration_msec(ptr, 1000u);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_append_new_scene(&screenplay, &ptr));
    sb_screenplay_scene_set_duration_msec(ptr, 2000u);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_append_new_scene(&screenplay, &ptr));
    sb_screenplay_scene_set_duration_msec(ptr, 3000u);

    /* time within first scene */
    time_msec = 500u;
    ptr = sb_screenplay_get_scene_ptr_at_time_msec(&screenplay, &time_msec, &scene_index);
    TEST_ASSERT_EQUAL_PTR(sb_screenplay_get_scene_ptr(&screenplay, 0), ptr);
    TEST_ASSERT_EQUAL_UINT32(500u, time_msec);
    TEST_ASSERT_EQUAL(0, scene_index);

    /* time exactly at end of first scene and start of second */
    time_msec = 1000u;
    ptr = sb_screenplay_get_scene_ptr_at_time_msec(&screenplay, &time_msec, &scene_index);
    TEST_ASSERT_EQUAL_PTR(sb_screenplay_get_scene_ptr(&screenplay, 1), ptr);
    TEST_ASSERT_EQUAL_UINT32(0u, time_msec);
    TEST_ASSERT_EQUAL(1, scene_index);

    /* time within second scene (1500 -> second scene offset 500) */
    time_msec = 1500u;
    ptr = sb_screenplay_get_scene_ptr_at_time_msec(&screenplay, &time_msec, &scene_index);
    TEST_ASSERT_EQUAL_PTR(sb_screenplay_get_scene_ptr(&screenplay, 1), ptr);
    TEST_ASSERT_EQUAL_UINT32(500u, time_msec);
    TEST_ASSERT_EQUAL(1, scene_index);

    /* time exactly at end of all scenes -> must return NULL and set time to 0 */
    time_msec = 6000u; /* 1000 + 2000 + 3000 */
    ptr = sb_screenplay_get_scene_ptr_at_time_msec(&screenplay, &time_msec, &scene_index);
    TEST_ASSERT_NULL(ptr);
    TEST_ASSERT_EQUAL_UINT32(0u, time_msec);
    TEST_ASSERT_EQUAL(-1, scene_index);

    /* time beyond total duration -> NULL and time 0 */
    time_msec = 7000u;
    ptr = sb_screenplay_get_scene_ptr_at_time_msec(&screenplay, &time_msec, &scene_index);
    TEST_ASSERT_NULL(ptr);
    TEST_ASSERT_EQUAL_UINT32(0u, time_msec);
    TEST_ASSERT_EQUAL(-1, scene_index);

    sb_screenplay_destroy(&screenplay);
}

void test_sb_screenplay_get_scene_ptr_at_time_msec_with_infinite_later_scene(void)
{
    sb_screenplay_t screenplay;
    uint32_t time_msec;
    sb_screenplay_scene_t* ptr;
    ssize_t scene_index;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_init(&screenplay));

    /* Append three scenes and set durations: 1000, 2000, infinite */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_append_new_scene(&screenplay, &ptr));
    sb_screenplay_scene_set_duration_msec(ptr, 1000u);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_append_new_scene(&screenplay, &ptr));
    sb_screenplay_scene_set_duration_msec(ptr, 2000u);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_append_new_scene(&screenplay, &ptr));
    /* scene 2 (index 2) remains infinite by default */

    /* time that falls into the infinite third scene: 1000+2000+500 -> should return third with offset 500 */
    time_msec = 3500u;
    ptr = sb_screenplay_get_scene_ptr_at_time_msec(&screenplay, &time_msec, &scene_index);
    TEST_ASSERT_EQUAL_PTR(sb_screenplay_get_scene_ptr(&screenplay, 2), ptr);
    TEST_ASSERT_EQUAL_UINT32(500u, time_msec);
    TEST_ASSERT_EQUAL(2, scene_index);

    sb_screenplay_destroy(&screenplay);
}

/* New tests for removing the last scene, including empty-case behavior */
void test_sb_screenplay_remove_last_scene(void)
{
    sb_screenplay_t screenplay;
    sb_screenplay_scene_t* ptr;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_init(&screenplay));

    /* Removing from an empty screenplay should return SB_EEMPTY and leave size 0 */
    TEST_ASSERT_EQUAL(SB_EEMPTY, sb_screenplay_remove_last_scene(&screenplay));
    TEST_ASSERT_EQUAL(0u, sb_screenplay_size(&screenplay));

    /* Append two scenes */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_append_new_scene(&screenplay, &ptr));
    TEST_ASSERT_EQUAL(1u, sb_screenplay_size(&screenplay));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_append_new_scene(&screenplay, &ptr));
    TEST_ASSERT_EQUAL(2u, sb_screenplay_size(&screenplay));

    /* Remove last scene -> success and size decrements */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_remove_last_scene(&screenplay));
    TEST_ASSERT_EQUAL(1u, sb_screenplay_size(&screenplay));

    /* Remove last scene again -> success and size becomes zero */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_remove_last_scene(&screenplay));
    TEST_ASSERT_EQUAL(0u, sb_screenplay_size(&screenplay));

    /* Removing now should return SB_EEMPTY again */
    TEST_ASSERT_EQUAL(SB_EEMPTY, sb_screenplay_remove_last_scene(&screenplay));
    TEST_ASSERT_EQUAL(0u, sb_screenplay_size(&screenplay));

    sb_screenplay_destroy(&screenplay);
}

/* Test updating a screenplay from a binary show file that is loaded
 * entirely in memory. The test keeps the buffer alive until the screenplay is
 * destroyed because the scene (and its trajectory) may reference the buffer.
 */
void test_screenplay_update_from_binary_file_in_memory(void)
{
    sb_screenplay_t screenplay;
    sb_screenplay_scene_t* scene;
    size_t num_bytes;
    uint8_t* buf = NULL;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_init(&screenplay));

    /* open fixture and read into memory */
    buf = load_fixture_to_buffer("fixtures/test.skyb", &num_bytes);
    TEST_ASSERT_NOT_NULL(buf);

    /* update screenplay from in-memory binary show */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_update_from_binary_file_in_memory(&screenplay, buf, num_bytes));

    /* no RTH plan in file so no RTH plan must be associated to the screenplay */
    TEST_ASSERT_NULL(sb_screenplay_get_rth_plan(&screenplay));

    /* check scene count */
    TEST_ASSERT_EQUAL(1, sb_screenplay_size(&screenplay));

    scene = sb_screenplay_get_scene_ptr(&screenplay, 0);
    TEST_ASSERT_NOT_NULL(scene);

    /* trajectory, light program and yaw control data must be loaded */
    TEST_ASSERT_NOT_NULL(sb_screenplay_scene_get_trajectory(scene));
    TEST_ASSERT_NOT_NULL(sb_screenplay_scene_get_light_program(scene));
    TEST_ASSERT_NOT_NULL(sb_screenplay_scene_get_yaw_control(scene));

    /* no events in file so no event list must be associated to the scene */
    TEST_ASSERT_NULL(sb_screenplay_scene_get_events(scene));

    /* duration must be infinite and time axis must be reset */
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, sb_screenplay_scene_get_duration_msec(scene));
    TEST_ASSERT_EQUAL(0, sb_time_axis_num_segments(sb_screenplay_scene_get_time_axis(scene)));

    /* update screenplay from null data */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_update_from_binary_file_in_memory(&screenplay, 0, 0));
    TEST_ASSERT_TRUE(sb_screenplay_is_empty(&screenplay));
    TEST_ASSERT_NULL(sb_screenplay_get_rth_plan(&screenplay));

    /* cleanup: destroy screenplay while buffer is still valid, then free buffer */
    sb_screenplay_destroy(&screenplay);

    free(buf);
}

void test_screenplay_update_from_binary_file_in_memory_loads_rth_plan(void)
{
    sb_screenplay_t screenplay;
    sb_rth_plan_t* rth_plan;
    size_t num_bytes;
    uint8_t* buf = NULL;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_init(&screenplay));

    /* open fixture and read into memory */
    buf = load_fixture_to_buffer("fixtures/hover_3m_with_rth_plan.skyb", &num_bytes);
    TEST_ASSERT_NOT_NULL(buf);

    /* update screenplay from in-memory binary show */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_update_from_binary_file_in_memory(&screenplay, buf, num_bytes));

    /* RTH plan must be loaded */
    rth_plan = sb_screenplay_get_rth_plan(&screenplay);
    TEST_ASSERT_NOT_NULL(rth_plan);
    TEST_ASSERT_FALSE(sb_rth_plan_is_empty(rth_plan));
    TEST_ASSERT_EQUAL(2, sb_rth_plan_get_num_points(rth_plan));
    TEST_ASSERT_EQUAL(7, sb_rth_plan_get_num_entries(rth_plan));

    /* cleanup: destroy screenplay while buffer is still valid, then free buffer */
    sb_screenplay_destroy(&screenplay);

    free(buf);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_screenplay_init_sets_defaults_and_allocates);
    RUN_TEST(test_sb_screenplay_get_scene_ptr_at_time_msec_empty);
    RUN_TEST(test_sb_screenplay_get_scene_ptr_at_time_msec_infinite_first);
    RUN_TEST(test_sb_screenplay_get_scene_ptr_at_time_msec_finite_offsets_and_overflow);
    RUN_TEST(test_sb_screenplay_get_scene_ptr_at_time_msec_with_infinite_later_scene);
    RUN_TEST(test_sb_screenplay_remove_last_scene);
    RUN_TEST(test_screenplay_update_from_binary_file_in_memory);
    RUN_TEST(test_screenplay_update_from_binary_file_in_memory_loads_rth_plan);

    return UNITY_END();
}
