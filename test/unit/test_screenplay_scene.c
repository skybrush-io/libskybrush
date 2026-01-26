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

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <skybrush/screenplay.h>
#include <skybrush/time_axis.h>

#include "unity.h"

void setUp(void)
{
    /* nothing to set up */
}

void tearDown(void)
{
    /* nothing to tear down */
}

void test_screenplay_scene_init_sets_defaults(void)
{
    sb_screenplay_scene_t scene;
    sb_error_t err;

    /* Initialize the scene */
    err = sb_screenplay_scene_init(&scene);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    /* Duration must be infinite by default */
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, sb_screenplay_scene_get_duration_msec(&scene));
    TEST_ASSERT_EQUAL_FLOAT(INFINITY, sb_screenplay_scene_get_duration_sec(&scene));

    /* Optional pointers must be NULL */
    TEST_ASSERT_NULL(sb_screenplay_scene_get_trajectory(&scene));
    TEST_ASSERT_NULL(sb_screenplay_scene_get_light_program(&scene));
    TEST_ASSERT_NULL(sb_screenplay_scene_get_yaw_control(&scene));
    TEST_ASSERT_NULL(sb_screenplay_scene_get_events(&scene));

    /* Time axis must be initialized and empty */
    TEST_ASSERT_EQUAL(0, sb_time_axis_num_segments(sb_screenplay_scene_get_time_axis(&scene)));

    /* Clean up */
    SB_DECREF_STATIC(&scene);
}

void test_screenplay_scene_getters_and_setters(void)
{
    sb_screenplay_scene_t scene;
    sb_trajectory_t traj;
    sb_light_program_t prog;
    sb_yaw_control_t yaw;
    sb_event_list_t events;
    sb_error_t err;

    /* initialize scene */
    err = sb_screenplay_scene_init(&scene);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    /* initialize objects to be attached */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_init(&traj));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_light_program_init(&prog));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_yaw_control_init(&yaw));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_event_list_init(&events, 0));

    /* initial refcounts should be 1 */
    TEST_ASSERT_EQUAL(1, SB_REFCNT(&traj));
    TEST_ASSERT_EQUAL(1, SB_REFCNT(&prog));
    TEST_ASSERT_EQUAL(1, SB_REFCNT(&yaw));
    TEST_ASSERT_EQUAL(1, SB_REFCNT(&events));

    /* set trajectory */
    sb_screenplay_scene_set_trajectory(&scene, &traj);
    TEST_ASSERT_EQUAL_PTR(&traj, sb_screenplay_scene_get_trajectory(&scene));
    TEST_ASSERT_EQUAL(2, SB_REFCNT(&traj));

    /* set light program */
    sb_screenplay_scene_set_light_program(&scene, &prog);
    TEST_ASSERT_EQUAL_PTR(&prog, sb_screenplay_scene_get_light_program(&scene));
    TEST_ASSERT_EQUAL(2, SB_REFCNT(&prog));

    /* set yaw control */
    sb_screenplay_scene_set_yaw_control(&scene, &yaw);
    TEST_ASSERT_EQUAL_PTR(&yaw, sb_screenplay_scene_get_yaw_control(&scene));
    TEST_ASSERT_EQUAL(2, SB_REFCNT(&yaw));

    /* set event list */
    sb_screenplay_scene_set_events(&scene, &events);
    TEST_ASSERT_EQUAL_PTR(&events, sb_screenplay_scene_get_events(&scene));
    TEST_ASSERT_EQUAL(2, SB_REFCNT(&events));

    /* getter for time axis */
    TEST_ASSERT_EQUAL_PTR(&scene.time_axis, sb_screenplay_scene_get_time_axis(&scene));

    /* now unset them (set to NULL) and ensure refcounts decrease */
    sb_screenplay_scene_set_trajectory(&scene, NULL);
    TEST_ASSERT_NULL(sb_screenplay_scene_get_trajectory(&scene));
    TEST_ASSERT_EQUAL(1, SB_REFCNT(&traj));

    sb_screenplay_scene_set_light_program(&scene, NULL);
    TEST_ASSERT_NULL(sb_screenplay_scene_get_light_program(&scene));
    TEST_ASSERT_EQUAL(1, SB_REFCNT(&prog));

    sb_screenplay_scene_set_yaw_control(&scene, NULL);
    TEST_ASSERT_NULL(sb_screenplay_scene_get_yaw_control(&scene));
    TEST_ASSERT_EQUAL(1, SB_REFCNT(&yaw));

    sb_screenplay_scene_set_events(&scene, NULL);
    TEST_ASSERT_NULL(sb_screenplay_scene_get_events(&scene));
    TEST_ASSERT_EQUAL(1, SB_REFCNT(&events));

    /* cleanup: destroy scene and then release static objects */
    SB_DECREF_STATIC(&scene);

    /* objects still have refcount 1 (their own) -> release them */
    SB_DECREF_STATIC(&traj);
    SB_DECREF_STATIC(&prog);
    SB_DECREF_STATIC(&yaw);
    SB_DECREF_STATIC(&events);
}

void test_screenplay_scene_set_duration_sec_finite_rounding(void)
{
    sb_screenplay_scene_t scene;
    sb_error_t err;

    err = sb_screenplay_scene_init(&scene);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    /* set a finite duration that will require rounding:
     * 1.2345 s -> 1234.5 ms -> rounds to 1235 ms */
    err = sb_screenplay_scene_set_duration_sec(&scene, 1.2345f);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    TEST_ASSERT_EQUAL_UINT32(1235u, sb_screenplay_scene_get_duration_msec(&scene));
    TEST_ASSERT_EQUAL_FLOAT(1235.0f / 1000.0f, sb_screenplay_scene_get_duration_sec(&scene));

    SB_DECREF_STATIC(&scene);
}

void test_screenplay_scene_set_duration_sec_infinite(void)
{
    sb_screenplay_scene_t scene;
    sb_error_t err;

    err = sb_screenplay_scene_init(&scene);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    /* set infinite using INFINITY */
    err = sb_screenplay_scene_set_duration_sec(&scene, INFINITY);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, sb_screenplay_scene_get_duration_msec(&scene));
    TEST_ASSERT_EQUAL_FLOAT(INFINITY, sb_screenplay_scene_get_duration_sec(&scene));

    SB_DECREF_STATIC(&scene);
}

void test_screenplay_scene_set_duration_sec_negative_is_invalid_and_preserves_old(void)
{
    sb_screenplay_scene_t scene;
    sb_error_t err;

    err = sb_screenplay_scene_init(&scene);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    /* Set a known finite duration first */
    err = sb_screenplay_scene_set_duration_msec(&scene, 2000u);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);
    TEST_ASSERT_EQUAL_UINT32(2000u, sb_screenplay_scene_get_duration_msec(&scene));

    /* Now try to set a negative duration -> should fail and preserve old */
    err = sb_screenplay_scene_set_duration_sec(&scene, -1.0f);
    TEST_ASSERT_EQUAL(SB_EINVAL, err);
    TEST_ASSERT_EQUAL_UINT32(2000u, sb_screenplay_scene_get_duration_msec(&scene));

    SB_DECREF_STATIC(&scene);
}

void test_screenplay_scene_set_duration_sec_nan_is_invalid_and_preserves_old(void)
{
    sb_screenplay_scene_t scene;
    sb_error_t err;

    err = sb_screenplay_scene_init(&scene);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    /* Set a known finite duration first */
    err = sb_screenplay_scene_set_duration_msec(&scene, 3000u);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);
    TEST_ASSERT_EQUAL_UINT32(3000u, sb_screenplay_scene_get_duration_msec(&scene));

    /* Now try to set NaN -> should fail and preserve old */
    err = sb_screenplay_scene_set_duration_sec(&scene, NAN);
    TEST_ASSERT_EQUAL(SB_EINVAL, err);
    TEST_ASSERT_EQUAL_UINT32(3000u, sb_screenplay_scene_get_duration_msec(&scene));

    SB_DECREF_STATIC(&scene);
}

void test_screenplay_scene_set_duration_sec_too_large_is_invalid_and_preserves_old(void)
{
    sb_screenplay_scene_t scene;
    sb_error_t err;

    err = sb_screenplay_scene_init(&scene);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    /* Set a known finite duration first */
    err = sb_screenplay_scene_set_duration_msec(&scene, 4000u);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);
    TEST_ASSERT_EQUAL_UINT32(4000u, sb_screenplay_scene_get_duration_msec(&scene));

    /* Choose a duration that will result in duration_msec_f > UINT32_MAX */
    float too_large_sec = (UINT32_MAX / 1000.0f) + 1000.0f; /* safely above the threshold */

    err = sb_screenplay_scene_set_duration_sec(&scene, too_large_sec);
    TEST_ASSERT_EQUAL(SB_EINVAL, err);
    TEST_ASSERT_EQUAL_UINT32(4000u, sb_screenplay_scene_get_duration_msec(&scene));

    SB_DECREF_STATIC(&scene);
}

void test_screenplay_scene_set_duration_sec_rounds_to_uint32_max_is_invalid_and_preserves_old(void)
{
    sb_screenplay_scene_t scene;
    sb_error_t err;

    err = sb_screenplay_scene_init(&scene);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    /* Set a known finite duration first */
    err = sb_screenplay_scene_set_duration_msec(&scene, 5000u);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);
    TEST_ASSERT_EQUAL_UINT32(5000u, sb_screenplay_scene_get_duration_msec(&scene));

    /* Create a duration that is <= UINT32_MAX but which will round to UINT32_MAX.
     * We pick a value slightly below UINT32_MAX so that duration_msec_f + 0.5f
     * will push it to UINT32_MAX when truncated to uint32_t.
     *
     * duration_msec_f = UINT32_MAX - 0.25f should do that:
     * duration_msec_f <= UINT32_MAX -> passes first check
     * (uint32_t)(duration_msec_f + 0.5f) == UINT32_MAX -> triggers second invalid branch
     */
    float duration_sec = ((float)UINT32_MAX - 0.25f) / 1000.0f;

    err = sb_screenplay_scene_set_duration_sec(&scene, duration_sec);
    TEST_ASSERT_EQUAL(SB_EINVAL, err);
    TEST_ASSERT_EQUAL_UINT32(5000u, sb_screenplay_scene_get_duration_msec(&scene));

    SB_DECREF_STATIC(&scene);
}

void test_screenplay_scene_reset(void)
{
    sb_screenplay_scene_t scene;
    sb_trajectory_t traj;
    sb_light_program_t prog;
    sb_yaw_control_t yaw;
    sb_event_list_t events;
    sb_error_t err;

    /* initialize scene and objects */
    err = sb_screenplay_scene_init(&scene);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_init(&traj));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_light_program_init(&prog));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_yaw_control_init(&yaw));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_event_list_init(&events, 0));

    /* initial refcounts */
    TEST_ASSERT_EQUAL(1, SB_REFCNT(&traj));
    TEST_ASSERT_EQUAL(1, SB_REFCNT(&prog));
    TEST_ASSERT_EQUAL(1, SB_REFCNT(&yaw));
    TEST_ASSERT_EQUAL(1, SB_REFCNT(&events));

    /* attach everything to scene */
    sb_screenplay_scene_set_trajectory(&scene, &traj);
    sb_screenplay_scene_set_light_program(&scene, &prog);
    sb_screenplay_scene_set_yaw_control(&scene, &yaw);
    sb_screenplay_scene_set_events(&scene, &events);

    /* ensure refcounts increased to 2 */
    TEST_ASSERT_EQUAL(2, SB_REFCNT(&traj));
    TEST_ASSERT_EQUAL(2, SB_REFCNT(&prog));
    TEST_ASSERT_EQUAL(2, SB_REFCNT(&yaw));
    TEST_ASSERT_EQUAL(2, SB_REFCNT(&events));

    /* set finite duration and add a time axis segment so clearing is visible */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_scene_set_duration_msec(&scene, 1234u));
    sb_time_axis_t* axis = sb_screenplay_scene_get_time_axis(&scene);
    TEST_ASSERT_NOT_NULL(axis);
    /* append a constant-rate segment (duration 1000 ms) */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_append_segment(axis, sb_time_segment_make_constant_rate(1000u, 1.0f)));
    TEST_ASSERT_EQUAL(1, sb_time_axis_num_segments(axis));
    TEST_ASSERT_EQUAL_UINT32(1234u, sb_screenplay_scene_get_duration_msec(&scene));

    /* perform reset */
    sb_screenplay_scene_reset(&scene);

    /* after reset, attached pointers must be NULL and refcounts back to 1 */
    TEST_ASSERT_NULL(sb_screenplay_scene_get_trajectory(&scene));
    TEST_ASSERT_NULL(sb_screenplay_scene_get_light_program(&scene));
    TEST_ASSERT_NULL(sb_screenplay_scene_get_yaw_control(&scene));
    TEST_ASSERT_NULL(sb_screenplay_scene_get_events(&scene));

    TEST_ASSERT_EQUAL(1, SB_REFCNT(&traj));
    TEST_ASSERT_EQUAL(1, SB_REFCNT(&prog));
    TEST_ASSERT_EQUAL(1, SB_REFCNT(&yaw));
    TEST_ASSERT_EQUAL(1, SB_REFCNT(&events));

    /* duration must be reset to infinite */
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, sb_screenplay_scene_get_duration_msec(&scene));
    TEST_ASSERT_EQUAL_FLOAT(INFINITY, sb_screenplay_scene_get_duration_sec(&scene));

    /* time axis must be cleared */
    TEST_ASSERT_EQUAL(0, sb_time_axis_num_segments(axis));

    /* cleanup */
    SB_DECREF_STATIC(&scene);

    SB_DECREF_STATIC(&traj);
    SB_DECREF_STATIC(&prog);
    SB_DECREF_STATIC(&yaw);
    SB_DECREF_STATIC(&events);
}

/* Test updating a screenplay scene from a binary show file that is loaded
 * entirely in memory. The test keeps the buffer alive until the scene is
 * destroyed because the scene (and its trajectory) may reference the buffer.
 */
void test_screenplay_scene_update_from_binary_file_in_memory(void)
{
    sb_screenplay_scene_t scene;
    FILE* fp;
    size_t num_bytes;
    uint8_t* buf = NULL;

    /* initialize scene */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_scene_init(&scene));

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

    /* update scene from in-memory binary show */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_scene_update_from_binary_file_in_memory(&scene, buf, num_bytes));

    /* trajectory, light program and yaw control data must be loaded */
    TEST_ASSERT_NOT_NULL(sb_screenplay_scene_get_trajectory(&scene));
    TEST_ASSERT_NOT_NULL(sb_screenplay_scene_get_light_program(&scene));
    TEST_ASSERT_NOT_NULL(sb_screenplay_scene_get_yaw_control(&scene));

    /* no events in file so no event list must be associated to the scene */
    TEST_ASSERT_NULL(sb_screenplay_scene_get_events(&scene));

    /* duration must be infinite and time axis must be reset */
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, sb_screenplay_scene_get_duration_msec(&scene));
    TEST_ASSERT_EQUAL(0, sb_time_axis_num_segments(sb_screenplay_scene_get_time_axis(&scene)));

    /* now update from empty data */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_scene_update_from_binary_file_in_memory(&scene, 0, 0));

    /* trajectory, light program and yaw control data must not be loaded */
    TEST_ASSERT_NULL(sb_screenplay_scene_get_trajectory(&scene));
    TEST_ASSERT_NULL(sb_screenplay_scene_get_light_program(&scene));
    TEST_ASSERT_NULL(sb_screenplay_scene_get_yaw_control(&scene));
    TEST_ASSERT_NULL(sb_screenplay_scene_get_events(&scene));

    /* duration must be infinite and time axis must be reset */
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, sb_screenplay_scene_get_duration_msec(&scene));
    TEST_ASSERT_EQUAL(0, sb_time_axis_num_segments(sb_screenplay_scene_get_time_axis(&scene)));

    /* cleanup: destroy scene while buffer is still valid, then free buffer */
    SB_DECREF_STATIC(&scene);

    free(buf);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_screenplay_scene_init_sets_defaults);
    RUN_TEST(test_screenplay_scene_getters_and_setters);
    RUN_TEST(test_screenplay_scene_set_duration_sec_finite_rounding);
    RUN_TEST(test_screenplay_scene_set_duration_sec_infinite);
    RUN_TEST(test_screenplay_scene_set_duration_sec_negative_is_invalid_and_preserves_old);
    RUN_TEST(test_screenplay_scene_set_duration_sec_nan_is_invalid_and_preserves_old);
    RUN_TEST(test_screenplay_scene_set_duration_sec_too_large_is_invalid_and_preserves_old);
    RUN_TEST(test_screenplay_scene_set_duration_sec_rounds_to_uint32_max_is_invalid_and_preserves_old);
    RUN_TEST(test_screenplay_scene_reset);
    RUN_TEST(test_screenplay_scene_update_from_binary_file_in_memory);

    return UNITY_END();
}
