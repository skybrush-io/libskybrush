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

#include "skybrush/refcount.h"
#include "unity.h"
#include <math.h>
#include <stdint.h>

#include <skybrush/screenplay.h>
#include <skybrush/time_axis.h>

void setUp(void)
{
    /* nothing to set up */
}

void tearDown(void)
{
    /* nothing to tear down */
}

void test_screenplay_chapter_init_sets_defaults(void)
{
    sb_screenplay_chapter_t chapter;
    sb_error_t err;

    /* Initialize the chapter */
    err = sb_screenplay_chapter_init(&chapter);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    /* Duration must be infinite by default */
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, sb_screenplay_chapter_get_duration_msec(&chapter));
    TEST_ASSERT_EQUAL_FLOAT(INFINITY, sb_screenplay_chapter_get_duration_sec(&chapter));

    /* Optional pointers must be NULL */
    TEST_ASSERT_NULL(sb_screenplay_chapter_get_trajectory(&chapter));
    TEST_ASSERT_NULL(sb_screenplay_chapter_get_light_program(&chapter));
    TEST_ASSERT_NULL(sb_screenplay_chapter_get_yaw_control(&chapter));
    TEST_ASSERT_NULL(sb_screenplay_chapter_get_events(&chapter));

    /* Time axis must be initialized and empty */
    TEST_ASSERT_EQUAL(0, sb_time_axis_num_segments(sb_screenplay_chapter_get_time_axis(&chapter)));

    /* Clean up */
    SB_DECREF_STATIC(&chapter);
}

void test_screenplay_chapter_getters_and_setters(void)
{
    sb_screenplay_chapter_t chapter;
    sb_trajectory_t traj;
    sb_light_program_t prog;
    sb_yaw_control_t yaw;
    sb_event_list_t events;
    sb_error_t err;

    /* initialize chapter */
    err = sb_screenplay_chapter_init(&chapter);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    /* initialize objects to be attached */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_init_empty(&traj));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_light_program_init_empty(&prog));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_yaw_control_init_empty(&yaw));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_event_list_init(&events, 0));

    /* initial refcounts should be 1 */
    TEST_ASSERT_EQUAL(1, SB_REFCNT(&traj));
    TEST_ASSERT_EQUAL(1, SB_REFCNT(&prog));
    TEST_ASSERT_EQUAL(1, SB_REFCNT(&yaw));
    TEST_ASSERT_EQUAL(1, SB_REFCNT(&events));

    /* set trajectory */
    sb_screenplay_chapter_set_trajectory(&chapter, &traj);
    TEST_ASSERT_EQUAL_PTR(&traj, sb_screenplay_chapter_get_trajectory(&chapter));
    TEST_ASSERT_EQUAL(2, SB_REFCNT(&traj));

    /* set light program */
    sb_screenplay_chapter_set_light_program(&chapter, &prog);
    TEST_ASSERT_EQUAL_PTR(&prog, sb_screenplay_chapter_get_light_program(&chapter));
    TEST_ASSERT_EQUAL(2, SB_REFCNT(&prog));

    /* set yaw control */
    sb_screenplay_chapter_set_yaw_control(&chapter, &yaw);
    TEST_ASSERT_EQUAL_PTR(&yaw, sb_screenplay_chapter_get_yaw_control(&chapter));
    TEST_ASSERT_EQUAL(2, SB_REFCNT(&yaw));

    /* set event list */
    sb_screenplay_chapter_set_event_list(&chapter, &events);
    TEST_ASSERT_EQUAL_PTR(&events, sb_screenplay_chapter_get_events(&chapter));
    TEST_ASSERT_EQUAL(2, SB_REFCNT(&events));

    /* getter for time axis */
    TEST_ASSERT_EQUAL_PTR(&chapter.time_axis, sb_screenplay_chapter_get_time_axis(&chapter));

    /* now unset them (set to NULL) and ensure refcounts decrease */
    sb_screenplay_chapter_set_trajectory(&chapter, NULL);
    TEST_ASSERT_NULL(sb_screenplay_chapter_get_trajectory(&chapter));
    TEST_ASSERT_EQUAL(1, SB_REFCNT(&traj));

    sb_screenplay_chapter_set_light_program(&chapter, NULL);
    TEST_ASSERT_NULL(sb_screenplay_chapter_get_light_program(&chapter));
    TEST_ASSERT_EQUAL(1, SB_REFCNT(&prog));

    sb_screenplay_chapter_set_yaw_control(&chapter, NULL);
    TEST_ASSERT_NULL(sb_screenplay_chapter_get_yaw_control(&chapter));
    TEST_ASSERT_EQUAL(1, SB_REFCNT(&yaw));

    sb_screenplay_chapter_set_event_list(&chapter, NULL);
    TEST_ASSERT_NULL(sb_screenplay_chapter_get_events(&chapter));
    TEST_ASSERT_EQUAL(1, SB_REFCNT(&events));

    /* cleanup: destroy chapter and then release static objects */
    SB_DECREF_STATIC(&chapter);

    /* objects still have refcount 1 (their own) -> release them */
    SB_DECREF_STATIC(&traj);
    SB_DECREF_STATIC(&prog);
    SB_DECREF_STATIC(&yaw);
    SB_DECREF_STATIC(&events);
}

void test_screenplay_chapter_set_duration_sec_finite_rounding(void)
{
    sb_screenplay_chapter_t chapter;
    sb_error_t err;

    err = sb_screenplay_chapter_init(&chapter);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    /* set a finite duration that will require rounding:
     * 1.2345 s -> 1234.5 ms -> rounds to 1235 ms */
    err = sb_screenplay_chapter_set_duration_sec(&chapter, 1.2345f);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    TEST_ASSERT_EQUAL_UINT32(1235u, sb_screenplay_chapter_get_duration_msec(&chapter));
    TEST_ASSERT_EQUAL_FLOAT(1235.0f / 1000.0f, sb_screenplay_chapter_get_duration_sec(&chapter));

    SB_DECREF_STATIC(&chapter);
}

void test_screenplay_chapter_set_duration_sec_infinite(void)
{
    sb_screenplay_chapter_t chapter;
    sb_error_t err;

    err = sb_screenplay_chapter_init(&chapter);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    /* set infinite using INFINITY */
    err = sb_screenplay_chapter_set_duration_sec(&chapter, INFINITY);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, sb_screenplay_chapter_get_duration_msec(&chapter));
    TEST_ASSERT_EQUAL_FLOAT(INFINITY, sb_screenplay_chapter_get_duration_sec(&chapter));

    SB_DECREF_STATIC(&chapter);
}

void test_screenplay_chapter_set_duration_sec_negative_is_invalid_and_preserves_old(void)
{
    sb_screenplay_chapter_t chapter;
    sb_error_t err;

    err = sb_screenplay_chapter_init(&chapter);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    /* Set a known finite duration first */
    err = sb_screenplay_chapter_set_duration_msec(&chapter, 2000u);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);
    TEST_ASSERT_EQUAL_UINT32(2000u, sb_screenplay_chapter_get_duration_msec(&chapter));

    /* Now try to set a negative duration -> should fail and preserve old */
    err = sb_screenplay_chapter_set_duration_sec(&chapter, -1.0f);
    TEST_ASSERT_EQUAL(SB_EINVAL, err);
    TEST_ASSERT_EQUAL_UINT32(2000u, sb_screenplay_chapter_get_duration_msec(&chapter));

    SB_DECREF_STATIC(&chapter);
}

void test_screenplay_chapter_set_duration_sec_nan_is_invalid_and_preserves_old(void)
{
    sb_screenplay_chapter_t chapter;
    sb_error_t err;

    err = sb_screenplay_chapter_init(&chapter);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    /* Set a known finite duration first */
    err = sb_screenplay_chapter_set_duration_msec(&chapter, 3000u);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);
    TEST_ASSERT_EQUAL_UINT32(3000u, sb_screenplay_chapter_get_duration_msec(&chapter));

    /* Now try to set NaN -> should fail and preserve old */
    err = sb_screenplay_chapter_set_duration_sec(&chapter, NAN);
    TEST_ASSERT_EQUAL(SB_EINVAL, err);
    TEST_ASSERT_EQUAL_UINT32(3000u, sb_screenplay_chapter_get_duration_msec(&chapter));

    SB_DECREF_STATIC(&chapter);
}

void test_screenplay_chapter_set_duration_sec_too_large_is_invalid_and_preserves_old(void)
{
    sb_screenplay_chapter_t chapter;
    sb_error_t err;

    err = sb_screenplay_chapter_init(&chapter);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    /* Set a known finite duration first */
    err = sb_screenplay_chapter_set_duration_msec(&chapter, 4000u);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);
    TEST_ASSERT_EQUAL_UINT32(4000u, sb_screenplay_chapter_get_duration_msec(&chapter));

    /* Choose a duration that will result in duration_msec_f > UINT32_MAX */
    float too_large_sec = (UINT32_MAX / 1000.0f) + 1000.0f; /* safely above the threshold */

    err = sb_screenplay_chapter_set_duration_sec(&chapter, too_large_sec);
    TEST_ASSERT_EQUAL(SB_EINVAL, err);
    TEST_ASSERT_EQUAL_UINT32(4000u, sb_screenplay_chapter_get_duration_msec(&chapter));

    SB_DECREF_STATIC(&chapter);
}

void test_screenplay_chapter_set_duration_sec_rounds_to_uint32_max_is_invalid_and_preserves_old(void)
{
    sb_screenplay_chapter_t chapter;
    sb_error_t err;

    err = sb_screenplay_chapter_init(&chapter);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    /* Set a known finite duration first */
    err = sb_screenplay_chapter_set_duration_msec(&chapter, 5000u);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);
    TEST_ASSERT_EQUAL_UINT32(5000u, sb_screenplay_chapter_get_duration_msec(&chapter));

    /* Create a duration that is <= UINT32_MAX but which will round to UINT32_MAX.
     * We pick a value slightly below UINT32_MAX so that duration_msec_f + 0.5f
     * will push it to UINT32_MAX when truncated to uint32_t.
     *
     * duration_msec_f = UINT32_MAX - 0.25f should do that:
     * duration_msec_f <= UINT32_MAX -> passes first check
     * (uint32_t)(duration_msec_f + 0.5f) == UINT32_MAX -> triggers second invalid branch
     */
    float duration_sec = ((float)UINT32_MAX - 0.25f) / 1000.0f;

    err = sb_screenplay_chapter_set_duration_sec(&chapter, duration_sec);
    TEST_ASSERT_EQUAL(SB_EINVAL, err);
    TEST_ASSERT_EQUAL_UINT32(5000u, sb_screenplay_chapter_get_duration_msec(&chapter));

    SB_DECREF_STATIC(&chapter);
}

void test_screenplay_chapter_reset(void)
{
    sb_screenplay_chapter_t chapter;
    sb_trajectory_t traj;
    sb_light_program_t prog;
    sb_yaw_control_t yaw;
    sb_event_list_t events;
    sb_error_t err;

    /* initialize chapter and objects */
    err = sb_screenplay_chapter_init(&chapter);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_init_empty(&traj));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_light_program_init_empty(&prog));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_yaw_control_init_empty(&yaw));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_event_list_init(&events, 0));

    /* initial refcounts */
    TEST_ASSERT_EQUAL(1, SB_REFCNT(&traj));
    TEST_ASSERT_EQUAL(1, SB_REFCNT(&prog));
    TEST_ASSERT_EQUAL(1, SB_REFCNT(&yaw));
    TEST_ASSERT_EQUAL(1, SB_REFCNT(&events));

    /* attach everything to chapter */
    sb_screenplay_chapter_set_trajectory(&chapter, &traj);
    sb_screenplay_chapter_set_light_program(&chapter, &prog);
    sb_screenplay_chapter_set_yaw_control(&chapter, &yaw);
    sb_screenplay_chapter_set_event_list(&chapter, &events);

    /* ensure refcounts increased to 2 */
    TEST_ASSERT_EQUAL(2, SB_REFCNT(&traj));
    TEST_ASSERT_EQUAL(2, SB_REFCNT(&prog));
    TEST_ASSERT_EQUAL(2, SB_REFCNT(&yaw));
    TEST_ASSERT_EQUAL(2, SB_REFCNT(&events));

    /* set finite duration and add a time axis segment so clearing is visible */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_chapter_set_duration_msec(&chapter, 1234u));
    sb_time_axis_t* axis = sb_screenplay_chapter_get_time_axis(&chapter);
    TEST_ASSERT_NOT_NULL(axis);
    /* append a constant-rate segment (duration 1000 ms) */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_append_segment(axis, sb_time_segment_make_constant_rate(1000u, 1.0f)));
    TEST_ASSERT_EQUAL(1, sb_time_axis_num_segments(axis));
    TEST_ASSERT_EQUAL_UINT32(1234u, sb_screenplay_chapter_get_duration_msec(&chapter));

    /* perform reset */
    sb_screenplay_chapter_reset(&chapter);

    /* after reset, attached pointers must be NULL and refcounts back to 1 */
    TEST_ASSERT_NULL(sb_screenplay_chapter_get_trajectory(&chapter));
    TEST_ASSERT_NULL(sb_screenplay_chapter_get_light_program(&chapter));
    TEST_ASSERT_NULL(sb_screenplay_chapter_get_yaw_control(&chapter));
    TEST_ASSERT_NULL(sb_screenplay_chapter_get_events(&chapter));

    TEST_ASSERT_EQUAL(1, SB_REFCNT(&traj));
    TEST_ASSERT_EQUAL(1, SB_REFCNT(&prog));
    TEST_ASSERT_EQUAL(1, SB_REFCNT(&yaw));
    TEST_ASSERT_EQUAL(1, SB_REFCNT(&events));

    /* duration must be reset to infinite */
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, sb_screenplay_chapter_get_duration_msec(&chapter));
    TEST_ASSERT_EQUAL_FLOAT(INFINITY, sb_screenplay_chapter_get_duration_sec(&chapter));

    /* time axis must be cleared */
    TEST_ASSERT_EQUAL(0, sb_time_axis_num_segments(axis));

    /* cleanup */
    SB_DECREF_STATIC(&chapter);

    SB_DECREF_STATIC(&traj);
    SB_DECREF_STATIC(&prog);
    SB_DECREF_STATIC(&yaw);
    SB_DECREF_STATIC(&events);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_screenplay_chapter_init_sets_defaults);
    RUN_TEST(test_screenplay_chapter_getters_and_setters);
    RUN_TEST(test_screenplay_chapter_set_duration_sec_finite_rounding);
    RUN_TEST(test_screenplay_chapter_set_duration_sec_infinite);
    RUN_TEST(test_screenplay_chapter_set_duration_sec_negative_is_invalid_and_preserves_old);
    RUN_TEST(test_screenplay_chapter_set_duration_sec_nan_is_invalid_and_preserves_old);
    RUN_TEST(test_screenplay_chapter_set_duration_sec_too_large_is_invalid_and_preserves_old);
    RUN_TEST(test_screenplay_chapter_set_duration_sec_rounds_to_uint32_max_is_invalid_and_preserves_old);
    RUN_TEST(test_screenplay_chapter_reset);

    return UNITY_END();
}
