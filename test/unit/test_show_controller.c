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
#include <stdint.h>
#include <stdio.h>

#include "utils.h"
#include <skybrush/control.h>
#include <skybrush/lights.h>
#include <skybrush/screenplay.h>
#include <skybrush/trajectory.h>

void setUp(void)
{
    /* nothing to set up */
}

void tearDown(void)
{
    /* nothing to tear down */
}

void test_show_controller_init_sets_defaults_and_get_current_output(void)
{
    sb_show_controller_t ctrl;
    sb_error_t err;
    sb_control_output_time_t output_time;

    /* Initialize controller with no screenplay */
    err = sb_show_controller_init(&ctrl, NULL);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    /* Current output should point to ctrl.output and reflect defaults */
    const sb_control_output_t* out = sb_show_controller_get_current_output(&ctrl);
    TEST_ASSERT_NOT_NULL(out);

    /* Default output should include velocity and yaw rate only */
    TEST_ASSERT_TRUE(sb_control_output_has_any_component_in(out, SB_CONTROL_OUTPUT_VELOCITY));
    TEST_ASSERT_TRUE(sb_control_output_has_any_component_in(out, SB_CONTROL_OUTPUT_YAW_RATE));

    TEST_ASSERT_FALSE(sb_control_output_has_any_component_in(out, SB_CONTROL_OUTPUT_POSITION));
    TEST_ASSERT_FALSE(sb_control_output_has_any_component_in(out, SB_CONTROL_OUTPUT_LIGHTS));
    TEST_ASSERT_FALSE(sb_control_output_has_any_component_in(out, SB_CONTROL_OUTPUT_YAW));

    /* Velocity values must be zero (default) and yaw rate zero */
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out->velocity.x);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out->velocity.y);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out->velocity.z);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out->velocity.yaw);

    /* Timestamps should be reported as not valid */
    TEST_ASSERT_FALSE(sb_show_controller_is_output_valid(&ctrl));
    output_time = sb_show_controller_get_current_output_time(&ctrl);
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, output_time.time_msec);
    TEST_ASSERT_EQUAL_UINT32(0, output_time.chapter);
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, output_time.time_in_chapter_msec);
    TEST_ASSERT_EQUAL_FLOAT(0, output_time.warped_time_in_chapter_sec);

    /* Destroy should clear and zero the controller */
    sb_show_controller_destroy(&ctrl);
    TEST_ASSERT_NULL(ctrl.screenplay);
    const sb_control_output_t* out_after_destroy = sb_show_controller_get_current_output(&ctrl);
    TEST_ASSERT_EQUAL_UINT8(0u, out_after_destroy->mask);
}

void test_show_controller_update_time_without_screenplay_returns_default(void)
{
    sb_show_controller_t ctrl;
    sb_error_t err;

    err = sb_show_controller_init(&ctrl, NULL);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    /* Updating time when no screenplay is attached should return default_output */
    err = sb_show_controller_update_time_msec(&ctrl, 12345u);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    const sb_control_output_t* out = sb_show_controller_get_current_output(&ctrl);
    TEST_ASSERT_NOT_NULL(out);

    /* Should still have velocity and yaw rate set as in default_output */
    TEST_ASSERT_TRUE(sb_control_output_has_any_component_in(out, SB_CONTROL_OUTPUT_VELOCITY));
    TEST_ASSERT_TRUE(sb_control_output_has_any_component_in(out, SB_CONTROL_OUTPUT_YAW_RATE));

    TEST_ASSERT_EQUAL_FLOAT(0.0f, out->velocity.x);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out->velocity.y);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out->velocity.z);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out->velocity.yaw);

    sb_show_controller_destroy(&ctrl);
}

void test_show_controller_update_time_with_empty_screenplay_produces_no_components(void)
{
    sb_show_controller_t ctrl;
    sb_screenplay_t screenplay;
    sb_screenplay_chapter_t* chapter = NULL;
    sb_error_t err;

    /* Initialize an empty screenplay (no chapters) */
    err = sb_screenplay_init(&screenplay);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    /* Append a chapter but do not set trajectory/light/yaw -> chapter present but no players */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_append_new_chapter(&screenplay, &chapter));
    TEST_ASSERT_NOT_NULL(chapter);

    /* Initialize controller with this screenplay */
    err = sb_show_controller_init(&ctrl, &screenplay);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    /* Update time within the (first) chapter: resulting output should be cleared (no components) */
    err = sb_show_controller_update_time_msec(&ctrl, 0u);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    const sb_control_output_t* out = sb_show_controller_get_current_output(&ctrl);
    TEST_ASSERT_NOT_NULL(out);

    /* Because the chapter has no trajectory/light/yaw players, output must be empty */
    TEST_ASSERT_EQUAL_UINT8(SB_CONTROL_OUTPUT_NONE, out->mask);

    /* Clean up */
    sb_show_controller_destroy(&ctrl);
    sb_screenplay_destroy(&screenplay);
}

void test_show_controller_chapter_transition_switches_players(void)
{
    sb_screenplay_t screenplay;
    sb_screenplay_chapter_t* ch0 = NULL;
    sb_screenplay_chapter_t* ch1 = NULL;
    sb_trajectory_t* traj_empty;
    sb_trajectory_t* traj_loaded;
    sb_light_program_t* prog;
    sb_show_controller_t ctrl;
    sb_error_t err;
    FILE* fp;

    /* Initialize screenplay and two chapters */
    err = sb_screenplay_init(&screenplay);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_append_new_chapter(&screenplay, &ch0));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_append_new_chapter(&screenplay, &ch1));
    TEST_ASSERT_NOT_NULL(ch0);
    TEST_ASSERT_NOT_NULL(ch1);

    /* Prepare an empty trajectory for chapter 0 (no outputs expected) */
    TEST_ASSERT_NOT_NULL(traj_empty = sb_trajectory_new());
    sb_screenplay_chapter_set_trajectory(ch0, traj_empty);
    /* finite duration 1000 ms for first chapter */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_chapter_set_duration_msec(ch0, 1000u));

    /* For chapter 1 load real trajectory and light program from fixture */
    fp = fopen("fixtures/test.skyb", "rb");
    TEST_ASSERT_NOT_NULL(fp);
    TEST_ASSERT_NOT_NULL(traj_loaded = sb_trajectory_new());
    TEST_ASSERT_NOT_NULL(prog = sb_light_program_new());
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_update_from_binary_file(traj_loaded, fileno(fp)));
    rewind(fp);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_light_program_update_from_binary_file(prog, fileno(fp)));
    fclose(fp);

    sb_screenplay_chapter_set_trajectory(ch1, traj_loaded);
    sb_screenplay_chapter_set_light_program(ch1, prog);

    /* Initialize controller with this screenplay */
    err = sb_show_controller_init(&ctrl, &screenplay);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    /* Update time within first chapter (500 ms) -> should select chapter 0 */
    err = sb_show_controller_update_time_msec(&ctrl, 500u);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);
    TEST_ASSERT_EQUAL_PTR(ch0, ctrl.current_chapter);

    const sb_control_output_t* out0 = sb_show_controller_get_current_output(&ctrl);
    TEST_ASSERT_NOT_NULL(out0);
    /* chapter 0 has an empty trajectory and no light -> output should have position
     * and velocity outputs with zero vectors, but no lights
     */
    TEST_ASSERT_EQUAL_UINT8(SB_CONTROL_OUTPUT_POSITION | SB_CONTROL_OUTPUT_VELOCITY, out0->mask);
    TEST_ASSERT_EQUAL_VECTOR3_XYZ(0.0f, 0.0f, 0.0f, out0->position);
    TEST_ASSERT_EQUAL_VECTOR3_XYZ(0.0f, 0.0f, 0.0f, out0->velocity);

    /* Update time that falls into second chapter (1500 ms -> 500 ms into chapter 1) */
    err = sb_show_controller_update_time_msec(&ctrl, 1500u);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);
    TEST_ASSERT_EQUAL_PTR(ch1, ctrl.current_chapter);

    const sb_control_output_t* out1 = sb_show_controller_get_current_output(&ctrl);
    TEST_ASSERT_NOT_NULL(out1);

    /* chapter 1 has a trajectory and a light program -> expect position/velocity/lights */
    TEST_ASSERT_TRUE(sb_control_output_has_any_component_in(out1, SB_CONTROL_OUTPUT_POSITION));
    TEST_ASSERT_TRUE(sb_control_output_has_any_component_in(out1, SB_CONTROL_OUTPUT_VELOCITY));
    TEST_ASSERT_TRUE(sb_control_output_has_any_component_in(out1, SB_CONTROL_OUTPUT_LIGHTS));

    /* Cleanup: destroy controller and screenplay, then release static objects */
    sb_show_controller_destroy(&ctrl);
    sb_screenplay_destroy(&screenplay);

    SB_DECREF(traj_empty);
    SB_DECREF(traj_loaded);
    SB_DECREF(prog);
}

void test_show_controller_play_fixture_single_chapter(void)
{
    sb_screenplay_t screenplay;
    sb_screenplay_chapter_t* ch = NULL;
    sb_trajectory_t* traj;
    sb_light_program_t* prog;
    sb_show_controller_t ctrl;
    sb_vector3_t pos;
    sb_vector3_t vel;
    sb_rgb_color_t color;
    const sb_control_output_t* cur;
    sb_control_output_time_t output_time;
    sb_error_t err;
    FILE* fp;

    /* Initialize screenplay and single chapter */
    err = sb_screenplay_init(&screenplay);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_append_new_chapter(&screenplay, &ch));
    TEST_ASSERT_NOT_NULL(ch);

    /* Load trajectory and light program from fixture */
    fp = fopen("fixtures/test.skyb", "rb");
    TEST_ASSERT_NOT_NULL(fp);
    TEST_ASSERT_NOT_NULL(traj = sb_trajectory_new());
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_update_from_binary_file(traj, fileno(fp)));
    rewind(fp);
    TEST_ASSERT_NOT_NULL(prog = sb_light_program_new());
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_light_program_update_from_binary_file(prog, fileno(fp)));
    fclose(fp);

    sb_screenplay_chapter_set_trajectory(ch, traj);
    sb_screenplay_chapter_set_light_program(ch, prog);

    /* Initialize controller */
    err = sb_show_controller_init(&ctrl, &screenplay);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    /* Query at t=0 ms -> expect position {0,0,0}, velocity {0,0,1000}, color {255,255,255} */
    err = sb_show_controller_update_time_msec(&ctrl, 0u);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);
    cur = sb_show_controller_get_current_output(&ctrl);
    TEST_ASSERT_TRUE(sb_control_output_get_position_if_set(cur, &pos));
    TEST_ASSERT_EQUAL_VECTOR3_XYZ(0.0f, 0.0f, 0.0f, pos);
    TEST_ASSERT_TRUE(sb_control_output_get_velocity_if_set(cur, &vel));
    TEST_ASSERT_EQUAL_VECTOR3_XYZ(0.0f, 0.0f, 1000.0f, vel);
    TEST_ASSERT_TRUE(sb_control_output_get_color_if_set(cur, &color));
    TEST_ASSERT_EQUAL_COLOR_RGB(255, 255, 255, color);

    /* Timestamps should be reported correctly */
    TEST_ASSERT_TRUE(sb_show_controller_is_output_valid(&ctrl));
    output_time = sb_show_controller_get_current_output_time(&ctrl);
    TEST_ASSERT_EQUAL_UINT32(0u, output_time.time_msec);
    TEST_ASSERT_EQUAL_UINT32(0u, output_time.chapter);
    TEST_ASSERT_EQUAL_UINT32(0u, output_time.time_in_chapter_msec);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, output_time.warped_time_in_chapter_sec);

    /* Query at t=5000 ms (5s) -> expect position {0,0,5000}, velocity {0,0,1000}, color {255,127,127} */
    err = sb_show_controller_update_time_msec(&ctrl, 5000u);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);
    cur = sb_show_controller_get_current_output(&ctrl);
    TEST_ASSERT_TRUE(sb_control_output_get_position_if_set(cur, &pos));
    TEST_ASSERT_EQUAL_VECTOR3_XYZ(0.0f, 0.0f, 5000.0f, pos);
    TEST_ASSERT_TRUE(sb_control_output_get_velocity_if_set(cur, &vel));
    TEST_ASSERT_EQUAL_VECTOR3_XYZ(0.0f, 0.0f, 1000.0f, vel);
    TEST_ASSERT_TRUE(sb_control_output_get_color_if_set(cur, &color));
    TEST_ASSERT_EQUAL_COLOR_RGB(255, 127, 127, color);

    /* Timestamps should be reported correctly */
    TEST_ASSERT_TRUE(sb_show_controller_is_output_valid(&ctrl));
    output_time = sb_show_controller_get_current_output_time(&ctrl);
    TEST_ASSERT_EQUAL_UINT32(5000u, output_time.time_msec);
    TEST_ASSERT_EQUAL_UINT32(0u, output_time.chapter);
    TEST_ASSERT_EQUAL_UINT32(5000u, output_time.time_in_chapter_msec);
    TEST_ASSERT_EQUAL_FLOAT(5.0f, output_time.warped_time_in_chapter_sec);

    /* Query at t=15000 ms (15s) -> expect position {5000,0,10000}, velocity {1000,0,0}, color {255,0,0} */
    err = sb_show_controller_update_time_msec(&ctrl, 15000u);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);
    cur = sb_show_controller_get_current_output(&ctrl);
    TEST_ASSERT_TRUE(sb_control_output_get_position_if_set(cur, &pos));
    TEST_ASSERT_EQUAL_VECTOR3_XYZ(5000.0f, 0.0f, 10000.0f, pos);
    TEST_ASSERT_TRUE(sb_control_output_get_velocity_if_set(cur, &vel));
    TEST_ASSERT_EQUAL_VECTOR3_XYZ(1000.0f, 0.0f, 0.0f, vel);
    TEST_ASSERT_TRUE(sb_control_output_get_color_if_set(cur, &color));
    TEST_ASSERT_EQUAL_COLOR_RGB(255, 0, 0, color);

    /* Timestamps should be reported correctly */
    TEST_ASSERT_TRUE(sb_show_controller_is_output_valid(&ctrl));
    output_time = sb_show_controller_get_current_output_time(&ctrl);
    TEST_ASSERT_EQUAL_UINT32(15000u, output_time.time_msec);
    TEST_ASSERT_EQUAL_UINT32(0u, output_time.chapter);
    TEST_ASSERT_EQUAL_UINT32(15000u, output_time.time_in_chapter_msec);
    TEST_ASSERT_EQUAL_FLOAT(15.0f, output_time.warped_time_in_chapter_sec);

    /* Cleanup */
    sb_show_controller_destroy(&ctrl);
    sb_screenplay_destroy(&screenplay);
    SB_DECREF(traj);
    SB_DECREF(prog);
}

void test_show_controller_play_fixture_time_axis_2x(void)
{
    sb_screenplay_t screenplay;
    sb_screenplay_chapter_t* ch = NULL;
    sb_trajectory_t* traj;
    sb_light_program_t* prog;
    sb_show_controller_t ctrl;
    sb_vector3_t pos;
    sb_vector3_t vel;
    sb_rgb_color_t color;
    const sb_control_output_t* cur;
    sb_error_t err;
    FILE* fp;

    /* Initialize screenplay and single chapter */
    err = sb_screenplay_init(&screenplay);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_append_new_chapter(&screenplay, &ch));
    TEST_ASSERT_NOT_NULL(ch);

    /* Load trajectory and light program from fixture */
    fp = fopen("fixtures/test.skyb", "rb");
    TEST_ASSERT_NOT_NULL(fp);
    TEST_ASSERT_NOT_NULL(traj = sb_trajectory_new());
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_update_from_binary_file(traj, fileno(fp)));
    rewind(fp);
    TEST_ASSERT_NOT_NULL(prog = sb_light_program_new());
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_light_program_update_from_binary_file(prog, fileno(fp)));
    fclose(fp);

    sb_screenplay_chapter_set_trajectory(ch, traj);
    sb_screenplay_chapter_set_light_program(ch, prog);

    /* Alter the chapter time axis to run at 2x real-time for the duration of the
     * fixture so warped_time = 2 * wall_clock_time.
     */
    sb_time_axis_t* axis = sb_screenplay_chapter_get_time_axis(ch);
    sb_time_segment_t seg = sb_time_segment_make_constant_rate(60000, 2.0f);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_append_segment(axis, seg));

    /* Initialize controller */
    err = sb_show_controller_init(&ctrl, &screenplay);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    /* Query at t=0 ms -> warped_time 0s -> expect position {0,0,0}, velocity doubled */
    err = sb_show_controller_update_time_msec(&ctrl, 0u);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);
    cur = sb_show_controller_get_current_output(&ctrl);
    TEST_ASSERT_TRUE(sb_control_output_get_position_if_set(cur, &pos));
    TEST_ASSERT_EQUAL_VECTOR3_XYZ(0.0f, 0.0f, 0.0f, pos);
    TEST_ASSERT_TRUE(sb_control_output_get_velocity_if_set(cur, &vel));
    /* base trajectory velocity at t=0 is {0,0,1000} -> with 2x rate becomes {0,0,2000} */
    TEST_ASSERT_EQUAL_VECTOR3_XYZ(0.0f, 0.0f, 2000.0f, vel);
    TEST_ASSERT_TRUE(sb_control_output_get_color_if_set(cur, &color));
    TEST_ASSERT_EQUAL_COLOR_RGB(255, 255, 255, color);

    /* Query at t=2500 ms (2.5s wall clock) -> warped=5s -> expect position {0,0,5000},
     * base velocity {0,0,1000} but multiplied by 2 => {0,0,2000}, color at 5s.
     */
    err = sb_show_controller_update_time_msec(&ctrl, 2500u);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);
    cur = sb_show_controller_get_current_output(&ctrl);
    TEST_ASSERT_TRUE(sb_control_output_get_position_if_set(cur, &pos));
    TEST_ASSERT_EQUAL_VECTOR3_XYZ(0.0f, 0.0f, 5000.0f, pos);
    TEST_ASSERT_TRUE(sb_control_output_get_velocity_if_set(cur, &vel));
    TEST_ASSERT_EQUAL_VECTOR3_XYZ(0.0f, 0.0f, 2000.0f, vel);
    TEST_ASSERT_TRUE(sb_control_output_get_color_if_set(cur, &color));
    TEST_ASSERT_EQUAL_COLOR_RGB(255, 127, 127, color);

    /* Query at t=7500 ms (7.5s wall clock) -> warped=15s -> expect position {5000,0,10000},
     * base velocity at 15s {1000,0,0} -> doubled {2000,0,0}, color at 15s.
     */
    err = sb_show_controller_update_time_msec(&ctrl, 7500u);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);
    cur = sb_show_controller_get_current_output(&ctrl);
    TEST_ASSERT_TRUE(sb_control_output_get_position_if_set(cur, &pos));
    TEST_ASSERT_EQUAL_VECTOR3_XYZ(5000.0f, 0.0f, 10000.0f, pos);
    TEST_ASSERT_TRUE(sb_control_output_get_velocity_if_set(cur, &vel));
    TEST_ASSERT_EQUAL_VECTOR3_XYZ(2000.0f, 0.0f, 0.0f, vel);
    TEST_ASSERT_TRUE(sb_control_output_get_color_if_set(cur, &color));
    TEST_ASSERT_EQUAL_COLOR_RGB(255, 0, 0, color);

    /* Cleanup */
    sb_show_controller_destroy(&ctrl);
    sb_screenplay_destroy(&screenplay);
    SB_DECREF(traj);
    SB_DECREF(prog);
}

/* forward_left_back fixture played with a time axis that runs at real-time
 * for 25 seconds and then slows down linearly to a standstill over the next 5 seconds.
 *
 * We verify positions and velocities at representative wall-clock times before and
 * during the slowdown so we ensure the controller multiplies velocities by the
 * instantaneous warped rate.
 */
void test_show_controller_forward_left_back_slowdown(void)
{
    sb_screenplay_t screenplay;
    sb_screenplay_chapter_t* ch = NULL;
    sb_trajectory_t* traj;
    sb_light_program_t* prog;
    sb_show_controller_t ctrl;
    sb_vector3_t pos;
    sb_vector3_t vel;
    const sb_control_output_t* cur;
    sb_error_t err;
    FILE* fp;

    /* Initialize screenplay and chapter */
    err = sb_screenplay_init(&screenplay);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_append_new_chapter(&screenplay, &ch));
    TEST_ASSERT_NOT_NULL(ch);

    /* Load the forward_left_back fixture (trajectory + light program) */
    fp = fopen("fixtures/forward_left_back.skyb", "rb");
    TEST_ASSERT_NOT_NULL(fp);
    TEST_ASSERT_NOT_NULL(traj = sb_trajectory_new());
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_update_from_binary_file(traj, fileno(fp)));
    rewind(fp);
    TEST_ASSERT_NOT_NULL(prog = sb_light_program_new());
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_light_program_update_from_binary_file(prog, fileno(fp)));
    fclose(fp);

    sb_screenplay_chapter_set_trajectory(ch, traj);
    sb_screenplay_chapter_set_light_program(ch, prog);

    /* Set time axis: 25s at rate=1.0 (normal), then slowdown from realtime to 0 over 5s */
    sb_time_axis_t* axis = sb_screenplay_chapter_get_time_axis(ch);
    TEST_ASSERT_NOT_NULL(axis);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_append_segment(axis, sb_time_segment_make_constant_rate(25000, 1.0f)));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_append_segment(axis, sb_time_segment_make_slowdown_from_realtime(5000)));

    /* Initialize controller */
    err = sb_show_controller_init(&ctrl, &screenplay);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    /* 1) Wall = 25s -> warped = 25s (normal section) -> halfway through left move:
     *    expected position {10000, 5000, 10000} and base lateral velocity approx 1125.7 mm/s
     */
    err = sb_show_controller_update_time_msec(&ctrl, 25000u);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);
    cur = sb_show_controller_get_current_output(&ctrl);
    TEST_ASSERT_TRUE(sb_control_output_get_position_if_set(cur, &pos));
    TEST_ASSERT_TRUE(sb_control_output_get_velocity_if_set(cur, &vel));
    TEST_ASSERT_EQUAL_VECTOR3_XYZ_EPS(10000.0f, 5000.0f, 10000.0f, pos, 1e-1);
    /* Expect lateral velocity (y) approx 1125.7 mm/s (from trajectory tests), x and z near 0 */
    TEST_ASSERT_EQUAL_VECTOR3_XYZ_EPS(0.0f, 1125.7f, 0.0f, vel, 1e-1);

    /* 2) Wall = 27.5s (mid slowdown): wall into slowdown = 2.5s (relative_t = 0.5)
     *    warped increment inside slowdown = 1.875s -> warped total = 26.875s
     *    This is still within the left move, expected y ~ 7111 mm.
     *    (This is because 1.875s * 1125.7 mm/s = 2111.8 mm + 5000 mm start of left move)
     *    The instantaneous warped rate is 0.5 so velocity should be half the base value.
     */
    err = sb_show_controller_update_time_msec(&ctrl, 27500u);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);
    cur = sb_show_controller_get_current_output(&ctrl);
    TEST_ASSERT_TRUE(sb_control_output_get_position_if_set(cur, &pos));
    TEST_ASSERT_TRUE(sb_control_output_get_velocity_if_set(cur, &vel));
    TEST_ASSERT_EQUAL_VECTOR3_XYZ_EPS(10000.0f, 7110.8f, 10000.0f, pos, 1e-1);
    /* velocity should be ~1125.7 * 0.5 = 562.85 mm/s in Y */
    TEST_ASSERT_EQUAL_VECTOR3_XYZ_EPS(0.0f, 562.85f, 0.0f, vel, 1e-1);

    /* 3) Wall = 30s (end of slowdown): warped total = 27.5s (still inside left move),
     *    expected y = 7814 mm. The instantaneous rate at this boundary is 0.0,
     *    so controller should report zero velocity.
     */
    err = sb_show_controller_update_time_msec(&ctrl, 30000u);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);
    cur = sb_show_controller_get_current_output(&ctrl);
    TEST_ASSERT_TRUE(sb_control_output_get_position_if_set(cur, &pos));
    /* velocity component may be set but should be zero due to final rate zero */
    TEST_ASSERT_TRUE(sb_control_output_get_velocity_if_set(cur, &vel));
    TEST_ASSERT_EQUAL_VECTOR3_XYZ_EPS(10000.0f, 7814.4f, 10000.0f, pos, 1e-1);
    TEST_ASSERT_EQUAL_VECTOR3_XYZ_EPS(0.0f, 0.0f, 0.0f, vel, 1e-1);

    /* Cleanup */
    sb_show_controller_destroy(&ctrl);
    sb_screenplay_destroy(&screenplay);
    SB_DECREF(traj);
    SB_DECREF(prog);
}

/*
 * Integration test: verify that when a chapter has a yaw control object the
 * show controller exposes yaw and yaw rate components and produces values
 * matching the yaw player (fixture: fixtures/test.skyb).
 *
 * The numerical expectations are taken from the yaw player unit tests that
 * use the same fixture.
 */
void test_show_controller_play_fixture_with_yaw_control(void)
{
    sb_screenplay_t screenplay;
    sb_screenplay_chapter_t* ch = NULL;
    sb_yaw_control_t* yaw;
    sb_show_controller_t ctrl;
    const sb_control_output_t* out;
    sb_error_t err;
    FILE* fp;
    float yaw_val;
    float yaw_rate;

    /* Initialize screenplay and single chapter */
    err = sb_screenplay_init(&screenplay);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_append_new_chapter(&screenplay, &ch));
    TEST_ASSERT_NOT_NULL(ch);

    /* Load yaw control from fixture */
    fp = fopen("fixtures/test.skyb", "rb");
    TEST_ASSERT_NOT_NULL(fp);
    TEST_ASSERT_NOT_NULL(yaw = sb_yaw_control_new());
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_yaw_control_update_from_binary_file(yaw, fileno(fp)));
    fclose(fp);

    /* Attach yaw control to chapter (no trajectory / lights needed for this test) */
    sb_screenplay_chapter_set_yaw_control(ch, yaw);

    /* Initialize controller with this screenplay */
    err = sb_show_controller_init(&ctrl, &screenplay);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    /* --- t = 0 ms --- */
    err = sb_show_controller_update_time_msec(&ctrl, 0u);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);
    out = sb_show_controller_get_current_output(&ctrl);
    TEST_ASSERT_NOT_NULL(out);

    /* Expect only yaw + yaw_rate components present */
    TEST_ASSERT_EQUAL_UINT8(SB_CONTROL_OUTPUT_YAW | SB_CONTROL_OUTPUT_YAW_RATE, out->mask);

    /* Yaw at t=0 should match fixture (4.0 deg) */
    TEST_ASSERT_TRUE(sb_control_output_get_yaw_if_set(out, &yaw_val));
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 4.0f, yaw_val);

    /* Yaw rate at t=0 should match fixture (~200 deg/s) */
    TEST_ASSERT_TRUE(sb_control_output_get_yaw_rate_if_set(out, &yaw_rate));
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 200.0f, yaw_rate);

    /* --- t = 1 ms --- */
    err = sb_show_controller_update_time_msec(&ctrl, 1u);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);
    out = sb_show_controller_get_current_output(&ctrl);
    TEST_ASSERT_NOT_NULL(out);

    TEST_ASSERT_TRUE(sb_control_output_get_yaw_if_set(out, &yaw_val));
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 4.2f, yaw_val);

    TEST_ASSERT_TRUE(sb_control_output_get_yaw_rate_if_set(out, &yaw_rate));
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 200.0f, yaw_rate);

    /* --- t = 4 ms --- */
    err = sb_show_controller_update_time_msec(&ctrl, 4u);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);
    out = sb_show_controller_get_current_output(&ctrl);
    TEST_ASSERT_NOT_NULL(out);

    TEST_ASSERT_TRUE(sb_control_output_get_yaw_if_set(out, &yaw_val));
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 4.6f, yaw_val);

    TEST_ASSERT_TRUE(sb_control_output_get_yaw_rate_if_set(out, &yaw_rate));
    TEST_ASSERT_FLOAT_WITHIN(1e-3, 400.0 / 3, yaw_rate);

    /* Cleanup */
    sb_show_controller_destroy(&ctrl);
    sb_screenplay_destroy(&screenplay);
    SB_DECREF(yaw);
}

void test_show_controller_get_current_chapter(void)
{
    sb_show_controller_t ctrl;
    sb_screenplay_t screenplay;
    sb_screenplay_chapter_t* ch0 = NULL;
    sb_screenplay_chapter_t* ch1 = NULL;
    sb_error_t err;

    /* Initialize a screenplay with two chapters */
    err = sb_screenplay_init(&screenplay);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_append_new_chapter(&screenplay, &ch0));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_append_new_chapter(&screenplay, &ch1));
    TEST_ASSERT_NOT_NULL(ch0);
    TEST_ASSERT_NOT_NULL(ch1);

    /* Give each chapter a finite duration of 1000 ms so the screenplay spans 0..2000 ms */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_chapter_set_duration_msec(ch0, 1000u));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_chapter_set_duration_msec(ch1, 1000u));

    /* Initialize controller with the screenplay */
    err = sb_show_controller_init(&ctrl, &screenplay);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    /* Before any update, current chapter should be NULL (not yet established) */
    TEST_ASSERT_NULL(sb_show_controller_get_current_chapter(&ctrl));

    /* Update time within first chapter -> get_current_chapter should return ch0 */
    err = sb_show_controller_update_time_msec(&ctrl, 500u);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);
    TEST_ASSERT_EQUAL_PTR(ch0, sb_show_controller_get_current_chapter(&ctrl));

    /* Update time within second chapter -> get_current_chapter should return ch1 */
    err = sb_show_controller_update_time_msec(&ctrl, 1500u);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);
    TEST_ASSERT_EQUAL_PTR(ch1, sb_show_controller_get_current_chapter(&ctrl));

    /* Update time out of bounds -> no current chapter */
    err = sb_show_controller_update_time_msec(&ctrl, 3000u);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);
    TEST_ASSERT_NULL(sb_show_controller_get_current_chapter(&ctrl));

    /* Cleanup */
    sb_show_controller_destroy(&ctrl);
    sb_screenplay_destroy(&screenplay);
}

/* Steps:
 *  - construct a show controller with a chapter loaded from fixtures/test.skyb
 *  - query the show controller at t=5000 ms (within fixture range)
 *  - query it again at same timestamp
 *  - invalidate the show controller
 *  - query it again at same timestamp
 *
 * We assert that update_time_msec returns success and that the controller's
 * internal output_time_msec reflects caching/invalidation.
 */
void test_show_controller_invalidate_cached_output(void)
{
    sb_screenplay_t screenplay;
    sb_screenplay_chapter_t* ch = NULL;
    sb_trajectory_t* traj;
    sb_light_program_t* prog;
    sb_show_controller_t ctrl;
    const sb_control_output_t* out;
    sb_control_output_time_t output_time;
    sb_error_t err;
    FILE* fp;

    /* Initialize screenplay and single chapter */
    err = sb_screenplay_init(&screenplay);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_append_new_chapter(&screenplay, &ch));
    TEST_ASSERT_NOT_NULL(ch);

    /* Load trajectory and light program from fixture */
    fp = fopen("fixtures/test.skyb", "rb");
    TEST_ASSERT_NOT_NULL(fp);
    TEST_ASSERT_NOT_NULL(traj = sb_trajectory_new());
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_update_from_binary_file(traj, fileno(fp)));
    rewind(fp);
    TEST_ASSERT_NOT_NULL(prog = sb_light_program_new());
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_light_program_update_from_binary_file(prog, fileno(fp)));
    fclose(fp);

    sb_screenplay_chapter_set_trajectory(ch, traj);
    sb_screenplay_chapter_set_light_program(ch, prog);

    /* Initialize controller */
    err = sb_show_controller_init(&ctrl, &screenplay);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    /* First query at t=5000 ms */
    err = sb_show_controller_update_time_msec(&ctrl, 5000u);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);
    out = sb_show_controller_get_current_output(&ctrl);
    TEST_ASSERT_NOT_NULL(out);
    /* Expect position/velocity/lights present for this fixture */
    TEST_ASSERT_TRUE(sb_control_output_has_any_component_in(out, SB_CONTROL_OUTPUT_POSITION));
    TEST_ASSERT_TRUE(sb_control_output_has_any_component_in(out, SB_CONTROL_OUTPUT_VELOCITY));
    TEST_ASSERT_TRUE(sb_control_output_has_any_component_in(out, SB_CONTROL_OUTPUT_LIGHTS));
    /* internal cached timestamp should be 5000 */
    output_time = sb_show_controller_get_current_output_time(&ctrl);
    TEST_ASSERT_EQUAL_UINT32(5000u, output_time.time_msec);

    /* Query again at the same timestamp: cache should be valid and timestamp unchanged */
    err = sb_show_controller_update_time_msec(&ctrl, 5000u);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);
    output_time = sb_show_controller_get_current_output_time(&ctrl);
    TEST_ASSERT_EQUAL_UINT32(5000u, output_time.time_msec);

    /* Invalidate cached output */
    sb_show_controller_invalidate_output(&ctrl);
    output_time = sb_show_controller_get_current_output_time(&ctrl);
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, output_time.time_msec);

    /* Query again after invalidation: controller should recompute and set timestamp */
    err = sb_show_controller_update_time_msec(&ctrl, 5000u);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);
    output_time = sb_show_controller_get_current_output_time(&ctrl);
    TEST_ASSERT_EQUAL_UINT32(5000u, output_time.time_msec);
    out = sb_show_controller_get_current_output(&ctrl);
    TEST_ASSERT_NOT_NULL(out);
    TEST_ASSERT_TRUE(sb_control_output_has_any_component_in(out, SB_CONTROL_OUTPUT_POSITION));
    TEST_ASSERT_TRUE(sb_control_output_has_any_component_in(out, SB_CONTROL_OUTPUT_VELOCITY));
    TEST_ASSERT_TRUE(sb_control_output_has_any_component_in(out, SB_CONTROL_OUTPUT_LIGHTS));

    /* Cleanup */
    sb_show_controller_destroy(&ctrl);
    sb_screenplay_destroy(&screenplay);
    SB_DECREF(traj);
    SB_DECREF(prog);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_show_controller_init_sets_defaults_and_get_current_output);
    RUN_TEST(test_show_controller_update_time_without_screenplay_returns_default);
    RUN_TEST(test_show_controller_update_time_with_empty_screenplay_produces_no_components);
    RUN_TEST(test_show_controller_chapter_transition_switches_players);
    RUN_TEST(test_show_controller_play_fixture_single_chapter);
    RUN_TEST(test_show_controller_play_fixture_time_axis_2x);
    RUN_TEST(test_show_controller_forward_left_back_slowdown);
    RUN_TEST(test_show_controller_play_fixture_with_yaw_control);
    RUN_TEST(test_show_controller_invalidate_cached_output);
    RUN_TEST(test_show_controller_get_current_chapter);

    return UNITY_END();
}
