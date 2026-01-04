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
    sb_trajectory_t traj_empty;
    sb_trajectory_t traj_loaded;
    sb_light_program_t prog;
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
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_init_empty(&traj_empty));
    sb_screenplay_chapter_set_trajectory(ch0, &traj_empty);
    /* finite duration 1000 ms for first chapter */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_screenplay_chapter_set_duration_msec(ch0, 1000u));

    /* For chapter 1 load real trajectory and light program from fixture */
    fp = fopen("fixtures/test.skyb", "rb");
    TEST_ASSERT_NOT_NULL(fp);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_init_from_binary_file(&traj_loaded, fileno(fp)));
    fclose(fp);

    fp = fopen("fixtures/test.skyb", "rb");
    TEST_ASSERT_NOT_NULL(fp);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_light_program_init_from_binary_file(&prog, fileno(fp)));
    fclose(fp);

    sb_screenplay_chapter_set_trajectory(ch1, &traj_loaded);
    sb_screenplay_chapter_set_light_program(ch1, &prog);

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

    SB_DECREF_STATIC(&traj_empty);
    SB_DECREF_STATIC(&traj_loaded);
    SB_DECREF_STATIC(&prog);
}

void test_show_controller_play_fixture_single_chapter(void)
{
    sb_screenplay_t screenplay;
    sb_screenplay_chapter_t* ch = NULL;
    sb_trajectory_t traj;
    sb_light_program_t prog;
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
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_init_from_binary_file(&traj, fileno(fp)));
    fclose(fp);

    fp = fopen("fixtures/test.skyb", "rb");
    TEST_ASSERT_NOT_NULL(fp);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_light_program_init_from_binary_file(&prog, fileno(fp)));
    fclose(fp);

    sb_screenplay_chapter_set_trajectory(ch, &traj);
    sb_screenplay_chapter_set_light_program(ch, &prog);

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

    /* Cleanup */
    sb_show_controller_destroy(&ctrl);
    sb_screenplay_destroy(&screenplay);
    SB_DECREF_STATIC(&traj);
    SB_DECREF_STATIC(&prog);
}

void test_show_controller_play_fixture_time_axis_2x(void)
{
    sb_screenplay_t screenplay;
    sb_screenplay_chapter_t* ch = NULL;
    sb_trajectory_t traj;
    sb_light_program_t prog;
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
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_init_from_binary_file(&traj, fileno(fp)));
    fclose(fp);

    fp = fopen("fixtures/test.skyb", "rb");
    TEST_ASSERT_NOT_NULL(fp);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_light_program_init_from_binary_file(&prog, fileno(fp)));
    fclose(fp);

    sb_screenplay_chapter_set_trajectory(ch, &traj);
    sb_screenplay_chapter_set_light_program(ch, &prog);

    /* Alter the chapter time axis to run at 2x real-time for the duration of the
     * fixture so warped_time = 2 * wall_clock_time.
     */
    sb_time_axis_t* axis = sb_screenplay_chapter_get_time_axis(ch);
    sb_time_segment_t seg = sb_time_segment_make_constant_rate(60.0f, 2.0f);
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

    /* Query at t=2500 ms (2.5s wall clock) -> warped_time = 5s -> expect position {0,0,5000},
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

    /* Query at t=7500 ms (7.5s wall clock) -> warped_time = 15s -> expect position {5000,0,10000},
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
    SB_DECREF_STATIC(&traj);
    SB_DECREF_STATIC(&prog);
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

    return UNITY_END();
}
