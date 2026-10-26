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

/**
 * @file fuzz_show.c
 * @brief Fuzz target that loads a show file and plays all of its blocks.
 *
 * When compiled with \c FUZZ_WRAP_BLOCK defined, the input is not a show file
 * but a single block: the first byte is the type of the block and the rest is
 * the body of the block. The block is wrapped in a version 1 show file (which
 * has no checksum) before it is loaded so the fuzzer does not need to discover
 * the structure of the file header to reach the block parsers.
 *
 * Errors returned by the library are ignored; the target looks for crashes,
 * memory errors, undefined behaviour and hangs only.
 */

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <skybrush/control.h>
#include <skybrush/events.h>
#include <skybrush/gcs_light_control.h>
#include <skybrush/lights.h>
#include <skybrush/metadata.h>
#include <skybrush/refcount.h>
#include <skybrush/rth_plan.h>
#include <skybrush/screenplay.h>
#include <skybrush/stats.h>
#include <skybrush/trajectory.h>
#include <skybrush/yaw_control.h>

/* Number of evenly spaced samples taken from each player */
#define NUM_SAMPLES 64

/* Largest number of steps when seeking through a light program */
#define MAX_LIGHT_PLAYER_STEPS 4096

/* Upper bound of the evenly spaced sample timestamps */
#define MAX_SAMPLE_TIME_MSEC (3600u * 1000u)

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

/* Sample timestamps: evenly spaced ones plus a few extreme values */
static uint32_t sample_time_msec(size_t index)
{
    static const uint32_t extremes[] = { 0, 1, 19, 20, 21, INT32_MAX, (uint32_t)INT32_MAX + 1, UINT32_MAX };
    const size_t num_extremes = sizeof(extremes) / sizeof(extremes[0]);

    if (index < num_extremes) {
        return extremes[index];
    } else {
        return (uint32_t)((index - num_extremes) * (MAX_SAMPLE_TIME_MSEC / NUM_SAMPLES));
    }
}

static void exercise_trajectory(sb_trajectory_t* trajectory)
{
    sb_trajectory_player_t player;
    sb_trajectory_stats_calculator_t calc;
    sb_trajectory_stats_t stats;
    sb_vector3_with_yaw_t vec;
    sb_vector3_t landing;
    size_t i;
    float t;

    if (sb_trajectory_player_init(&player, trajectory) == SB_SUCCESS) {
        for (i = 0; i < NUM_SAMPLES; i++) {
            t = sample_time_msec(i) / 1000.0f;
            sb_trajectory_player_get_position_at(&player, t, &vec);
            sb_trajectory_player_get_velocity_at(&player, t, &vec);
            sb_trajectory_player_get_acceleration_at(&player, t, &vec);
        }
        sb_trajectory_player_get_position_at(&player, -1.0f, &vec);
        sb_trajectory_player_get_position_at(&player, NAN, &vec);
        sb_trajectory_player_get_position_at(&player, INFINITY, &vec);
        sb_trajectory_player_destroy(&player);
    }

    sb_trajectory_propose_takeoff_time_sec(trajectory, 2500.0f, 1000.0f, 1000.0f);
    sb_trajectory_propose_landing_time_sec(trajectory, 2500.0f, 500.0f);

    if (sb_trajectory_stats_init(&stats) != SB_SUCCESS) {
        return; /* LCOV_EXCL_LINE */
    }

    if (sb_trajectory_stats_calculator_init(&calc, 1000.0f) == SB_SUCCESS) {
        if (sb_trajectory_stats_calculator_run(&calc, trajectory, &stats) == SB_SUCCESS) {
            /* Modifies the trajectory, therefore this must come last */
            landing.x = stats.initial_pos.x;
            landing.y = stats.initial_pos.y;
            landing.z = stats.initial_pos.z;
            sb_trajectory_replace_end_to_land_at(trajectory, &stats, landing, 500.0f);
        }
        sb_trajectory_stats_calculator_destroy(&calc);
    }

    sb_trajectory_stats_destroy(&stats);
}

static void exercise_light_program(sb_light_program_t* program)
{
    sb_light_player_t player;
    unsigned long t = 0, next = 0;
    size_t i;

    if (sb_light_player_init(&player, program) != SB_SUCCESS) {
        return;
    }

    for (i = 0; i < MAX_LIGHT_PLAYER_STEPS; i++) {
        if (sb_light_player_seek(&player, t, &next)) {
            break;
        }
        t = next > t ? next : t + 1;
    }

    for (i = 0; i < NUM_SAMPLES; i++) {
        sb_light_player_get_color_at(&player, sample_time_msec(i));
    }

    sb_light_player_destroy(&player);
}

static void exercise_yaw_control(sb_yaw_control_t* ctrl)
{
    sb_yaw_player_t player;
    uint32_t duration_msec;
    float value, t;
    size_t i;

    if (sb_yaw_player_init(&player, ctrl) != SB_SUCCESS) {
        return;
    }

    sb_yaw_player_get_total_duration_msec(&player, &duration_msec);
    for (i = 0; i < NUM_SAMPLES; i++) {
        t = sample_time_msec(i) / 1000.0f;
        sb_yaw_player_get_yaw_at(&player, t, &value);
        sb_yaw_player_get_yaw_rate_at(&player, t, &value);
    }

    sb_yaw_player_destroy(&player);
}

static void exercise_events(sb_event_list_t* events)
{
    sb_event_list_player_t player;
    size_t i;

    if (sb_event_list_player_init(&player, events) != SB_SUCCESS) {
        return;
    }

    while (sb_event_list_player_get_next_event(&player)) {
    }

    for (i = 0; i < NUM_SAMPLES; i++) {
        sb_event_list_player_seek(&player, sample_time_msec(i) / 1000.0f);
        sb_event_list_player_get_next_event_not_later_than(&player, sample_time_msec(i) / 1000.0f + 10.0f);
    }

    sb_event_list_player_destroy(&player);
}

static void exercise_rth_plan(sb_rth_plan_t* plan)
{
    sb_rth_plan_entry_t entry;
    sb_trajectory_t trajectory;
    sb_vector3_t start = { 0, 0, 10000.0f };
    size_t i;

    if (sb_trajectory_init(&trajectory) != SB_SUCCESS) {
        return; /* LCOV_EXCL_LINE */
    }

    for (i = 0; i < NUM_SAMPLES; i++) {
        if (sb_rth_plan_evaluate_at(plan, sample_time_msec(i) / 1000.0f, &entry) == SB_SUCCESS) {
            if (sb_trajectory_update_from_rth_plan_entry(&trajectory, &entry, start) == SB_SUCCESS) {
                exercise_trajectory(&trajectory);
            }
        }
    }

    SB_DECREF_STATIC(&trajectory);
}

static void exercise_show_controller(uint8_t* buf, size_t size)
{
    sb_screenplay_t screenplay;
    sb_show_controller_t ctrl;
    size_t i;

    if (sb_screenplay_init(&screenplay) != SB_SUCCESS) {
        return; /* LCOV_EXCL_LINE */
    }

    if (sb_screenplay_update_from_binary_file_in_memory(&screenplay, buf, size) == SB_SUCCESS) {
        if (sb_show_controller_init(&ctrl, &screenplay) == SB_SUCCESS) {
            for (i = 0; i < NUM_SAMPLES; i++) {
                sb_show_controller_update_time_msec(&ctrl, sample_time_msec(i));
                sb_show_controller_get_current_output(&ctrl);
                while (sb_show_controller_get_next_event(&ctrl)) {
                }
            }
            sb_show_controller_destroy(&ctrl);
        }
    }

    sb_screenplay_destroy(&screenplay);
}

static void exercise_show_file(uint8_t* buf, size_t size)
{
    sb_trajectory_t trajectory;
    sb_light_program_t program;
    sb_yaw_control_t yaw_control;
    sb_event_list_t events;
    sb_rth_plan_t rth_plan;
    sb_gcs_light_control_setup_t gcs_setup;
    sb_show_metadata_t metadata;
    uint8_t show_id[SB_SHOW_METADATA_SHOW_ID_LENGTH];
    uint16_t drone_index;

    if (sb_trajectory_init(&trajectory) == SB_SUCCESS) {
        if (sb_trajectory_update_from_binary_file_in_memory(&trajectory, buf, size) == SB_SUCCESS) {
            exercise_trajectory(&trajectory);
        }
        SB_DECREF_STATIC(&trajectory);
    }

    if (sb_light_program_init(&program) == SB_SUCCESS) {
        if (sb_light_program_update_from_binary_file_in_memory(&program, buf, size) == SB_SUCCESS) {
            exercise_light_program(&program);
        }
        SB_DECREF_STATIC(&program);
    }

    if (sb_yaw_control_init(&yaw_control) == SB_SUCCESS) {
        if (sb_yaw_control_update_from_binary_file_in_memory(&yaw_control, buf, size) == SB_SUCCESS) {
            exercise_yaw_control(&yaw_control);
        }
        SB_DECREF_STATIC(&yaw_control);
    }

    if (sb_event_list_init(&events, 0) == SB_SUCCESS) {
        if (sb_event_list_update_from_binary_file_in_memory(&events, buf, size) == SB_SUCCESS) {
            exercise_events(&events);
        }
        SB_DECREF_STATIC(&events);
    }

    if (sb_rth_plan_init(&rth_plan) == SB_SUCCESS) {
        if (sb_rth_plan_update_from_binary_file_in_memory(&rth_plan, buf, size) == SB_SUCCESS) {
            exercise_rth_plan(&rth_plan);
        }
        SB_DECREF_STATIC(&rth_plan);
    }

    if (sb_gcs_light_control_setup_init(&gcs_setup) == SB_SUCCESS) {
        sb_gcs_light_control_setup_update_from_binary_file_in_memory(&gcs_setup, buf, size);
        sb_gcs_light_control_setup_destroy(&gcs_setup);
    }

    if (sb_show_metadata_init(&metadata) == SB_SUCCESS) {
        if (sb_show_metadata_update_from_binary_file_in_memory(&metadata, buf, size) == SB_SUCCESS) {
            sb_show_metadata_get_show_id(&metadata, show_id);
            sb_show_metadata_get_drone_index(&metadata, &drone_index);
        }
        sb_show_metadata_destroy(&metadata);
    }

    exercise_show_controller(buf, size);
}

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
    uint8_t* buf;
    size_t buf_size;

#ifdef FUZZ_WRAP_BLOCK
    /* "skyb", version 1, block type, 16-bit little-endian block length, body */
    size_t body_size;

    if (size < 1) {
        return 0;
    }

    body_size = size - 1;
    if (body_size > UINT16_MAX) {
        body_size = UINT16_MAX;
    }

    buf_size = 5 + 3 + body_size;
    buf = malloc(buf_size);
    if (buf == 0) {
        return 0;
    }

    memcpy(buf, "skyb\x01", 5);
    buf[5] = data[0];
    buf[6] = body_size & 0xff;
    buf[7] = (body_size >> 8) & 0xff;
    memcpy(buf + 8, data + 1, body_size);
#else
    /* Copy the input so the library can take a non-const pointer, and so
     * that the address sanitizer catches reads past the end of the input */
    buf_size = size;
    buf = malloc(buf_size > 0 ? buf_size : 1);
    if (buf == 0) {
        return 0;
    }

    if (size > 0) {
        memcpy(buf, data, size);
    }
#endif

    exercise_show_file(buf, buf_size);

    free(buf);
    return 0;
}
