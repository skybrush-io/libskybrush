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
 * \file test/traces/generate_trace.c
 * \brief Generates a golden trace of a Skybrush binary show file.
 *
 * The trace samples every component of the show (trajectory, yaw, lights,
 * RTH plan and the output of the show controller) at regular intervals and
 * records the static contents of the show (statistics, events, GCS light
 * control setup and metadata) in JSON format. Traces are stored together
 * with the specification of the file format and are used to check that the
 * behaviour of the library does not change unintentionally.
 *
 * Usage: generate_trace <input.skyb> <output.json>
 */

#include <math.h>
#include <stdio.h>
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
#include <skybrush/version.h>
#include <skybrush/yaw_control.h>

#include "trace_output.h"

/** Interval between consecutive samples of the trace */
#define SAMPLE_INTERVAL_MSEC 100

/** Extra time to sample after the end of the longest component of the show */
#define MARGIN_MSEC 1000

/** Light programs that do not end within this time are treated as endless */
#define MAX_LIGHT_PROGRAM_DURATION_MSEC 3600000

static uint8_t* show_data;
static size_t show_length;
static uint32_t num_samples;

static uint32_t sample_time_msec(uint32_t index)
{
    return index * SAMPLE_INTERVAL_MSEC;
}

/* ************************************************************************** */
/* Loading the components of the show                                        */
/* ************************************************************************** */

static uint32_t max_u32(uint32_t a, uint32_t b)
{
    return a > b ? a : b;
}

/** Returns the time when the light program ends, or UINT32_MAX if it never ends */
static uint32_t get_light_program_end_msec(sb_light_program_t* program)
{
    sb_light_player_t player;
    unsigned long t = 0, next = 0;
    uint32_t result = UINT32_MAX;

    CHECK(sb_light_player_init(&player, program));
    while (t <= MAX_LIGHT_PROGRAM_DURATION_MSEC) {
        if (sb_light_player_seek(&player, t, &next)) {
            result = t;
            break;
        }
        t = next > t ? next : t + 1;
    }
    sb_light_player_destroy(&player);

    return result;
}

/* ************************************************************************** */
/* Sections of the trace                                                     */
/* ************************************************************************** */

static void print_trajectory(sb_trajectory_t* trajectory)
{
    sb_trajectory_player_t player;
    sb_vector3_with_yaw_t vec;
    uint32_t i;

    CHECK(sb_trajectory_player_init(&player, trajectory));

    fprintf(out, "  \"trajectory\": {\n");
    fprintf(out, "    \"duration_ms\": %u,\n", (unsigned)sb_trajectory_get_total_duration_msec(trajectory));

    fprintf(out, "    \"position_mm\": [\n      ");
    for (i = 0; i < num_samples; i++) {
        CHECK(sb_trajectory_player_get_position_at(&player, sample_time_msec(i) / 1000.0f, &vec));
        print_vector3_with_yaw(vec);
        print_item_separator(i, num_samples);
    }
    fprintf(out, "],\n");

    fprintf(out, "    \"velocity_mm_s\": [\n      ");
    for (i = 0; i < num_samples; i++) {
        CHECK(sb_trajectory_player_get_velocity_at(&player, sample_time_msec(i) / 1000.0f, &vec));
        print_vector3_with_yaw(vec);
        print_item_separator(i, num_samples);
    }
    fprintf(out, "],\n");

    fprintf(out, "    \"acceleration_mm_s2\": [\n      ");
    for (i = 0; i < num_samples; i++) {
        CHECK(sb_trajectory_player_get_acceleration_at(&player, sample_time_msec(i) / 1000.0f, &vec));
        print_vector3_with_yaw(vec);
        print_item_separator(i, num_samples);
    }
    fprintf(out, "]\n");
    fprintf(out, "  },\n");

    sb_trajectory_player_destroy(&player);
}

static void print_stats(sb_trajectory_t* trajectory)
{
    sb_trajectory_stats_calculator_t calc;
    sb_trajectory_stats_t stats;

    /* Default settings of the calculator for coordinates in millimeters */
    CHECK(sb_trajectory_stats_init(&stats));
    CHECK(sb_trajectory_stats_calculator_init(&calc, 1000));
    CHECK(sb_trajectory_stats_calculator_run(&calc, trajectory, &stats));
    sb_trajectory_stats_calculator_destroy(&calc);

    fprintf(out, "  \"stats\": {\n");
    fprintf(out, "    \"duration_ms\": %u,\n", (unsigned)stats.duration_msec);
    fprintf(out, "    \"earliest_above_sec\": ");
    print_float(stats.earliest_above_sec);
    fprintf(out, ",\n    \"takeoff_time_sec\": ");
    print_float(stats.takeoff_time_sec);
    fprintf(out, ",\n    \"landing_time_sec\": ");
    print_float(stats.landing_time_sec);
    fprintf(out, ",\n    \"initial_position_mm\": ");
    print_vector3_with_yaw(stats.initial_pos);
    fprintf(out, ",\n    \"position_at_landing_mm\": ");
    print_vector3_with_yaw(stats.pos_at_landing_time);
    fprintf(out, ",\n    \"velocity_at_landing_mm_s\": ");
    print_vector3_with_yaw(stats.vel_at_landing_time);
    fprintf(out, ",\n    \"start_to_end_distance_xy_mm\": ");
    print_float(stats.start_to_end_distance_xy);
    fprintf(out, "\n  },\n");

    sb_trajectory_stats_destroy(&stats);
}

static void print_yaw_control(sb_yaw_control_t* ctrl)
{
    sb_yaw_player_t player;
    uint32_t duration_msec, i;
    float value;

    CHECK(sb_yaw_player_init(&player, ctrl));
    CHECK(sb_yaw_player_get_total_duration_msec(&player, &duration_msec));

    fprintf(out, "  \"yaw_control\": {\n");
    fprintf(out, "    \"duration_ms\": %u,\n", (unsigned)duration_msec);

    fprintf(out, "    \"yaw_deg\": [\n      ");
    for (i = 0; i < num_samples; i++) {
        CHECK(sb_yaw_player_get_yaw_at(&player, sample_time_msec(i) / 1000.0f, &value));
        print_float(value);
        print_item_separator(i, num_samples);
    }
    fprintf(out, "],\n");

    fprintf(out, "    \"yaw_rate_deg_s\": [\n      ");
    for (i = 0; i < num_samples; i++) {
        CHECK(sb_yaw_player_get_yaw_rate_at(&player, sample_time_msec(i) / 1000.0f, &value));
        print_float(value);
        print_item_separator(i, num_samples);
    }
    fprintf(out, "]\n");
    fprintf(out, "  },\n");

    sb_yaw_player_destroy(&player);
}

static void print_light_program(sb_light_program_t* program, uint32_t end_msec)
{
    sb_light_player_t player;
    uint32_t i;

    fprintf(out, "  \"light_program\": {\n");
    fprintf(out, "    \"ends_at_ms\": ");
    if (end_msec == UINT32_MAX) {
        fprintf(out, "null,\n");
    } else {
        fprintf(out, "%u,\n", (unsigned)end_msec);
    }

    CHECK(sb_light_player_init(&player, program));
    fprintf(out, "    \"color\": [\n      ");
    for (i = 0; i < num_samples; i++) {
        print_color(sb_light_player_get_color_at(&player, sample_time_msec(i)));
        print_item_separator(i, num_samples);
    }
    fprintf(out, "]\n");
    fprintf(out, "  },\n");

    sb_light_player_destroy(&player);
}

static void print_event(const sb_event_t* event)
{
    fprintf(out, "[%u, %d, %d, ", (unsigned)event->time_msec, (int)event->type, (int)event->subtype);
    print_hex(event->payload.as_buf, sizeof(event->payload.as_buf));
    fprintf(out, "]");
}

static void print_event_list(sb_event_list_t* events)
{
    size_t i, n = sb_event_list_size(events);

    fprintf(out, "  \"event_list\": {\n");
    fprintf(out, "    \"events\": [");
    for (i = 0; i < n; i++) {
        fprintf(out, i == 0 ? "\n      " : ",\n      ");
        print_event(sb_event_list_get_ptr_const(events, i));
    }
    fprintf(out, n > 0 ? "\n    ]\n" : "]\n");
    fprintf(out, "  },\n");
}

static void print_rth_plan(sb_rth_plan_t* plan)
{
    sb_rth_plan_entry_t entry;
    uint32_t i;

    fprintf(out, "  \"rth_plan\": {\n");
    fprintf(out, "    \"entry_fields\": [\"time_sec\", \"flags\", \"landing_target_x_mm\", \"landing_target_y_mm\", "
                 "\"arrival_altitude_mm\", \"duration_sec\", \"pre_delay_sec\", \"post_delay_sec\", \"pre_neck_mm\", "
                 "\"pre_neck_duration_sec\", \"max_acceleration_mm_s2\", \"landing_velocity_mm_s\", "
                 "\"landing_altitude_mm\"],\n");
    fprintf(out, "    \"entries\": [\n      ");
    for (i = 0; i < num_samples; i++) {
        CHECK(sb_rth_plan_evaluate_at(plan, sample_time_msec(i) / 1000.0f, &entry));
        fprintf(out, "[");
        print_float(entry.time_sec);
        fprintf(out, ", %d, ", (int)entry.flags);
        print_float(entry.landing_target.x);
        fprintf(out, ", ");
        print_float(entry.landing_target.y);
        fprintf(out, ", ");
        print_float(entry.arrival_altitude);
        fprintf(out, ", ");
        print_float(entry.duration_sec);
        fprintf(out, ", ");
        print_float(entry.pre_delay_sec);
        fprintf(out, ", ");
        print_float(entry.post_delay_sec);
        fprintf(out, ", ");
        print_float(entry.pre_neck);
        fprintf(out, ", ");
        print_float(entry.pre_neck_duration_sec);
        fprintf(out, ", ");
        print_float(entry.max_acceleration);
        fprintf(out, ", ");
        print_float(entry.landing_velocity);
        fprintf(out, ", ");
        print_float(entry.landing_altitude);
        fprintf(out, "]");
        print_item_separator(i, num_samples);
    }
    fprintf(out, "]\n");
    fprintf(out, "  },\n");
}

static void print_gcs_light_control_setup(const sb_gcs_light_control_setup_t* setup)
{
    size_t i, n = sb_color_palette_size(&setup->palette);

    fprintf(out, "  \"gcs_light_control_setup\": {\n");
    fprintf(out, "    \"coords\": [%d, %d],\n", setup->coords.x, setup->coords.y);
    fprintf(out, "    \"matrix_size\": [%d, %d],\n", setup->size.x, setup->size.y);
    fprintf(out, "    \"palette\": [");
    for (i = 0; i < n; i++) {
        sb_rgb_color_t color = sb_color_palette_get_color(&setup->palette, i);
        fprintf(out, "%s\"%02x%02x%02x\"", i > 0 ? ", " : "", color.red, color.green, color.blue);
    }
    fprintf(out, "]\n");
    fprintf(out, "  },\n");
}

static void print_metadata(const sb_show_metadata_t* metadata)
{
    uint8_t show_id[SB_SHOW_METADATA_SHOW_ID_LENGTH];
    uint16_t drone_index;

    CHECK(sb_show_metadata_get_show_id(metadata, show_id));
    CHECK(sb_show_metadata_get_drone_index(metadata, &drone_index));

    fprintf(out, "  \"metadata\": {\n");
    fprintf(out, "    \"show_id\": ");
    print_hex(show_id, sizeof(show_id));
    fprintf(out, ",\n    \"drone_index\": %u\n", (unsigned)drone_index);
    fprintf(out, "  },\n");
}

/**
 * Prints the output of the show controller, i.e. what the drone is commanded
 * to do, along with the events fired by the controller between consecutive
 * samples.
 */
static void print_show_controller(void)
{
    sb_screenplay_t screenplay;
    sb_show_controller_t ctrl;
    const sb_control_output_t* output;
    const sb_event_t* event;
    sb_vector3_t vec;
    sb_rgb_color_t color;
    float value;
    uint32_t i;
    int first_event = 1;

    CHECK(sb_screenplay_init(&screenplay));
    CHECK(sb_screenplay_update_from_binary_file_in_memory(&screenplay, show_data, show_length));
    CHECK(sb_show_controller_init(&ctrl, &screenplay));

    fprintf(out, "  \"show_controller\": {\n");
    fprintf(out, "    \"samples\": [\n      ");
    for (i = 0; i < num_samples; i++) {
        CHECK(sb_show_controller_update_time_msec(&ctrl, sample_time_msec(i)));
        output = sb_show_controller_get_current_output(&ctrl);

        fprintf(out, "{\"position_mm\": ");
        if (sb_control_output_get_position_if_set(output, &vec)) {
            print_vector3(vec);
        } else {
            fprintf(out, "null");
        }

        fprintf(out, ", \"velocity_mm_s\": ");
        if (sb_control_output_get_velocity_if_set(output, &vec)) {
            print_vector3(vec);
        } else {
            fprintf(out, "null");
        }

        fprintf(out, ", \"yaw_deg\": ");
        if (sb_control_output_get_yaw_if_set(output, &value)) {
            print_float(value);
        } else {
            fprintf(out, "null");
        }

        fprintf(out, ", \"yaw_rate_deg_s\": ");
        if (sb_control_output_get_yaw_rate_if_set(output, &value)) {
            print_float(value);
        } else {
            fprintf(out, "null");
        }

        fprintf(out, ", \"color\": ");
        if (sb_control_output_get_color_if_set(output, &color)) {
            print_color(color);
        } else {
            fprintf(out, "null");
        }

        fprintf(out, ", \"ended\": %s}", sb_show_controller_has_reached_end(&ctrl) ? "true" : "false");
        print_item_separator(i, num_samples);
    }
    fprintf(out, "],\n");

    /* Events fired by the controller; the controller is rewound first so all
     * the events are fired in order while the time moves forward */
    sb_show_controller_destroy(&ctrl);
    CHECK(sb_show_controller_init(&ctrl, &screenplay));
    fprintf(out, "    \"fired_events\": [");
    for (i = 0; i < num_samples; i++) {
        CHECK(sb_show_controller_update_time_msec(&ctrl, sample_time_msec(i)));
        while ((event = sb_show_controller_get_next_event(&ctrl)) != 0) {
            fprintf(out, first_event ? "\n      " : ",\n      ");
            fprintf(out, "[%u, ", (unsigned)sample_time_msec(i));
            print_event(event);
            fprintf(out, "]");
            first_event = 0;
        }
    }
    fprintf(out, first_event ? "]\n" : "\n    ]\n");
    fprintf(out, "  }\n");

    sb_show_controller_destroy(&ctrl);
    sb_screenplay_destroy(&screenplay);
}

/* ************************************************************************** */

int main(int argc, char* argv[])
{
    sb_trajectory_t* trajectory;
    sb_yaw_control_t* yaw_control;
    sb_light_program_t* light_program;
    sb_event_list_t* events;
    sb_rth_plan_t* rth_plan;
    sb_gcs_light_control_setup_t gcs_setup;
    sb_show_metadata_t metadata;
    int has_trajectory, has_yaw_control, has_light_program, has_events, has_rth_plan;
    int has_gcs_setup, has_metadata;
    uint32_t end_msec = 0, light_end_msec = UINT32_MAX, yaw_duration_msec;
    sb_yaw_player_t yaw_player;
    const char* fixture;

    if (argc != 3) {
        fprintf(stderr, "Usage: %s <input.skyb> <output.json>\n", argv[0]);
        return 2;
    }

    read_show_file(argv[1], &show_data, &show_length);
    fixture = base_name(argv[1]);

    /* Load all the components of the show */
    trajectory = sb_trajectory_new();
    yaw_control = sb_yaw_control_new();
    light_program = sb_light_program_new();
    events = sb_event_list_new(0);
    rth_plan = sb_rth_plan_new();
    if (!trajectory || !yaw_control || !light_program || !events || !rth_plan) {
        fprintf(stderr, "out of memory\n");
        return 1;
    }
    CHECK(sb_gcs_light_control_setup_init(&gcs_setup));
    CHECK(sb_show_metadata_init(&metadata));

    has_trajectory = is_present(sb_trajectory_update_from_binary_file_in_memory(trajectory, show_data, show_length), "trajectory");
    has_yaw_control = is_present(sb_yaw_control_update_from_binary_file_in_memory(yaw_control, show_data, show_length), "yaw control");
    has_light_program = is_present(sb_light_program_update_from_binary_file_in_memory(light_program, show_data, show_length), "light program");
    has_events = is_present(sb_event_list_update_from_binary_file_in_memory(events, show_data, show_length), "event list");
    has_rth_plan = is_present(sb_rth_plan_update_from_binary_file_in_memory(rth_plan, show_data, show_length), "RTH plan");
    has_gcs_setup = is_present(sb_gcs_light_control_setup_update_from_binary_file_in_memory(&gcs_setup, show_data, show_length), "GCS light control setup");
    has_metadata = is_present(sb_show_metadata_update_from_binary_file_in_memory(&metadata, show_data, show_length), "metadata");

    /* The trace covers the longest component of the show and some margin */
    if (has_trajectory) {
        end_msec = max_u32(end_msec, sb_trajectory_get_total_duration_msec(trajectory));
    }
    if (has_yaw_control) {
        CHECK(sb_yaw_player_init(&yaw_player, yaw_control));
        CHECK(sb_yaw_player_get_total_duration_msec(&yaw_player, &yaw_duration_msec));
        sb_yaw_player_destroy(&yaw_player);
        end_msec = max_u32(end_msec, yaw_duration_msec);
    }
    if (has_light_program) {
        light_end_msec = get_light_program_end_msec(light_program);
        if (light_end_msec != UINT32_MAX) {
            end_msec = max_u32(end_msec, light_end_msec);
        }
    }
    if (has_events && sb_event_list_size(events) > 0) {
        end_msec = max_u32(end_msec, sb_event_list_get_ptr_const(events, sb_event_list_size(events) - 1)->time_msec);
    }
    end_msec += MARGIN_MSEC;
    num_samples = (end_msec + SAMPLE_INTERVAL_MSEC - 1) / SAMPLE_INTERVAL_MSEC + 1;

    out = fopen(argv[2], "w");
    if (out == 0) {
        perror(argv[2]);
        return 1;
    }

    fprintf(out, "{\n");
    fprintf(out, "  \"$schema\": \"./trace.schema.json\",\n");
    fprintf(out, "  \"version\": 1,\n");
    fprintf(out, "  \"fixture\": \"%s\",\n", fixture);
    fprintf(out, "  \"generator\": \"libskybrush %d.%d.%d\",\n",
        SKYBRUSH_VERSION_MAJOR, SKYBRUSH_VERSION_MINOR, SKYBRUSH_VERSION_PATCH);
    fprintf(out, "  \"sample_interval_ms\": %d,\n", SAMPLE_INTERVAL_MSEC);
    fprintf(out, "  \"num_samples\": %u,\n", (unsigned)num_samples);

    if (has_trajectory) {
        print_trajectory(trajectory);
        print_stats(trajectory);
    }
    if (has_yaw_control) {
        print_yaw_control(yaw_control);
    }
    if (has_light_program) {
        print_light_program(light_program, light_end_msec);
    }
    if (has_events) {
        print_event_list(events);
    }
    if (has_rth_plan) {
        print_rth_plan(rth_plan);
    }
    if (has_gcs_setup) {
        print_gcs_light_control_setup(&gcs_setup);
    }
    if (has_metadata) {
        print_metadata(&metadata);
    }
    print_show_controller();

    fprintf(out, "}\n");
    fclose(out);

    sb_show_metadata_destroy(&metadata);
    sb_gcs_light_control_setup_destroy(&gcs_setup);
    SB_XDECREF(rth_plan);
    SB_XDECREF(events);
    SB_XDECREF(light_program);
    SB_XDECREF(yaw_control);
    SB_XDECREF(trajectory);
    free(show_data);

    return 0;
}
