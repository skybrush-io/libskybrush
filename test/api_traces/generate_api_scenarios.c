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
 * \file test/api_traces/generate_api_scenarios.c
 * \brief Generates a golden trace of the API-level behaviour of the library in
 * scenarios that do not start from a show file.
 *
 * The scenarios cover the trajectory builder, the conversion of RTH plan
 * entries to trajectories, time axes, screenplays played by the show
 * controller, and the public motion and color helpers. The inputs of each
 * scenario are recorded next to its results; trajectories and light programs
 * are recorded as the hex-encoded bytes of their blocks.
 *
 * Usage: generate_api_scenarios <output.json>
 */

#include <math.h>
#include <string.h>

#include <skybrush/colors.h>
#include <skybrush/control.h>
#include <skybrush/events.h>
#include <skybrush/formats/binary.h>
#include <skybrush/lights.h>
#include <skybrush/motion.h>
#include <skybrush/refcount.h>
#include <skybrush/rth_plan.h>
#include <skybrush/screenplay.h>
#include <skybrush/stats.h>
#include <skybrush/time_axis.h>
#include <skybrush/trajectory.h>
#include <skybrush/version.h>
#include <skybrush/yaw_control.h>

#include "../../src/lights/commands.h"
#include "../traces/trace_output.h"

/** Interval between consecutive samples of trajectories and the show controller */
#define SAMPLE_INTERVAL_MSEC 100

/** Extra time to sample after the end of a trajectory or a screenplay */
#define MARGIN_MSEC 1000

/** Relative times within each segment where built trajectories are sampled */
static const float SEGMENT_SAMPLES[] = { 0, 0.25f, 0.5f, 0.75f, 1 };

/* ************************************************************************** */
/* A minimal JSON writer                                                     */
/* ************************************************************************** */

#define JSON_MAX_DEPTH 32

/** Items of containers up to this depth are written on separate lines */
#define JSON_BREAK_DEPTH 3

static int j_depth;
static int j_count[JSON_MAX_DEPTH];
static int j_after_key;

static void j_indent(void)
{
    fputc('\n', out);
    for (int i = 0; i < j_depth; i++) {
        fputs("  ", out);
    }
}

/** Writes the separator before the next item of the current container */
static void j_before_item(void)
{
    if (j_after_key) {
        j_after_key = 0;
        return;
    }
    if (j_count[j_depth]++ > 0) {
        fputc(',', out);
        if (j_depth > JSON_BREAK_DEPTH) {
            fputc(' ', out);
        }
    }
    if (j_depth > 0 && j_depth <= JSON_BREAK_DEPTH) {
        j_indent();
    }
}

static void j_open(char bracket)
{
    j_before_item();
    fputc(bracket, out);
    j_depth++;
    j_count[j_depth] = 0;
}

static void j_close(char bracket)
{
    int had_items = j_count[j_depth] > 0;
    j_depth--;
    if (had_items && j_depth + 1 <= JSON_BREAK_DEPTH) {
        j_indent();
    }
    fputc(bracket, out);
}

#define j_begin_object() j_open('{')
#define j_end_object() j_close('}')
#define j_begin_array() j_open('[')
#define j_end_array() j_close(']')

static void j_key(const char* key)
{
    j_before_item();
    fprintf(out, "\"%s\": ", key);
    j_after_key = 1;
}

static void j_float(float value)
{
    j_before_item();
    print_float(value);
}

static void j_int(long value)
{
    j_before_item();
    fprintf(out, "%ld", value);
}

static void j_string(const char* value)
{
    j_before_item();
    fprintf(out, "\"%s\"", value);
}

static void j_bool(int value)
{
    j_before_item();
    fputs(value ? "true" : "false", out);
}

static void j_null(void)
{
    j_before_item();
    fputs("null", out);
}

static void j_error(sb_error_t error)
{
    j_before_item();
    print_error(error);
}

static void j_hex(const uint8_t* bytes, size_t length)
{
    j_before_item();
    print_hex(bytes, length);
}

static void j_vector3(sb_vector3_t vec)
{
    j_begin_array();
    j_float(vec.x);
    j_float(vec.y);
    j_float(vec.z);
    j_end_array();
}

/** Writes the X, Y and Z coordinates of a vector, ignoring its yaw */
static void j_vector3_xyz(sb_vector3_with_yaw_t vec)
{
    j_begin_array();
    j_float(vec.x);
    j_float(vec.y);
    j_float(vec.z);
    j_end_array();
}

static void j_vector3_with_yaw(sb_vector3_with_yaw_t vec)
{
    j_begin_array();
    j_float(vec.x);
    j_float(vec.y);
    j_float(vec.z);
    j_float(vec.yaw);
    j_end_array();
}

static void j_color(sb_rgb_color_t color)
{
    j_begin_array();
    j_int(color.red);
    j_int(color.green);
    j_int(color.blue);
    j_end_array();
}

static void j_rgbw_color(sb_rgbw_color_t color)
{
    j_begin_array();
    j_int(color.red);
    j_int(color.green);
    j_int(color.blue);
    j_int(color.white);
    j_end_array();
}

#define j_kfloat(key, value) (j_key(key), j_float(value))
#define j_kint(key, value) (j_key(key), j_int(value))
#define j_kstring(key, value) (j_key(key), j_string(value))
#define j_kerror(key, value) (j_key(key), j_error(value))

/* ************************************************************************** */
/* Helpers                                                                   */
/* ************************************************************************** */

static void* checked_malloc(size_t size)
{
    void* result = malloc(size > 0 ? size : 1);
    if (result == 0) {
        fprintf(stderr, "out of memory\n");
        exit(1);
    }
    return result;
}

/** Records the encoded bytes of a trajectory */
static void j_trajectory_hex(sb_trajectory_t* trajectory)
{
    j_hex(SB_BUFFER(trajectory->buffer), sb_buffer_size(&trajectory->buffer));
}

/**
 * Records the segments of a trajectory, sampling each one at a few relative
 * times. Positions are recorded in the units of the scale of the trajectory
 * as the builder quantizes them to those units.
 */
static void j_trajectory_segments(sb_trajectory_t* trajectory)
{
    /* One player iterates over the segments, the other one samples them;
     * sampling moves the player and changes its current segment */
    sb_trajectory_player_t player, sampler;
    const sb_trajectory_segment_t* segment;
    sb_vector3_with_yaw_t pos;
    float scale = trajectory->scale > 0 ? trajectory->scale : 1;
    size_t i;

    CHECK(sb_trajectory_player_init(&player, trajectory));
    CHECK(sb_trajectory_player_init(&sampler, trajectory));
    j_kint("scale", (long)trajectory->scale);
    j_kint("duration_ms", (long)sb_trajectory_get_total_duration_msec(trajectory));
    j_key("segments");
    j_begin_array();
    while (sb_trajectory_player_has_more_segments(&player)) {
        segment = sb_trajectory_player_get_current_segment(&player);
        j_begin_object();
        j_kint("start_ms", (long)segment->start_time_msec);
        j_kint("duration_ms", (long)segment->duration_msec);
        j_key("position_units");
        j_begin_array();
        for (i = 0; i < sizeof(SEGMENT_SAMPLES) / sizeof(SEGMENT_SAMPLES[0]); i++) {
            CHECK(sb_trajectory_player_get_position_at(&sampler,
                segment->start_time_sec + SEGMENT_SAMPLES[i] * segment->duration_sec, &pos));
            j_begin_array();
            j_float(pos.x / scale);
            j_float(pos.y / scale);
            j_float(pos.z / scale);
            j_end_array();
        }
        j_end_array();
        j_end_object();
        CHECK(sb_trajectory_player_build_next_segment(&player));
    }
    j_end_array();
    sb_trajectory_player_destroy(&sampler);
    sb_trajectory_player_destroy(&player);
}

/** Records the position and the velocity along a trajectory at regular intervals */
static void j_trajectory_samples(sb_trajectory_t* trajectory, uint32_t from_msec, uint32_t to_msec)
{
    sb_trajectory_player_t player;
    sb_vector3_with_yaw_t vec;
    uint32_t t;

    from_msec -= from_msec % SAMPLE_INTERVAL_MSEC;

    CHECK(sb_trajectory_player_init(&player, trajectory));
    j_kint("samples_from_ms", (long)from_msec);
    j_key("position_mm");
    j_begin_array();
    for (t = from_msec; t <= to_msec; t += SAMPLE_INTERVAL_MSEC) {
        CHECK(sb_trajectory_player_get_position_at(&player, t / 1000.0f, &vec));
        j_vector3_xyz(vec);
    }
    j_end_array();
    j_key("velocity_mm_s");
    j_begin_array();
    for (t = from_msec; t <= to_msec; t += SAMPLE_INTERVAL_MSEC) {
        CHECK(sb_trajectory_player_get_velocity_at(&player, t / 1000.0f, &vec));
        j_vector3_xyz(vec);
    }
    j_end_array();
    sb_trajectory_player_destroy(&player);
}

/* ************************************************************************** */
/* Trajectory builder                                                        */
/* ************************************************************************** */

typedef enum {
    OP_END = 0,
    OP_SET_START,
    OP_LINE,
    OP_BEZIER,
    OP_HOLD,
    OP_MOVE_TO_IN_TIME,
} builder_op_type_t;

typedef struct {
    builder_op_type_t type;
    sb_vector3_with_yaw_t target;
    sb_vector3_with_yaw_t control1;
    sb_vector3_with_yaw_t control2;
    uint32_t duration_msec;
    float max_acceleration;
} builder_op_t;

typedef struct {
    const char* name;
    uint8_t scale;
    builder_op_t ops[8];
} builder_scenario_t;

#define P(x, y, z) \
    {              \
        x, y, z, 0 \
    }

static const builder_scenario_t BUILDER_SCENARIOS[] = {
    { "lines_and_hold", 1, {
                               { OP_SET_START, P(0, 0, 0) },
                               { OP_LINE, P(1000, 0, 0), P(0, 0, 0), P(0, 0, 0), 2000 },
                               { OP_LINE, P(1000, 1000, 500), P(0, 0, 0), P(0, 0, 0), 1500 },
                               { OP_HOLD, P(0, 0, 0), P(0, 0, 0), P(0, 0, 0), 1000 },
                           } },
    { "without_start_position", 1, {
                                       { OP_LINE, P(500, 0, 0), P(0, 0, 0), P(0, 0, 0), 1000 },
                                   } },
    { "cubic_bezier", 1, {
                             { OP_SET_START, P(0, 0, 1000) },
                             { OP_BEZIER, P(3000, 0, 1000), P(1000, 1000, 1000), P(2000, -1000, 1000), 3000 },
                             { OP_BEZIER, P(3000, 0, 2000), P(3000, 0, 1000), P(3000, 0, 2000), 1000 },
                         } },
    { "long_hold_is_split", 1, {
                                   { OP_SET_START, P(10, 20, 30) },
                                   { OP_HOLD, P(0, 0, 0), P(0, 0, 0), P(0, 0, 0), 200000 },
                               } },
    { "zero_duration_hold", 1, {
                                   { OP_SET_START, P(10, 20, 30) },
                                   { OP_HOLD, P(0, 0, 0), P(0, 0, 0), P(0, 0, 0), 0 },
                                   { OP_HOLD, P(0, 0, 0), P(0, 0, 0), P(0, 0, 0), 1 },
                               } },
    { "move_to_in_time", 1, {
                                { OP_SET_START, P(0, 0, 0) },
                                { OP_MOVE_TO_IN_TIME, P(10000, 0, 0), P(0, 0, 0), P(0, 0, 0), 10000, 1000 },
                                { OP_MOVE_TO_IN_TIME, P(10000, 5000, 3000), P(0, 0, 0), P(0, 0, 0), 8000, 4000 },
                            } },
    { "move_to_in_time_short_cruise", 1, {
                                             { OP_SET_START, P(0, 0, 0) },
                                             { OP_MOVE_TO_IN_TIME, P(2187.5f, 0, 0), P(0, 0, 0), P(0, 0, 0), 3000, 1000 },
                                         } },
    { "move_to_in_time_extreme_acceleration", 1, {
                                                     { OP_SET_START, P(0, 0, 0) },
                                                     { OP_MOVE_TO_IN_TIME, P(100, 0, 0), P(0, 0, 0), P(0, 0, 0), 5000, 65535 },
                                                     { OP_MOVE_TO_IN_TIME, P(200, 0, 0), P(0, 0, 0), P(0, 0, 0), 5000, INFINITY },
                                                 } },
    { "move_to_in_time_short_durations", 1, {
                                                { OP_SET_START, P(0, 0, 0) },
                                                { OP_MOVE_TO_IN_TIME, P(100, 0, 0), P(0, 0, 0), P(0, 0, 0), 0, 1000 },
                                                { OP_MOVE_TO_IN_TIME, P(100, 0, 0), P(0, 0, 0), P(0, 0, 0), 1, 1000 },
                                                { OP_MOVE_TO_IN_TIME, P(200, 0, 0), P(0, 0, 0), P(0, 0, 0), 2, 1000 },
                                                { OP_MOVE_TO_IN_TIME, P(300, 0, 0), P(0, 0, 0), P(0, 0, 0), 3, 1000 },
                                            } },
    { "large_coordinates_with_scale", 10, {
                                              { OP_SET_START, P(-300000, 250000, 0) },
                                              { OP_LINE, P(300000, -250000, 100000), P(0, 0, 0), P(0, 0, 0), 60000 },
                                          } },
    { "coordinate_out_of_range", 1, {
                                        { OP_SET_START, P(0, 0, 0) },
                                        { OP_LINE, P(40000, 0, 0), P(0, 0, 0), P(0, 0, 0), 1000 },
                                        { OP_LINE, P(1000, 0, 0), P(0, 0, 0), P(0, 0, 0), 1000 },
                                    } },
    { "nan_coordinate", 1, {
                               { OP_SET_START, P(0, 0, 0) },
                               { OP_LINE, P(NAN, 0, 0), P(0, 0, 0), P(0, 0, 0), 1000 },
                               { OP_SET_START, P(0, NAN, 0) },
                           } },
    { "invalid_scale_zero", 0, { { OP_LINE, P(0, 0, 0), P(0, 0, 0), P(0, 0, 0), 1000 } } },
    { "invalid_scale_too_large", 128, { { OP_LINE, P(0, 0, 0), P(0, 0, 0), P(0, 0, 0), 1000 } } },
    { "largest_scale", 127, { { OP_LINE, P(127000, 0, 0), P(0, 0, 0), P(0, 0, 0), 1000 } } },
    { "no_segments", 1, { { OP_END } } },
    { "long_line_is_split", 1, {
                                   { OP_SET_START, P(0, 0, 0) },
                                   { OP_LINE, P(1000, 30000, 500), P(0, 0, 0), P(0, 0, 0), 120001 },
                               } },
};

static void j_builder_op(const builder_op_t* op, sb_trajectory_builder_t* builder)
{
    sb_error_t retval = SB_SUCCESS;

    j_begin_object();
    switch (op->type) {
    case OP_SET_START:
        j_kstring("op", "set_start_position");
        j_key("position_mm");
        j_vector3_with_yaw(op->target);
        retval = sb_trajectory_builder_set_start_position(builder, op->target);
        break;
    case OP_LINE:
        j_kstring("op", "append_line");
        j_key("target_mm");
        j_vector3_with_yaw(op->target);
        j_kint("duration_ms", (long)op->duration_msec);
        retval = sb_trajectory_builder_append_line(builder, op->target, op->duration_msec);
        break;
    case OP_BEZIER:
        j_kstring("op", "append_cubic_bezier");
        j_key("control1_mm");
        j_vector3_with_yaw(op->control1);
        j_key("control2_mm");
        j_vector3_with_yaw(op->control2);
        j_key("target_mm");
        j_vector3_with_yaw(op->target);
        j_kint("duration_ms", (long)op->duration_msec);
        retval = sb_trajectory_builder_append_cubic_bezier(builder, op->control1, op->control2, op->target, op->duration_msec);
        break;
    case OP_HOLD:
        j_kstring("op", "hold_position_for");
        j_kint("duration_ms", (long)op->duration_msec);
        retval = sb_trajectory_builder_hold_position_for(builder, op->duration_msec);
        break;
    case OP_MOVE_TO_IN_TIME:
        j_kstring("op", "move_to_in_time");
        j_key("target_mm");
        j_vector3_with_yaw(op->target);
        j_kint("duration_ms", (long)op->duration_msec);
        j_kfloat("max_acceleration_mm_s2", op->max_acceleration);
        retval = sb_trajectory_builder_move_to_in_time(builder, op->target, op->duration_msec, op->max_acceleration);
        break;
    default:
        break;
    }
    j_kerror("result", retval);
    j_end_object();
}

static void j_builder_scenarios(void)
{
    sb_trajectory_builder_t builder;
    sb_trajectory_t trajectory;
    const builder_scenario_t* scenario;
    sb_error_t retval;
    size_t i, j;

    j_key("builder");
    j_begin_array();
    for (i = 0; i < sizeof(BUILDER_SCENARIOS) / sizeof(BUILDER_SCENARIOS[0]); i++) {
        scenario = &BUILDER_SCENARIOS[i];
        j_begin_object();
        j_kstring("scenario", scenario->name);
        j_kint("scale", scenario->scale);

        retval = sb_trajectory_builder_init(&builder, scenario->scale, 0);
        j_kerror("init_result", retval);
        if (retval == SB_SUCCESS) {
            j_key("ops");
            j_begin_array();
            for (j = 0; j < sizeof(scenario->ops) / sizeof(scenario->ops[0]) && scenario->ops[j].type != OP_END; j++) {
                j_builder_op(&scenario->ops[j], &builder);
            }
            j_end_array();

            CHECK(sb_trajectory_init(&trajectory));
            CHECK(sb_trajectory_update_from_builder(&trajectory, &builder));
            j_key("trajectory");
            j_begin_object();
            j_key("is_empty");
            j_bool(sb_trajectory_is_empty(&trajectory));
            {
                sb_vector3_with_yaw_t start_position;
                CHECK(sb_trajectory_get_start_position(&trajectory, &start_position));
                j_key("start_position_mm");
                j_vector3_xyz(start_position);
            }
            j_trajectory_segments(&trajectory);
            j_end_object();
            SB_DECREF_STATIC(&trajectory);
            sb_trajectory_builder_destroy(&builder);
        }
        j_end_object();
    }
    j_end_array();
}

/* ************************************************************************** */
/* RTH plan entries                                                          */
/* ************************************************************************** */

typedef struct {
    const char* name;
    float time_sec;
    int flags;
    float target_x, target_y, arrival_altitude;
    float duration_sec, pre_delay_sec, post_delay_sec;
    float pre_neck, pre_neck_duration_sec;
    float max_acceleration, landing_velocity, landing_altitude;
    float start_z; /* zero means the default start altitude */
} rth_scenario_t;

static const rth_scenario_t RTH_SCENARIOS[] = {
    { "land_in_place", 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, INFINITY, NAN, 0 },
    { "neck_only", 10, SB_RTH_PLAN_ENTRY_HAS_NECK, 0, 0, 0, 0, 0, 0, 2000, 4, INFINITY, NAN, 0 },
    { "go_to_xy", 10, SB_RTH_PLAN_ENTRY_HAS_TARGET_XY, 20000, -10000, 0, 15, 0, 0, 0, 0, INFINITY, NAN, 0 },
    { "go_to_z", 10, SB_RTH_PLAN_ENTRY_HAS_TARGET_Z, 0, 0, 8000, 5, 0, 0, 0, 0, INFINITY, NAN, 0 },
    { "everything", 10.5f,
        SB_RTH_PLAN_ENTRY_HAS_NECK | SB_RTH_PLAN_ENTRY_HAS_TARGET_XY | SB_RTH_PLAN_ENTRY_HAS_TARGET_Z | SB_RTH_PLAN_ENTRY_HAS_PRE_DELAY | SB_RTH_PLAN_ENTRY_HAS_POST_DELAY,
        15000, 5000, 6000, 12, 2, 3, 1500, 3, 2000, NAN, 0 },
    { "landing_descent", 10, SB_RTH_PLAN_ENTRY_HAS_TARGET_XY | SB_RTH_PLAN_ENTRY_HAS_LANDING_ALTITUDE, 5000, 5000, 0, 8, 0, 0, 0, 0, INFINITY, 1000, 500 },
    { "zero_durations", 10, SB_RTH_PLAN_ENTRY_HAS_NECK | SB_RTH_PLAN_ENTRY_HAS_TARGET_XY, 1000, 0, 0, 0, 0, 0, 500, 0, INFINITY, NAN, 0 },
    { "negative_start_time", -3, SB_RTH_PLAN_ENTRY_HAS_TARGET_XY, 1000, 0, 0, 5, 0, 0, 0, 0, INFINITY, NAN, 0 },
    { "limited_acceleration", 10, SB_RTH_PLAN_ENTRY_HAS_TARGET_XY, 30000, 0, 0, 20, 0, 0, 0, 0, 500, NAN, 0 },
    { "too_long_pre_delay", 10, SB_RTH_PLAN_ENTRY_HAS_PRE_DELAY, 0, 0, 0, 0, 5000000, 0, 0, 0, INFINITY, NAN, 0 },
    /* The neck raises the altitude beyond what a scale of 1 can represent */
    { "neck_needs_larger_scale", 10, SB_RTH_PLAN_ENTRY_HAS_NECK, 0, 0, 0, 0, 0, 0, 5001, 2, INFINITY, NAN, 0, 30001 },
    { "fractional_start_time", 0.5f, SB_RTH_PLAN_ENTRY_HAS_TARGET_XY, 3000, 0, 0, 2, 0, 0, 0, 0, INFINITY, NAN, 0 },
};

static void j_rth_scenarios(void)
{
    sb_vector3_t start = { 1000, 2000, 10000 };
    const rth_scenario_t* scenario;
    sb_rth_plan_entry_t entry;
    sb_trajectory_t trajectory;
    sb_error_t retval;
    size_t i;

    j_key("rth_conversions");
    j_begin_array();
    for (i = 0; i < sizeof(RTH_SCENARIOS) / sizeof(RTH_SCENARIOS[0]); i++) {
        scenario = &RTH_SCENARIOS[i];
        start.z = scenario->start_z != 0 ? scenario->start_z : 10000;
        sb_rth_plan_entry_clear(&entry, 0, scenario->time_sec);
        entry.flags = scenario->flags;
        entry.landing_target.x = scenario->target_x;
        entry.landing_target.y = scenario->target_y;
        entry.arrival_altitude = scenario->arrival_altitude;
        entry.duration_sec = scenario->duration_sec;
        entry.pre_delay_sec = scenario->pre_delay_sec;
        entry.post_delay_sec = scenario->post_delay_sec;
        entry.pre_neck = scenario->pre_neck;
        entry.pre_neck_duration_sec = scenario->pre_neck_duration_sec;
        entry.max_acceleration = scenario->max_acceleration;
        entry.landing_velocity = scenario->landing_velocity;
        entry.landing_altitude = scenario->landing_altitude;

        j_begin_object();
        j_kstring("scenario", scenario->name);
        j_key("start_mm");
        j_vector3(start);
        j_key("entry");
        j_begin_object();
        j_kfloat("time_sec", entry.time_sec);
        j_kint("flags", (long)entry.flags);
        j_key("landing_target_mm");
        j_begin_array();
        j_float(entry.landing_target.x);
        j_float(entry.landing_target.y);
        j_end_array();
        j_kfloat("arrival_altitude_mm", entry.arrival_altitude);
        j_kfloat("duration_sec", entry.duration_sec);
        j_kfloat("pre_delay_sec", entry.pre_delay_sec);
        j_kfloat("post_delay_sec", entry.post_delay_sec);
        j_kfloat("pre_neck_mm", entry.pre_neck);
        j_kfloat("pre_neck_duration_sec", entry.pre_neck_duration_sec);
        j_kfloat("max_acceleration_mm_s2", entry.max_acceleration);
        j_kfloat("landing_velocity_mm_s", entry.landing_velocity);
        j_kfloat("landing_altitude_mm", entry.landing_altitude);
        j_end_object();

        CHECK(sb_trajectory_init(&trajectory));
        retval = sb_trajectory_update_from_rth_plan_entry(&trajectory, &entry, start);
        j_kerror("result", retval);
        if (retval == SB_SUCCESS) {
            j_kint("duration_ms", (long)sb_trajectory_get_total_duration_msec(&trajectory));
            j_trajectory_samples(&trajectory,
                entry.time_sec > 0 ? (uint32_t)(entry.time_sec * 1000) : 0,
                sb_trajectory_get_total_duration_msec(&trajectory) + MARGIN_MSEC);
        }
        j_end_object();
        SB_DECREF_STATIC(&trajectory);
    }
    j_end_array();

    /* An empty RTH plan */
    {
        sb_rth_plan_t plan;

        CHECK(sb_rth_plan_init(&plan));
        j_key("empty_rth_plan");
        j_begin_object();
        j_key("is_empty");
        j_bool(sb_rth_plan_is_empty(&plan));
        j_kint("num_entries", (long)sb_rth_plan_get_num_entries(&plan));
        j_kint("num_points", (long)sb_rth_plan_get_num_points(&plan));
        j_kerror("evaluate_at_result", sb_rth_plan_evaluate_at(&plan, 5, &entry));
        j_kfloat("default_landing_altitude_mm", sb_rth_plan_get_default_landing_altitude(&plan));
        j_end_object();
        SB_DECREF_STATIC(&plan);
    }
}

/* ************************************************************************** */
/* Time axes                                                                 */
/* ************************************************************************** */

typedef struct {
    uint32_t duration_msec; /* ignored if warped_duration_sec is not NaN */
    float warped_duration_sec;
    float initial_rate;
    float final_rate;
} time_segment_spec_t;

typedef struct {
    const char* name;
    int32_t origin_msec;
    time_segment_spec_t segments[4];
    size_t num_segments;
} time_axis_scenario_t;

#define REALTIME(ms)  \
    {                 \
        ms, NAN, 1, 1 \
    }

static const time_axis_scenario_t TIME_AXIS_SCENARIOS[] = {
    { "empty", 0, { REALTIME(0) }, 0 },
    { "realtime", 0, { REALTIME(10000) }, 1 },
    { "double_speed", 0, { { 5000, NAN, 2, 2 } }, 1 },
    { "spinup_realtime_slowdown", 0, { { 2000, NAN, 0, 1 }, REALTIME(3000), { 2000, NAN, 1, 0 } }, 3 },
    { "frozen_in_the_middle", 0, { REALTIME(2000), { 3000, NAN, 0, 0 }, REALTIME(2000) }, 3 },
    { "ramp_between_rates", 0, { { 4000, NAN, 0.5f, 2 } }, 1 },
    { "warped_duration", 0, { { 0, 3, 1, 0.5f } }, 1 },
    { "positive_origin", 1500, { { 2000, NAN, 0, 1 }, REALTIME(3000) }, 2 },
    { "negative_origin", -1000, { { 2000, NAN, 0, 1 }, REALTIME(3000) }, 2 },
    { "nan_rate", 0, { REALTIME(1000), { 1000, NAN, NAN, 1 } }, 2 },
    { "negative_rate", 0, { REALTIME(1000), { 1000, NAN, -1, 1 } }, 2 },
};

/**
 * Records the segments and the duration of a time axis, and its mapping
 * between wall clock time and warped time, from before its origin to after
 * its end
 */
static void j_time_axis_mapping(const sb_time_axis_t* axis)
{
    const sb_time_segment_t* segment;
    int32_t origin_msec = sb_time_axis_get_origin_msec(axis), t, end_msec;
    float warped, rate, total_warped = sb_time_axis_get_total_warped_duration_sec(axis);
    uint32_t duration_msec = sb_time_axis_get_total_duration_msec(axis);
    size_t j;

    j_kint("num_segments", (long)sb_time_axis_num_segments(axis));
    j_key("resulting_segments");
    j_begin_array();
    for (j = 0; j < sb_time_axis_num_segments(axis); j++) {
        segment = sb_time_axis_get_segment(axis, j);
        j_begin_array();
        j_int((long)segment->duration_msec);
        j_float(segment->initial_rate);
        j_float(segment->final_rate);
        j_end_array();
    }
    j_end_array();
    j_kint("total_duration_ms", duration_msec == UINT32_MAX ? -1 : (long)duration_msec);
    j_kfloat("total_duration_sec", sb_time_axis_get_total_duration_sec(axis));
    j_kfloat("total_warped_duration_sec", total_warped);

    /* Infinite axes are sampled for a while after their origin only */
    end_msec = origin_msec + (duration_msec < 60000 ? (int32_t)duration_msec : 60000) + MARGIN_MSEC;
    j_kint("map_from_ms", (long)(origin_msec - MARGIN_MSEC));
    j_key("map_sec");
    j_begin_array();
    for (t = origin_msec - MARGIN_MSEC; t <= end_msec; t += SAMPLE_INTERVAL_MSEC) {
        j_float(sb_time_axis_map(axis, t));
    }
    j_end_array();
    j_key("map_rate");
    j_begin_array();
    for (t = origin_msec - MARGIN_MSEC; t <= end_msec; t += SAMPLE_INTERVAL_MSEC) {
        sb_time_axis_map_ex(axis, t, &rate);
        j_float(rate);
    }
    j_end_array();

    /* Warped time -> wall clock time */
    j_key("reverse_map");
    j_begin_array();
    for (j = 0; j <= 20; j++) {
        warped = (isfinite(total_warped) && total_warped > 0 ? total_warped : 10) * (j / 20.0f) * 1.2f - 1;
        j_begin_array();
        j_float(warped);
        j_int((long)sb_time_axis_reverse_map(axis, warped));
        j_end_array();
    }
    j_end_array();
}

static void j_time_axis_scenarios(void)
{
    const time_axis_scenario_t* scenario;
    const time_segment_spec_t* spec;
    sb_time_segment_t segment;
    sb_time_axis_t axis;
    size_t i, j;

    j_key("time_axes");
    j_begin_array();
    for (i = 0; i < sizeof(TIME_AXIS_SCENARIOS) / sizeof(TIME_AXIS_SCENARIOS[0]); i++) {
        scenario = &TIME_AXIS_SCENARIOS[i];
        CHECK(sb_time_axis_init(&axis));

        j_begin_object();
        j_kstring("scenario", scenario->name);
        j_kint("origin_ms", (long)scenario->origin_msec);
        /* Negative origins are rejected */
        j_kerror("set_origin_result", sb_time_axis_set_origin_msec(&axis, scenario->origin_msec));
        j_kint("resulting_origin_ms", (long)sb_time_axis_get_origin_msec(&axis));
        j_key("segments");
        j_begin_array();
        for (j = 0; j < scenario->num_segments; j++) {
            spec = &scenario->segments[j];
            j_begin_object();
            if (isnan(spec->warped_duration_sec)) {
                segment = sb_time_segment_make(spec->duration_msec, spec->initial_rate, spec->final_rate);
                j_kint("duration_ms", (long)spec->duration_msec);
            } else {
                segment = sb_time_segment_make_warped(spec->warped_duration_sec, spec->initial_rate, spec->final_rate);
                j_kfloat("warped_duration_sec", spec->warped_duration_sec);
            }
            j_kfloat("initial_rate", spec->initial_rate);
            j_kfloat("final_rate", spec->final_rate);
            j_kerror("append_result", sb_time_axis_append_segment(&axis, segment));
            j_end_object();
        }
        j_end_array();

        j_time_axis_mapping(&axis);
        j_end_object();

        sb_time_axis_destroy(&axis);
    }
    j_end_array();
}

/* ************************************************************************** */
/* Screenplays                                                               */
/* ************************************************************************** */

/** Light program: red for 2 s, fade to blue in 2 s, then green */
static const uint8_t LIGHT_PROGRAM_A[] = {
    CMD_SET_COLOR, 255, 0, 0, 100, /* 100 frames = 2 s */
    CMD_FADE_TO_COLOR, 0, 0, 255, 100,
    CMD_SET_COLOR, 0, 255, 0, 0,
    CMD_END
};

/** Light program: white for 1 s, black for 1 s, repeated forever */
static const uint8_t LIGHT_PROGRAM_B[] = {
    CMD_LOOP_BEGIN, 0, /* zero iterations means forever */
    CMD_SET_WHITE, 50, /* 50 frames = 1 s */
    CMD_SET_BLACK, 50,
    CMD_LOOP_END,
    CMD_END
};

static sb_trajectory_t* build_trajectory(sb_vector3_with_yaw_t start, sb_vector3_with_yaw_t end, uint32_t duration_msec)
{
    sb_trajectory_builder_t builder;
    sb_trajectory_t* trajectory = sb_trajectory_new();

    if (trajectory == 0) {
        fprintf(stderr, "out of memory\n");
        exit(1);
    }

    CHECK(sb_trajectory_builder_init(&builder, 1, 0));
    CHECK(sb_trajectory_builder_set_start_position(&builder, start));
    CHECK(sb_trajectory_builder_move_to_in_time(&builder, end, duration_msec, 2000));
    CHECK(sb_trajectory_update_from_builder(trajectory, &builder));
    sb_trajectory_builder_destroy(&builder);

    return trajectory;
}

static sb_light_program_t* make_light_program(const uint8_t* bytecode, size_t size)
{
    sb_light_program_t* program = sb_light_program_new();
    uint8_t* copy = checked_malloc(size);

    if (program == 0) {
        fprintf(stderr, "out of memory\n");
        exit(1);
    }
    memcpy(copy, bytecode, size);
    CHECK(sb_light_program_update_from_bytes(program, copy, size));
    return program;
}

typedef struct {
    int32_t duration_msec; /* negative means infinite */
    int32_t origin_msec;
    sb_vector3_with_yaw_t start, end;
    uint32_t move_msec;
    const uint8_t* light_program;
    size_t light_program_size;
    float yaw_deg; /* NaN means no yaw control */
    time_segment_spec_t time_axis[3];
    size_t num_time_segments;
} scene_spec_t;

typedef struct {
    const char* name;
    scene_spec_t scenes[3];
    size_t num_scenes;
    uint32_t sample_until_msec;
} screenplay_scenario_t;

static const screenplay_scenario_t SCREENPLAY_SCENARIOS[] = {
    { "two_scenes", {
                        { 8000, 0, P(0, 0, 0), P(0, 0, 5000), 6000, LIGHT_PROGRAM_A, sizeof(LIGHT_PROGRAM_A), NAN, { REALTIME(0) }, 0 },
                        { 10000, 8000, P(0, 0, 5000), P(10000, 0, 5000), 8000, LIGHT_PROGRAM_B, sizeof(LIGHT_PROGRAM_B), 90, { REALTIME(0) }, 0 },
                    },
        2, 20000 },
    { "warped_scene", {
                          { 12000, 0, P(0, 0, 0), P(6000, 0, 3000), 8000, LIGHT_PROGRAM_A, sizeof(LIGHT_PROGRAM_A), 45, { REALTIME(2000), { 4000, NAN, 1, 0.5f }, { 6000, NAN, 0.5f, 0.5f } }, 3 },
                      },
        1, 14000 },
    { "frozen_then_infinite", {
                                  { 4000, 0, P(0, 0, 0), P(0, 3000, 3000), 3000, LIGHT_PROGRAM_B, sizeof(LIGHT_PROGRAM_B), NAN, { REALTIME(1500), { 1500, NAN, 0, 0 }, REALTIME(1000) }, 3 },
                                  { -1, 4000, P(0, 3000, 3000), P(0, 0, 0), 5000, LIGHT_PROGRAM_A, sizeof(LIGHT_PROGRAM_A), NAN, { REALTIME(0) }, 0 },
                              },
        2, 12000 },
    /* Scenes are placed on the show clock by their origins; the show
     * controller reports the end of the show between scenes */
    { "gap_between_scenes", {
                                { 4000, 0, P(0, 0, 0), P(0, 0, 2000), 3000, LIGHT_PROGRAM_A, sizeof(LIGHT_PROGRAM_A), NAN, { REALTIME(0) }, 0 },
                                { 4000, 6000, P(0, 0, 2000), P(2000, 0, 2000), 3000, LIGHT_PROGRAM_B, sizeof(LIGHT_PROGRAM_B), NAN, { REALTIME(0) }, 0 },
                            },
        2, 11000 },
    /* The first scene that contains a given time wins */
    { "overlapping_scenes", {
                                { 6000, 0, P(0, 0, 0), P(0, 0, 2000), 3000, LIGHT_PROGRAM_A, sizeof(LIGHT_PROGRAM_A), NAN, { REALTIME(0) }, 0 },
                                { 6000, 4000, P(5000, 0, 0), P(5000, 0, 2000), 3000, LIGHT_PROGRAM_B, sizeof(LIGHT_PROGRAM_B), NAN, { REALTIME(0) }, 0 },
                            },
        2, 11000 },
    { "scene_with_origin", {
                               { 10000, 2000, P(0, 0, 0), P(4000, 4000, 2000), 8000, LIGHT_PROGRAM_A, sizeof(LIGHT_PROGRAM_A), NAN, { REALTIME(0) }, 0 },
                           },
        1, 11000 },
};

/** Returns the components of a control output, determined one by one */
static int j_output_components(const sb_control_output_t* output)
{
    static const int components[] = {
        SB_CONTROL_OUTPUT_POSITION, SB_CONTROL_OUTPUT_VELOCITY, SB_CONTROL_OUTPUT_LIGHTS,
        SB_CONTROL_OUTPUT_YAW, SB_CONTROL_OUTPUT_YAW_RATE
    };
    int result = 0;
    size_t i;

    for (i = 0; i < sizeof(components) / sizeof(components[0]); i++) {
        if (sb_control_output_has_all_components_in(output, components[i])) {
            result |= components[i];
        }
    }
    return result;
}

static void j_screenplay_scenarios(void)
{
    const screenplay_scenario_t* scenario;
    const scene_spec_t* spec;
    sb_screenplay_t screenplay;
    sb_screenplay_scene_t* scene;
    sb_show_controller_t ctrl;
    const sb_control_output_t* output;
    sb_trajectory_t* trajectory;
    sb_light_program_t* program;
    sb_yaw_control_t* yaw;
    sb_vector3_t vec;
    sb_rgb_color_t color;
    float value;
    ssize_t scene_index;
    uint32_t t;
    size_t i, j, k;

    j_key("screenplays");
    j_begin_array();
    for (i = 0; i < sizeof(SCREENPLAY_SCENARIOS) / sizeof(SCREENPLAY_SCENARIOS[0]); i++) {
        scenario = &SCREENPLAY_SCENARIOS[i];
        CHECK(sb_screenplay_init(&screenplay));

        j_begin_object();
        j_kstring("scenario", scenario->name);
        j_key("scenes");
        j_begin_array();
        for (j = 0; j < scenario->num_scenes; j++) {
            spec = &scenario->scenes[j];
            CHECK(sb_screenplay_append_new_scene(&screenplay, &scene));

            j_begin_object();
            if (spec->duration_msec < 0) {
                sb_screenplay_scene_set_infinite(scene);
                j_key("duration_ms");
                j_null();
            } else {
                sb_screenplay_scene_set_duration_msec(scene, (uint32_t)spec->duration_msec);
                j_kint("duration_ms", (long)spec->duration_msec);
            }
            j_kint("origin_ms", (long)spec->origin_msec);
            j_kerror("set_origin_result", sb_screenplay_scene_set_origin_msec(scene, spec->origin_msec));

            trajectory = build_trajectory(spec->start, spec->end, spec->move_msec);
            sb_screenplay_scene_set_trajectory(scene, trajectory);
            j_key("trajectory_hex");
            j_trajectory_hex(trajectory);
            SB_XDECREF(trajectory);

            program = make_light_program(spec->light_program, spec->light_program_size);
            sb_screenplay_scene_set_light_program(scene, program);
            j_key("light_program_hex");
            j_hex(spec->light_program, spec->light_program_size);
            SB_XDECREF(program);

            j_key("yaw_deg");
            if (isnan(spec->yaw_deg)) {
                j_null();
            } else {
                yaw = sb_yaw_control_new();
                if (yaw == 0) {
                    fprintf(stderr, "out of memory\n");
                    exit(1);
                }
                CHECK(sb_yaw_control_set_constant_yaw(yaw, spec->yaw_deg));
                sb_screenplay_scene_set_yaw_control(scene, yaw);
                SB_XDECREF(yaw);
                j_float(spec->yaw_deg);
            }

            j_key("time_axis");
            j_begin_array();
            for (k = 0; k < spec->num_time_segments; k++) {
                CHECK(sb_time_axis_append_segment(sb_screenplay_scene_get_time_axis(scene),
                    sb_time_segment_make(spec->time_axis[k].duration_msec, spec->time_axis[k].initial_rate, spec->time_axis[k].final_rate)));
                j_begin_object();
                j_kint("duration_ms", (long)spec->time_axis[k].duration_msec);
                j_kfloat("initial_rate", spec->time_axis[k].initial_rate);
                j_kfloat("final_rate", spec->time_axis[k].final_rate);
                j_end_object();
            }
            j_end_array();
            j_end_object();
        }
        j_end_array();

        CHECK(sb_show_controller_init(&ctrl, &screenplay));
        j_key("samples");
        j_begin_array();
        for (k = 0, t = 0; t <= scenario->sample_until_msec || k < 3; t += SAMPLE_INTERVAL_MSEC) {
            if (t > scenario->sample_until_msec) {
                /* A few extreme timestamps after the regular samples */
                static const uint32_t extremes[] = { INT32_MAX, (uint32_t)INT32_MAX + 1, UINT32_MAX };
                t = extremes[k++];
            }
            CHECK(sb_show_controller_update_time_msec(&ctrl, t));
            output = sb_show_controller_get_current_output(&ctrl);
            sb_screenplay_get_scene_ptr_at_time_msec(&screenplay, t, &scene_index);

            j_begin_object();
            j_kint("time_ms", (long)t);
            j_kint("scene", (long)scene_index);
            j_key("position_mm");
            if (sb_control_output_get_position_if_set(output, &vec)) {
                j_vector3(vec);
            } else {
                j_null();
            }
            j_key("velocity_mm_s");
            if (sb_control_output_get_velocity_if_set(output, &vec)) {
                j_vector3(vec);
            } else {
                j_null();
            }
            j_key("yaw_deg");
            if (sb_control_output_get_yaw_if_set(output, &value)) {
                j_float(value);
            } else {
                j_null();
            }
            j_key("yaw_rate_deg_s");
            if (sb_control_output_get_yaw_rate_if_set(output, &value)) {
                j_float(value);
            } else {
                j_null();
            }
            j_key("color");
            if (sb_control_output_get_color_if_set(output, &color)) {
                j_color(color);
            } else {
                j_null();
            }
            j_kint("components", (long)j_output_components(output));
            {
                sb_control_output_time_t output_time = sb_show_controller_get_current_output_time(&ctrl);
                j_kint("output_time_ms", (long)output_time.time_msec);
                j_kint("time_in_scene_ms", (long)output_time.time_in_scene_msec);
                j_kfloat("warped_time_in_scene_sec", output_time.warped_time_in_scene_sec);
            }
            j_key("ended");
            j_bool(sb_show_controller_has_reached_end(&ctrl));
            j_end_object();
            if (k >= 3) {
                break;
            }
        }
        j_end_array();
        j_end_object();

        sb_show_controller_destroy(&ctrl);
        sb_screenplay_destroy(&screenplay);
    }
    j_end_array();
}

/* ************************************************************************** */
/* Motion helpers                                                            */
/* ************************************************************************** */

static void j_motion_helpers(void)
{
    static const float travel_times[][3] = {
        /* distance, speed, acceleration */
        { 0, 1000, 1000 },
        { 1, 1000, 1000 },
        { 100, 1000, 1000 },
        { 1000, 1000, 1000 },
        { 10000, 1000, 1000 },
        { 10000, 2000, 500 },
        { 10000, 1000, INFINITY },
        { 1000, 0, 1000 },
        { 1000, 1000, 0 },
        { -1, 1000, 1000 },
        { 1000, INFINITY, 1000 },
        { 1, 0.5f, 1000 },
        { 1, 1000, 0.5f },
        { 1, 1, 1 },
        { 0.5f, 1000, 1000 },
    };
    static const float travel_velocities[][3] = {
        /* distance, time, acceleration */
        { 0, 10, 1000 },
        { 1000, 10, 1000 },
        { 10000, 10, 1000 },
        { 10000, 2, 1000 },
        { 10000, 10, INFINITY },
        { 1000, 0, 1000 },
        { 1000, 10, 0 },
        { -1, 10, 1000 },
        { 1, 10, 1000 },
        { 1000, 2, 1000 }, /* exactly the shortest possible time */
        { 1000, 1.99f, 1000 },
        { 1, 0.5f, 1000 },
        { 1, 10, 0.5f },
    };
    static const struct {
        sb_vector3_with_yaw_t start, start_vel, end, end_vel;
        float duration_sec;
    } beziers[] = {
        { P(0, 0, 0), P(0, 0, 0), P(1000, 0, 0), P(0, 0, 0), 2 },
        { P(0, 0, 0), P(500, 0, 0), P(1000, 0, 0), P(500, 0, 0), 2 },
        { P(0, 0, 1000), P(0, 0, 1000), P(3000, 2000, 1000), P(-500, 0, 0), 4 },
        { { 0, 0, 0, 0 }, { 0, 0, 0, 90 }, { 0, 0, 0, 180 }, { 0, 0, 0, 0 }, 2 },
        { P(0, 0, 0), P(0, 0, 0), P(1000, 0, 0), P(0, 0, 0), 0 },
        { P(0, 0, 0), P(0, 0, 0), P(1000, 0, 0), P(0, 0, 0), -1 },
    };
    sb_vector3_with_yaw_t control1, control2;
    sb_error_t retval;
    size_t i;

    j_key("motion");
    j_begin_object();

    j_key("travel_time_for_distance");
    j_begin_array();
    for (i = 0; i < sizeof(travel_times) / sizeof(travel_times[0]); i++) {
        j_begin_object();
        j_kfloat("distance_mm", travel_times[i][0]);
        j_kfloat("speed_mm_s", travel_times[i][1]);
        j_kfloat("acceleration_mm_s2", travel_times[i][2]);
        j_kfloat("travel_time_sec", sb_get_travel_time_for_distance(travel_times[i][0], travel_times[i][1], travel_times[i][2]));
        j_end_object();
    }
    j_end_array();

    j_key("travel_velocity_for_distance");
    j_begin_array();
    for (i = 0; i < sizeof(travel_velocities) / sizeof(travel_velocities[0]); i++) {
        j_begin_object();
        j_kfloat("distance_mm", travel_velocities[i][0]);
        j_kfloat("time_sec", travel_velocities[i][1]);
        j_kfloat("acceleration_mm_s2", travel_velocities[i][2]);
        j_kfloat("velocity_mm_s", sb_get_travel_velocity_for_distance(travel_velocities[i][0], travel_velocities[i][1], travel_velocities[i][2]));
        j_end_object();
    }
    j_end_array();

    j_key("cubic_bezier_from_velocity_constraints");
    j_begin_array();
    for (i = 0; i < sizeof(beziers) / sizeof(beziers[0]); i++) {
        retval = sb_get_cubic_bezier_from_velocity_constraints(beziers[i].start, beziers[i].start_vel,
            beziers[i].end, beziers[i].end_vel, beziers[i].duration_sec, &control1, &control2);
        j_begin_object();
        j_key("start_mm");
        j_vector3_with_yaw(beziers[i].start);
        j_key("start_velocity_mm_s");
        j_vector3_with_yaw(beziers[i].start_vel);
        j_key("end_mm");
        j_vector3_with_yaw(beziers[i].end);
        j_key("end_velocity_mm_s");
        j_vector3_with_yaw(beziers[i].end_vel);
        j_kfloat("duration_sec", beziers[i].duration_sec);
        j_kerror("result", retval);
        if (retval == SB_SUCCESS) {
            j_key("control1_mm");
            j_vector3_with_yaw(control1);
            j_key("control2_mm");
            j_vector3_with_yaw(control2);
        }
        j_end_object();
    }
    j_end_array();

    j_end_object();
}

/* ************************************************************************** */
/* Color helpers                                                             */
/* ************************************************************************** */

static void j_color_helpers(void)
{
    static const float temperatures[] = {
        0, 500, 999, 1000, 1900, 2000, 2700, 4000, 6500, 6600, 6700, 7000, 10000, 20000, 40000, 40001, 100000
    };
    static const sb_rgb_color_t colors[] = {
        { 0, 0, 0 },
        { 255, 255, 255 },
        { 255, 0, 0 },
        { 0, 255, 0 },
        { 0, 0, 255 },
        { 255, 180, 107 },
        { 120, 200, 50 },
        { 10, 10, 10 },
        { 1, 2, 3 },
        { 254, 253, 252 },
    };
    static const float ratios[] = { 0, 0.25f, 0.5f, 0.6f, 0.75f, 1, -0.5f, 1.5f };
    sb_rgbw_conversion_t conv;
    uint16_t encoded;
    size_t i, j, mode;

    j_key("colors");
    j_begin_object();

    /* powf() and logf() may differ between math libraries by an amount that
     * changes the truncated integer value, hence the _approx8 suffix */
    j_key("from_color_temperature");
    j_begin_array();
    for (i = 0; i < sizeof(temperatures) / sizeof(temperatures[0]); i++) {
        j_begin_object();
        j_kfloat("temperature_k", temperatures[i]);
        j_key("color_approx8");
        j_color(sb_rgb_color_from_color_temperature(temperatures[i]));
        j_end_object();
    }
    j_end_array();

    j_key("rgb565");
    j_begin_array();
    for (i = 0; i < sizeof(colors) / sizeof(colors[0]); i++) {
        encoded = sb_rgb_color_encode_rgb565(colors[i]);
        j_begin_object();
        j_key("color");
        j_color(colors[i]);
        j_kint("encoded", encoded);
        j_key("decoded");
        j_color(sb_rgb_color_decode_rgb565(encoded));
        j_end_object();
    }
    j_end_array();

    j_key("linear_interpolation");
    j_begin_array();
    for (i = 0; i < sizeof(ratios) / sizeof(ratios[0]); i++) {
        j_begin_object();
        j_key("first");
        j_color(colors[2]);
        j_key("second");
        j_color(colors[6]);
        j_kfloat("ratio", ratios[i]);
        j_key("result");
        j_color(sb_rgb_color_linear_interpolation(colors[2], colors[6], ratios[i]));
        j_end_object();
    }
    j_end_array();

    j_key("rgbw");
    j_begin_array();
    for (mode = 0; mode < 7; mode++) {
        const char* name;
        int approximate = 0;

        switch (mode) {
        case 0:
            sb_rgbw_conversion_turn_off(&conv);
            name = "off";
            break;
        case 1:
            sb_rgbw_conversion_use_fixed_value(&conv, 128);
            name = "fixed_value_128";
            break;
        case 2:
            sb_rgbw_conversion_use_min_subtraction(&conv);
            name = "min_subtraction";
            break;
        case 3:
            sb_rgbw_conversion_use_color_temperature(&conv, 3000);
            name = "color_temperature_3000";
            approximate = 1;
            break;
        case 4:
            sb_rgbw_conversion_use_color_temperature(&conv, 6000);
            name = "color_temperature_6000";
            approximate = 1;
            break;
        case 5:
            sb_rgbw_conversion_use_reference_color(&conv, sb_rgb_color_make(255, 200, 100));
            name = "reference_color_255_200_100";
            break;
        default:
            sb_rgbw_conversion_use_reference_color(&conv, sb_rgb_color_make(200, 255, 100));
            name = "reference_color_200_255_100";
            break;
        }

        j_begin_object();
        j_kstring("conversion", name);
        j_key("results");
        j_begin_array();
        for (j = 0; j < sizeof(colors) / sizeof(colors[0]); j++) {
            j_begin_object();
            j_key("color");
            j_color(colors[j]);
            j_key(approximate ? "rgbw_approx8" : "rgbw");
            j_rgbw_color(sb_rgb_color_to_rgbw(colors[j], conv));
            j_end_object();
        }
        j_end_array();
        j_end_object();
    }
    j_end_array();

    j_end_object();
}

/* ************************************************************************** */
/* Trajectory statistics                                                     */
/* ************************************************************************** */

typedef struct {
    const char* name;
    sb_trajectory_stat_components_t components;
    float takeoff_speed;
    float acceleration;
    float min_ascent;
    float preferred_descent;
    float verticality_threshold;
} stats_settings_t;

/** Records the components of trajectory statistics that were calculated */
static void j_stats(const sb_trajectory_stats_t* stats)
{
    sb_trajectory_stat_components_t valid = stats->valid_components;

    j_begin_object();
    j_key("initial_position_mm");
    j_vector3_xyz(stats->initial_pos);
    if (valid & SB_TRAJECTORY_STATS_DURATION) {
        j_kint("duration_ms", (long)stats->duration_msec);
    }
    if (valid & SB_TRAJECTORY_STATS_TAKEOFF_TIME) {
        j_kfloat("earliest_above_sec", stats->earliest_above_sec);
        j_kfloat("takeoff_time_sec", stats->takeoff_time_sec);
    }
    if (valid & SB_TRAJECTORY_STATS_START_END_DISTANCE) {
        j_kfloat("start_to_end_distance_xy_mm", stats->start_to_end_distance_xy);
    }
    if (valid & SB_TRAJECTORY_STATS_LANDING_TIME) {
        j_kfloat("landing_time_sec", stats->landing_time_sec);
        j_key("position_at_landing_mm");
        j_vector3_xyz(stats->pos_at_landing_time);
        j_key("velocity_at_landing_mm_s");
        j_vector3_xyz(stats->vel_at_landing_time);
    }
    j_end_object();
}

/* ************************************************************************** */
/* Event lists                                                               */
/* ************************************************************************** */

typedef enum {
    EV_END = 0,
    EV_APPEND,
    EV_INSERT,
    EV_REMOVE,
    EV_SORT,
    EV_ADD_PYRO_OFF_EVENTS,
    EV_ADJUST_TIMESTAMPS,
} event_op_type_t;

typedef struct {
    event_op_type_t type;
    uint32_t time_msec; /* timestamp of the event, or duration of pyro ignitions */
    uint8_t event_type;
    uint8_t subtype;
    uint32_t payload;
    int32_t value; /* index to remove, or timestamp adjustment */
} event_op_t;

#define PYRO_OFF_PAYLOAD UINT32_MAX
#define ON(t, ch)                                  \
    {                                              \
        EV_INSERT, t, SB_EVENT_TYPE_PYRO, ch, 0, 0 \
    }
#define OFF(t, ch)                                                \
    {                                                             \
        EV_INSERT, t, SB_EVENT_TYPE_PYRO, ch, PYRO_OFF_PAYLOAD, 0 \
    }
#define LIGHT(t)                                                                            \
    {                                                                                       \
        EV_INSERT, t, SB_EVENT_TYPE_LIGHT, SB_EVENT_SUBTYPE_LIGHT_SET_SOURCE, 0x04030201, 0 \
    }

typedef struct {
    const char* name;
    event_op_t ops[14];
} event_list_scenario_t;

static const event_list_scenario_t EVENT_LIST_SCENARIOS[] = {
    { "insert_keeps_order", {
                                ON(5000, 1), ON(1000, 2), LIGHT(3000), ON(3000, 3), ON(0, 4), ON(3000, 5), ON(1001, 7), /* between 1000 * t and 1001 * t for t = 1 s */
                                ON(4294000000u, 8), /* close to the end of the time range */
                            } },
    /* Appending is allowed only at or after the last timestamp */
    { "append", {
                    { EV_APPEND, 1000, SB_EVENT_TYPE_PYRO, 1, 0, 0 },
                    { EV_APPEND, 1000, SB_EVENT_TYPE_PYRO, 2, 0, 0 },
                    { EV_APPEND, 3000, SB_EVENT_TYPE_PYRO, 3, 0, 0 },
                    { EV_APPEND, 2000, SB_EVENT_TYPE_PYRO, 4, 0, 0 },
                    { EV_SORT },
                } },
    { "remove", {
                    ON(1000, 1),
                    ON(2000, 2),
                    ON(3000, 3),
                    { EV_REMOVE, 0, 0, 0, 0, 1 },
                    { EV_REMOVE, 0, 0, 0, 0, 5 },
                    { EV_REMOVE, 0, 0, 0, 0, 2 }, /* exactly the size of the list */
                    { EV_REMOVE, 0, 0, 0, 0, 0 },
                } },
    { "add_pyro_off_events", {
                                 ON(1000, 1), ON(2000, 2), OFF(2500, 2), /* off event within the ignition duration is kept */
                                 ON(3000, 3), OFF(10000, 3), /* off event too late, a new one is added */
                                 LIGHT(1500), /* not a pyro event */
                                 OFF(5000, 4), /* off event without an on event */
                                 ON(8000, 1), /* same channel again */
                                 ON(4294967000u, 6), /* the off event would overflow the timestamp */
                                 { EV_ADD_PYRO_OFF_EVENTS, 1000 }, { EV_ADD_PYRO_OFF_EVENTS, 1000 }, /* nothing more to add */
                             } },
    /* The search for an existing "off" event does not stop at the next "on"
     * event of the same channel */
    { "add_pyro_off_events_channel_fired_twice", {
                                                     ON(1000, 1),
                                                     ON(8000, 1),
                                                     OFF(8500, 1),
                                                     { EV_ADD_PYRO_OFF_EVENTS, 1000 },
                                                 } },
    { "add_pyro_off_events_zero_duration", {
                                               ON(1000, 1),
                                               ON(1000, 2),
                                               { EV_ADD_PYRO_OFF_EVENTS, 0 },
                                           } },
    { "adjust_timestamps_by_type", {
                                       ON(1000, 1),
                                       LIGHT(2000),
                                       ON(3000, 2),
                                       ON(4294966000u, 3),
                                       { EV_ADJUST_TIMESTAMPS, 0, SB_EVENT_TYPE_PYRO, 0, 0, 500 },
                                       { EV_ADJUST_TIMESTAMPS, 0, SB_EVENT_TYPE_LIGHT, 0, 0, -5000 }, /* clamped to zero */
                                       { EV_ADJUST_TIMESTAMPS, 0, SB_EVENT_TYPE_PYRO, 0, 0, 2000 }, /* clamped to UINT32_MAX */
                                       { EV_ADJUST_TIMESTAMPS, 0, SB_EVENT_TYPE_PYRO, 0, 0, 0 },
                                       { EV_ADJUST_TIMESTAMPS, 0, SB_EVENT_TYPE_PYRO, 0, 0, -1500 },
                                   } },
};

static void j_event(const sb_event_t* event)
{
    j_begin_array();
    j_int((long)event->time_msec);
    j_int((long)event->type);
    j_int((long)event->subtype);
    j_hex(event->payload.as_buf, sizeof(event->payload.as_buf));
    j_end_array();
}

static void j_event_list(const sb_event_list_t* events)
{
    size_t i;

    j_key("events");
    j_begin_array();
    for (i = 0; i < sb_event_list_size(events); i++) {
        j_event(sb_event_list_get_ptr_const(events, i));
    }
    j_end_array();
    j_key("is_sorted");
    j_bool(sb_event_list_is_sorted(events));
    j_key("is_empty");
    j_bool(sb_event_list_is_empty(events));
}

static void j_event_index(const sb_event_list_t* events, const sb_event_t* event)
{
    if (event) {
        j_int((long)(event - sb_event_list_get_ptr_const(events, 0)));
    } else {
        j_null();
    }
}

static void j_event_list_scenarios(void)
{
    static const float query_times_sec[] = { -1, 0, 0.5f, 1, 2.5f, 3, 3.5f, 4292000.0f, 4294966.0f, 4294967.5f, INFINITY, NAN };
    const event_list_scenario_t* scenario;
    const event_op_t* op;
    sb_event_list_t events;
    sb_event_list_player_t player;
    sb_event_t event;
    sb_error_t retval;
    size_t i, j, n = sizeof(query_times_sec) / sizeof(query_times_sec[0]);
    const sb_event_t* next;

    j_key("event_lists");
    j_begin_array();
    for (i = 0; i < sizeof(EVENT_LIST_SCENARIOS) / sizeof(EVENT_LIST_SCENARIOS[0]); i++) {
        scenario = &EVENT_LIST_SCENARIOS[i];
        CHECK(sb_event_list_init(&events, 2));

        j_begin_object();
        j_kstring("scenario", scenario->name);
        j_key("ops");
        j_begin_array();
        for (j = 0; j < sizeof(scenario->ops) / sizeof(scenario->ops[0]) && scenario->ops[j].type != EV_END; j++) {
            op = &scenario->ops[j];
            memset(&event, 0, sizeof(event));
            event.time_msec = op->time_msec;
            event.type = op->event_type;
            event.subtype = op->subtype;
            event.payload.as_uint32 = op->payload;

            j_begin_object();
            switch (op->type) {
            case EV_APPEND:
            case EV_INSERT:
                j_kstring("op", op->type == EV_APPEND ? "append" : "insert");
                j_key("event");
                j_event(&event);
                retval = op->type == EV_APPEND ? sb_event_list_append(&events, &event) : sb_event_list_insert(&events, &event);
                break;
            case EV_REMOVE:
                j_kstring("op", "remove");
                j_kint("index", op->value);
                retval = sb_event_list_remove(&events, (size_t)op->value);
                break;
            case EV_SORT:
                j_kstring("op", "sort");
                sb_event_list_sort(&events);
                retval = SB_SUCCESS;
                break;
            case EV_ADD_PYRO_OFF_EVENTS:
                j_kstring("op", "add_pyro_off_events");
                j_kint("duration_ms", (long)op->time_msec);
                retval = sb_event_list_add_pyro_off_events(&events, op->time_msec);
                break;
            default:
                j_kstring("op", "adjust_timestamps_by_type");
                j_kint("type", op->event_type);
                j_kint("delta_ms", op->value);
                sb_event_list_adjust_timestamps_by_type(&events, (sb_event_type_t)op->event_type, op->value);
                retval = SB_SUCCESS;
                break;
            }
            j_kerror("result", retval);
            j_event_list(&events);
            j_end_object();
        }
        j_end_array();

        /* Playing the resulting list; the player requires a sorted list */
        if (!sb_event_list_is_sorted(&events)) {
            sb_event_list_sort(&events);
        }
        CHECK(sb_event_list_player_init(&player, &events));
        j_key("sorted_for_playback");
        j_begin_object();
        j_event_list(&events);
        j_end_object();

        j_key("next_events");
        j_begin_array();
        while ((next = sb_event_list_player_get_next_event(&player)) != NULL) {
            j_event_index(&events, next);
        }
        j_end_array();

        j_key("query_times_sec");
        j_begin_array();
        for (j = 0; j < n; j++) {
            j_float(query_times_sec[j]);
        }
        j_end_array();

        /* Consecutive queries, as during playback */
        sb_event_list_player_rewind(&player);
        j_key("next_event_not_later_than");
        j_begin_array();
        for (j = 0; j < n; j++) {
            j_event_index(&events, sb_event_list_player_get_next_event_not_later_than(&player, query_times_sec[j]));
        }
        j_end_array();

        j_key("peek_after_seek");
        j_begin_array();
        for (j = 0; j < n; j++) {
            sb_event_list_player_seek(&player, query_times_sec[j]);
            j_event_index(&events, sb_event_list_player_peek_next_event(&player));
        }
        j_end_array();

        j_end_object();
        sb_event_list_player_destroy(&player);
        SB_DECREF_STATIC(&events);
    }
    j_end_array();
}

/* ************************************************************************** */
/* Light programs                                                            */
/* ************************************************************************** */

/** Number of steps and the time range of the light player that are recorded */
#define MAX_LIGHT_CHANGES 600
#define LIGHT_SAMPLES_UNTIL_MSEC 12000

static const uint8_t LP_GRAY_AND_FADES[] = {
    CMD_SET_GRAY, 128, 50,
    CMD_FADE_TO_GRAY, 0, 50,
    CMD_FADE_TO_WHITE, 50,
    CMD_FADE_TO_BLACK, 50,
    CMD_SET_WHITE, 25,
    CMD_SET_BLACK, 25,
    CMD_FADE_TO_COLOR, 10, 20, 30, 0, /* zero-duration fade */
    CMD_SLEEP, 25,
    CMD_END
};
static const uint8_t LP_SLEEP_AND_WAIT_UNTIL[] = {
    CMD_SET_COLOR, 255, 0, 0, 0,
    CMD_SLEEP, 50,
    CMD_SET_COLOR, 0, 0, 255, 0,
    CMD_WAIT_UNTIL, 0x96, 0x01, /* 150 frames = 3 s after the start */
    CMD_SET_COLOR, 0, 255, 0, 0,
    CMD_WAIT_UNTIL, 100, /* already in the past */
    CMD_SET_WHITE, 25,
    CMD_NOP,
    CMD_END
};
static const uint8_t LP_NESTED_LOOPS[] = {
    CMD_LOOP_BEGIN, 2,
    CMD_LOOP_BEGIN, 3,
    CMD_SET_WHITE, 5,
    CMD_SET_BLACK, 5,
    CMD_LOOP_END,
    CMD_SET_COLOR, 255, 0, 0, 10,
    CMD_LOOP_END,
    CMD_SET_GRAY, 64, 10,
    CMD_END
};
static const uint8_t LP_MULTI_BYTE_DURATIONS[] = {
    CMD_SET_WHITE, 0x80, 0x01, /* 128 frames */
    CMD_SET_BLACK, 0xe8, 0x01, /* 232 frames */
    CMD_SET_WHITE, 0,
    CMD_END
};
static const uint8_t LP_LONGEST_VARINT[] = {
    CMD_SET_WHITE, 0xff, 0xff, 0xff, 0xff, 0x0f, /* UINT32_MAX frames */
    CMD_SET_BLACK, 0,
    CMD_END
};
static const uint8_t LP_OVERFLOWING_VARINTS[] = {
    CMD_SET_WHITE, 0x80, 0x80, 0x80, 0x80, 0x10, /* too large, clamped */
    CMD_SET_BLACK, 0,
    CMD_END
};
static const uint8_t LP_TOO_LONG_VARINT[] = {
    CMD_SET_WHITE, 0x81, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, /* too long, clamped */
    CMD_SET_BLACK, 0,
    CMD_END
};
static const uint8_t LP_LOOP_END_WITHOUT_BEGIN[] = {
    CMD_LOOP_END,
    CMD_SET_WHITE, 10,
    CMD_END
};
static const uint8_t LP_INFINITE_EMPTY_LOOP[] = {
    CMD_SET_WHITE, 10,
    CMD_LOOP_BEGIN, 0,
    CMD_LOOP_END,
    CMD_SET_BLACK, 10,
    CMD_END
};
static const uint8_t LP_INVALID_OPCODE[] = {
    CMD_SET_WHITE, 10,
    0x7f,
    CMD_SET_BLACK, 10,
    CMD_END
};
static const uint8_t LP_EMPTY[] = { CMD_END };

static const struct {
    const char* name;
    const uint8_t* bytecode;
    size_t size;
} LIGHT_PROGRAM_SCENARIOS[] = {
    { "gray_and_fades", LP_GRAY_AND_FADES, sizeof(LP_GRAY_AND_FADES) },
    { "sleep_and_wait_until", LP_SLEEP_AND_WAIT_UNTIL, sizeof(LP_SLEEP_AND_WAIT_UNTIL) },
    { "nested_loops", LP_NESTED_LOOPS, sizeof(LP_NESTED_LOOPS) },
    { "multi_byte_durations", LP_MULTI_BYTE_DURATIONS, sizeof(LP_MULTI_BYTE_DURATIONS) },
    { "longest_varint", LP_LONGEST_VARINT, sizeof(LP_LONGEST_VARINT) },
    { "overflowing_varint", LP_OVERFLOWING_VARINTS, sizeof(LP_OVERFLOWING_VARINTS) },
    { "too_long_varint", LP_TOO_LONG_VARINT, sizeof(LP_TOO_LONG_VARINT) },
    { "loop_end_without_begin", LP_LOOP_END_WITHOUT_BEGIN, sizeof(LP_LOOP_END_WITHOUT_BEGIN) },
    { "infinite_empty_loop", LP_INFINITE_EMPTY_LOOP, sizeof(LP_INFINITE_EMPTY_LOOP) },
    { "invalid_opcode", LP_INVALID_OPCODE, sizeof(LP_INVALID_OPCODE) },
    { "only_end", LP_EMPTY, sizeof(LP_EMPTY) },
    { "no_bytecode", LP_EMPTY, 0 },
};

static void j_light_program_playback(sb_light_program_t* program)
{
    sb_light_player_t player;
    unsigned long t = 0, next = 0;
    size_t steps = 0;
    int ended = 0;
    uint32_t sample;

    j_key("bytecode_hex");
    j_hex(SB_BUFFER(program->buffer), sb_buffer_size(&program->buffer));

    CHECK(sb_light_player_init(&player, program));
    j_key("changes_ms");
    j_begin_array();
    while (steps < MAX_LIGHT_CHANGES) {
        ended = sb_light_player_seek(&player, t, &next);
        j_begin_array();
        j_int((long)t);
        j_int((long)next);
        j_end_array();
        steps++;
        if (ended || next <= t) {
            break;
        }
        t = next;
    }
    j_end_array();
    j_key("ended");
    j_bool(ended);

    j_key("color");
    j_begin_array();
    for (sample = 0; sample <= LIGHT_SAMPLES_UNTIL_MSEC; sample += SAMPLE_INTERVAL_MSEC) {
        j_color(sb_light_player_get_color_at(&player, sample));
    }
    j_end_array();
    sb_light_player_destroy(&player);
}

static void j_light_program_scenarios(void)
{
    static const sb_rgb_color_t constant_colors[] = { { 10, 20, 30 }, { 0, 0, 0 }, { 255, 255, 255 } };
    sb_light_program_t* program;
    size_t i;

    j_key("light_programs");
    j_begin_array();
    for (i = 0; i < sizeof(LIGHT_PROGRAM_SCENARIOS) / sizeof(LIGHT_PROGRAM_SCENARIOS[0]); i++) {
        j_begin_object();
        j_kstring("scenario", LIGHT_PROGRAM_SCENARIOS[i].name);
        program = make_light_program(LIGHT_PROGRAM_SCENARIOS[i].bytecode, LIGHT_PROGRAM_SCENARIOS[i].size);
        j_light_program_playback(program);
        SB_XDECREF(program);
        j_end_object();
    }
    for (i = 0; i < sizeof(constant_colors) / sizeof(constant_colors[0]); i++) {
        program = sb_light_program_new();
        if (program == 0) {
            fprintf(stderr, "out of memory\n");
            exit(1);
        }
        j_begin_object();
        j_kstring("scenario", "set_constant_color");
        j_key("constant_color");
        j_color(constant_colors[i]);
        j_kerror("result", sb_light_program_set_constant_color(program, constant_colors[i]));
        j_light_program_playback(program);
        j_end_object();
        SB_XDECREF(program);
    }
    j_end_array();
}

/* ************************************************************************** */
/* Screenplay queries                                                        */
/* ************************************************************************** */

/** Builds a show file in memory with a single trajectory block */
static uint8_t* make_show_file(sb_trajectory_t* trajectory, size_t* length)
{
    size_t body = sb_buffer_size(&trajectory->buffer);
    uint8_t* result = checked_malloc(5 + 3 + body);

    memcpy(result, "skyb\x01", 5);
    result[5] = SB_BINARY_BLOCK_TRAJECTORY;
    result[6] = body & 0xff;
    result[7] = (body >> 8) & 0xff;
    memcpy(result + 8, SB_BUFFER(trajectory->buffer), body);
    *length = 5 + 3 + body;
    return result;
}

static void j_scene_index(sb_screenplay_t* screenplay, const sb_screenplay_scene_t* scene)
{
    size_t i;

    for (i = 0; i < sb_screenplay_size(screenplay); i++) {
        if (sb_screenplay_get_scene_ptr(screenplay, i) == scene) {
            j_int((long)i);
            return;
        }
    }
    j_null();
}

static void j_screenplay_queries(void)
{
    static const float warped_times_sec[] = { -1, 0, 0.5f, 2, 3.9f, 4, 5, 6, 100, NAN };
    static const float durations_sec[] = { 0, 0.0004f, 0.0005f, 2.3456f, -1, NAN, INFINITY, -INFINITY, 4294967.0f, 4294967.5f };
    sb_screenplay_t screenplay, other;
    sb_screenplay_scene_t *scenes[3], *scene, *foreign;
    sb_trajectory_t* trajectory;
    float value;
    sb_error_t retval;
    uint8_t* show_data;
    size_t i, j, show_length;
    uint8_t tag;

    CHECK(sb_screenplay_init(&screenplay));
    CHECK(sb_screenplay_init(&other));

    /* Scene 0: tag 1, dynamic duration, trajectory longer than the scene */
    CHECK(sb_screenplay_append_new_scene(&screenplay, &scenes[0]));
    sb_screenplay_scene_set_duration_msec(scenes[0], 5000);
    sb_screenplay_scene_set_tag(scenes[0], 1);
    sb_screenplay_scene_set_flag(scenes[0], SB_SCREENPLAY_SCENE_FLAG_DYNAMIC_DURATION);
    trajectory = build_trajectory((sb_vector3_with_yaw_t)P(0, 0, 0), (sb_vector3_with_yaw_t)P(0, 0, 3000), 6000);
    sb_screenplay_scene_set_trajectory(scenes[0], trajectory);
    SB_XDECREF(trajectory);
    CHECK(sb_time_axis_append_segment(sb_screenplay_scene_get_time_axis(scenes[0]), sb_time_segment_make_realtime(5000)));

    /* Scene 1: tag 2, starts at 5 s, double speed then real time */
    CHECK(sb_screenplay_append_new_scene(&screenplay, &scenes[1]));
    sb_screenplay_scene_set_duration_msec(scenes[1], 4000);
    CHECK(sb_screenplay_scene_set_origin_msec(scenes[1], 5000));
    sb_screenplay_scene_set_tag(scenes[1], 2);
    trajectory = build_trajectory((sb_vector3_with_yaw_t)P(0, 0, 3000), (sb_vector3_with_yaw_t)P(3000, 0, 3000), 8000);
    sb_screenplay_scene_set_trajectory(scenes[1], trajectory);
    SB_XDECREF(trajectory);
    CHECK(sb_time_axis_append_segment(sb_screenplay_scene_get_time_axis(scenes[1]), sb_time_segment_make_constant_rate(2000, 2)));
    CHECK(sb_time_axis_append_segment(sb_screenplay_scene_get_time_axis(scenes[1]), sb_time_segment_make_realtime(2000)));
    CHECK(sb_time_axis_set_origin_msec(sb_screenplay_scene_get_time_axis(scenes[1]), 5000));

    /* Scene 2: tag 1 again, starts at 9 s, infinite, no time axis segments */
    CHECK(sb_screenplay_append_new_scene(&screenplay, &scenes[2]));
    sb_screenplay_scene_set_infinite(scenes[2]);
    CHECK(sb_screenplay_scene_set_origin_msec(scenes[2], 9000));
    sb_screenplay_scene_set_tag(scenes[2], 1);
    trajectory = build_trajectory((sb_vector3_with_yaw_t)P(3000, 0, 3000), (sb_vector3_with_yaw_t)P(3000, 0, 0), 3000);
    sb_screenplay_scene_set_trajectory(scenes[2], trajectory);
    SB_XDECREF(trajectory);
    CHECK(sb_time_axis_set_origin_msec(sb_screenplay_scene_get_time_axis(scenes[2]), 9000));

    CHECK(sb_screenplay_append_new_scene(&other, &foreign));

    j_key("screenplay_queries");
    j_begin_object();

    j_key("scenes");
    j_begin_array();
    for (i = 0; i < 3; i++) {
        scene = scenes[i];
        j_begin_object();
        j_kint("tag", sb_screenplay_scene_get_tag(scene));
        j_kint("flags", sb_screenplay_scene_get_flags(scene));
        j_kint("origin_ms", (long)sb_screenplay_scene_get_origin_msec(scene));
        j_key("duration_ms");
        if (sb_screenplay_scene_is_infinite(scene)) {
            j_null();
        } else {
            j_int((long)sb_screenplay_scene_get_duration_msec(scene));
        }
        j_key("is_infinite");
        j_bool(sb_screenplay_scene_is_infinite(scene));
        j_key("has_any_flag");
        j_begin_array();
        for (j = 0; j < 4; j++) {
            j_bool(sb_screenplay_scene_has_any_flag(scene, (sb_screenplay_scene_flag_mask_t)j) != 0);
        }
        j_end_array();
        j_key("has_all_flags");
        j_begin_array();
        for (j = 0; j < 4; j++) {
            j_bool(sb_screenplay_scene_has_all_flags(scene, (sb_screenplay_scene_flag_mask_t)j) != 0);
        }
        j_end_array();
        retval = sb_screenplay_scene_get_uncovered_trajectory_duration_sec(scene, &value);
        j_kerror("uncovered_trajectory_duration_result", retval);
        j_kfloat("uncovered_trajectory_duration_sec", retval == SB_SUCCESS ? value : NAN);
        j_kfloat("warped_time_remaining_from_trajectory_sec",
            sb_screenplay_scene_get_warped_time_remaining_from_trajectory_at_end_of_time_axis(scene));
        j_key("contained");
        j_bool(sb_screenplay_contains_scene(&screenplay, scene));
        j_end_object();
    }
    j_end_array();

    j_key("contains_foreign_scene");
    j_bool(sb_screenplay_contains_scene(&screenplay, foreign));

    j_key("first_scene_with_tag");
    j_begin_array();
    for (tag = 0; tag < 4; tag++) {
        j_scene_index(&screenplay, sb_screenplay_find_first_scene_with_tag(&screenplay, tag));
    }
    j_end_array();

    /* Show clock time where a scene with a given tag reaches a given warped
     * time; UINT32_MAX (or infinity) means that it never does */
    j_key("warped_times_sec");
    j_begin_array();
    for (j = 0; j < sizeof(warped_times_sec) / sizeof(warped_times_sec[0]); j++) {
        j_float(warped_times_sec[j]);
    }
    j_end_array();
    j_key("time_for_tag_and_warped_time");
    j_begin_array();
    for (tag = 0; tag < 4; tag++) {
        j_begin_object();
        j_kint("tag", tag);
        j_key("time_ms");
        j_begin_array();
        for (j = 0; j < sizeof(warped_times_sec) / sizeof(warped_times_sec[0]); j++) {
            j_int((long)sb_screenplay_get_time_msec_for_scene_tag_and_warped_time_in_scene(&screenplay, tag, warped_times_sec[j]));
        }
        j_end_array();
        j_key("time_sec");
        j_begin_array();
        for (j = 0; j < sizeof(warped_times_sec) / sizeof(warped_times_sec[0]); j++) {
            j_float(sb_screenplay_get_time_sec_for_scene_tag_and_warped_time_in_scene(&screenplay, tag, warped_times_sec[j]));
        }
        j_end_array();
        j_end_object();
    }
    j_end_array();

    /* Flags of a scene */
    j_key("flag_operations");
    j_begin_array();
    sb_screenplay_scene_set_flags(foreign, 0x0f, 0x05);
    j_int(sb_screenplay_scene_get_flags(foreign));
    sb_screenplay_scene_set_flags(foreign, 0x03, 0x02);
    j_int(sb_screenplay_scene_get_flags(foreign));
    sb_screenplay_scene_clear_flag(foreign, 0x04);
    j_int(sb_screenplay_scene_get_flags(foreign));
    sb_screenplay_scene_set_flag(foreign, 0x80);
    j_int(sb_screenplay_scene_get_flags(foreign));
    j_end_array();

    /* Setting the duration of a scene in seconds */
    j_key("set_duration_sec");
    j_begin_array();
    for (j = 0; j < sizeof(durations_sec) / sizeof(durations_sec[0]); j++) {
        sb_screenplay_scene_set_duration_msec(foreign, 1234);
        retval = sb_screenplay_scene_set_duration_sec(foreign, durations_sec[j]);
        j_begin_object();
        j_kfloat("duration_sec", durations_sec[j]);
        j_kerror("result", retval);
        j_key("resulting_duration_ms");
        if (sb_screenplay_scene_is_infinite(foreign)) {
            j_null();
        } else {
            j_int((long)sb_screenplay_scene_get_duration_msec(foreign));
        }
        j_end_object();
    }
    j_end_array();

    /* The screenplay of the foreign scene is not needed any more; it is
     * reused below for loading show files */
    sb_screenplay_destroy(&other);

    /* Loading a screenplay from a show file in memory */
    trajectory = build_trajectory((sb_vector3_with_yaw_t)P(0, 0, 0), (sb_vector3_with_yaw_t)P(0, 0, 1000), 2000);
    show_data = make_show_file(trajectory, &show_length);
    SB_XDECREF(trajectory);
    {
        const struct {
            const char* name;
            size_t length;
        } inputs[] = {
            { "empty", 0 },
            { "one_byte", 1 },
            { "header_only", 5 },
            { "truncated_block", show_length - 1 },
            { "valid", show_length },
        };

        j_key("update_from_binary_file_in_memory");
        j_begin_array();
        for (j = 0; j < sizeof(inputs) / sizeof(inputs[0]); j++) {
            CHECK(sb_screenplay_init(&other));
            retval = sb_screenplay_update_from_binary_file_in_memory(&other, show_data, inputs[j].length);
            j_begin_object();
            j_kstring("input", inputs[j].name);
            j_key("show_hex");
            j_hex(show_data, inputs[j].length);
            j_kerror("result", retval);
            j_kint("num_scenes", (long)sb_screenplay_size(&other));
            if (sb_screenplay_size(&other) > 0) {
                scene = sb_screenplay_get_scene_ptr(&other, 0);
                j_key("duration_ms");
                if (sb_screenplay_scene_is_infinite(scene)) {
                    j_null();
                } else {
                    j_int((long)sb_screenplay_scene_get_duration_msec(scene));
                }
            }
            j_end_object();
            sb_screenplay_destroy(&other);
        }
        j_end_array();
    }
    free(show_data);

    j_end_object();
    sb_screenplay_destroy(&screenplay);
}

/* ************************************************************************** */
/* Editing time axes                                                         */
/* ************************************************************************** */

typedef enum {
    TA_END = 0,
    TA_APPEND,
    TA_INSERT,
    TA_REMOVE,
    TA_SET_ORIGIN_SEC,
} time_axis_op_type_t;

typedef struct {
    time_axis_op_type_t type;
    size_t index;
    uint32_t duration_msec; /* ignored if warped_duration_sec is not NaN */
    float warped_duration_sec;
    float initial_rate;
    float final_rate;
    float origin_sec;
} time_axis_op_t;

#define TA_SEG(op, index, ms, r0, r1) \
    {                                 \
        op, index, ms, NAN, r0, r1, 0 \
    }
#define TA_WARPED(op, index, sec, r0, r1) \
    {                                     \
        op, index, 0, sec, r0, r1, 0      \
    }

static const struct {
    const char* name;
    time_axis_op_t ops[16];
} TIME_AXIS_EDITING_SCENARIOS[] = {
    { "insert_and_remove", {
                               TA_SEG(TA_APPEND, 0, 1000, 1, 1), TA_SEG(TA_APPEND, 0, 1000, 2, 2), TA_SEG(TA_INSERT, 0, 2000, 1, 0), TA_SEG(TA_INSERT, 3, 500, 1, 1), /* at the end */
                               TA_SEG(TA_INSERT, 10, 500, 1, 1), /* out of range */
                               TA_SEG(TA_REMOVE, 1, 0, 0, 0), TA_SEG(TA_REMOVE, 5, 0, 0, 0), /* out of range */
                               TA_SEG(TA_REMOVE, 3, 0, 0, 0), /* exactly the number of segments */
                               TA_SEG(TA_REMOVE, 2, 0, 0, 0), /* the last one */
                               TA_SEG(TA_INSERT, 1, 700, 1, 1), /* moves exactly one segment */
                               TA_SEG(TA_REMOVE, 0, 0, 0, 0), /* moves exactly two segments */
                               TA_SEG(TA_REMOVE, 0, 0, 0, 0), /* moves exactly one segment */
                           } },
    { "suspend_and_resume", {
                                TA_SEG(TA_APPEND, 0, 3000, 1, 1), TA_SEG(TA_APPEND, 0, 1000, 1, 0), /* slow down */
                                TA_SEG(TA_APPEND, 0, 4000, 0, 0), /* suspended */
                                TA_SEG(TA_APPEND, 0, 1000, 0, 1), /* speed up */
                                TA_SEG(TA_APPEND, 0, 5000, 1, 1), TA_SEG(TA_REMOVE, 2, 0, 0, 0), /* resume earlier */
                            } },
    { "many_segments", {
                           TA_SEG(TA_APPEND, 0, 500, 1, 2),
                           TA_SEG(TA_APPEND, 0, 500, 2, 1),
                           TA_SEG(TA_APPEND, 0, 500, 1, 0.5f),
                           TA_SEG(TA_APPEND, 0, 500, 0.5f, 1),
                           TA_SEG(TA_APPEND, 0, 500, 1, 1),
                           TA_SEG(TA_APPEND, 0, 500, 1, 3),
                           TA_SEG(TA_APPEND, 0, 500, 3, 3),
                           TA_SEG(TA_APPEND, 0, 500, 3, 1),
                           TA_SEG(TA_APPEND, 0, 500, 1, 1),
                           TA_SEG(TA_INSERT, 4, 500, 1, 0),
                       } },
    { "origin_in_seconds", {
                               TA_SEG(TA_APPEND, 0, 2000, 1, 1),
                               { TA_SET_ORIGIN_SEC, 0, 0, NAN, 0, 0, 1.2345f },
                               { TA_SET_ORIGIN_SEC, 0, 0, NAN, 0, 0, -2.5f },
                               { TA_SET_ORIGIN_SEC, 0, 0, NAN, 0, 0, NAN },
                               { TA_SET_ORIGIN_SEC, 0, 0, NAN, 0, 0, 3000000.0f },
                               { TA_SET_ORIGIN_SEC, 0, 0, NAN, 0, 0, 0.5f },
                           } },
    { "warped_segments", {
                             TA_WARPED(TA_APPEND, 0, 0, 1, 1),
                             TA_WARPED(TA_APPEND, 0, 2, 1, 1),
                             TA_WARPED(TA_APPEND, 0, 2, 0.5f, 0.5f),
                             TA_WARPED(TA_APPEND, 0, 2, 0, 1),
                             TA_WARPED(TA_APPEND, 0, 2, 0, 0),
                             TA_WARPED(TA_APPEND, 0, -1, 1, 1),
                             TA_WARPED(TA_APPEND, 0, 2, NAN, 1),
                             TA_WARPED(TA_APPEND, 0, 2, 1, -1),
                         } },
    { "infinite_last_segment", {
                                   TA_SEG(TA_APPEND, 0, 1000, 1, 1),
                                   TA_SEG(TA_APPEND, 0, UINT32_MAX, 1, 1),
                               } },
};

static void j_time_axis_editing_scenarios(void)
{
    const time_axis_op_t* op;
    sb_time_segment_t segment;
    sb_time_axis_t axis;
    sb_error_t retval;
    size_t i, j;

    j_key("time_axis_editing");
    j_begin_array();
    for (i = 0; i < sizeof(TIME_AXIS_EDITING_SCENARIOS) / sizeof(TIME_AXIS_EDITING_SCENARIOS[0]); i++) {
        CHECK(sb_time_axis_init(&axis));
        j_begin_object();
        j_kstring("scenario", TIME_AXIS_EDITING_SCENARIOS[i].name);
        j_key("ops");
        j_begin_array();
        for (j = 0; j < 16 && TIME_AXIS_EDITING_SCENARIOS[i].ops[j].type != TA_END; j++) {
            op = &TIME_AXIS_EDITING_SCENARIOS[i].ops[j];
            j_begin_object();
            if (op->type == TA_SET_ORIGIN_SEC) {
                j_kstring("op", "set_origin_sec");
                j_kfloat("origin_sec", op->origin_sec);
                retval = sb_time_axis_set_origin_sec(&axis, op->origin_sec);
            } else if (op->type == TA_REMOVE) {
                j_kstring("op", "remove_segment_at");
                j_kint("index", (long)op->index);
                retval = sb_time_axis_remove_segment_at(&axis, op->index);
            } else {
                if (isnan(op->warped_duration_sec)) {
                    segment = sb_time_segment_make(op->duration_msec, op->initial_rate, op->final_rate);
                    j_kint("duration_ms", op->duration_msec == UINT32_MAX ? -1 : (long)op->duration_msec);
                } else {
                    segment = sb_time_segment_make_warped(op->warped_duration_sec, op->initial_rate, op->final_rate);
                    j_kfloat("warped_duration_sec", op->warped_duration_sec);
                }
                j_kfloat("initial_rate", op->initial_rate);
                j_kfloat("final_rate", op->final_rate);
                if (op->type == TA_INSERT) {
                    j_kstring("op", "insert_segment_at");
                    j_kint("index", (long)op->index);
                    retval = sb_time_axis_insert_segment_at(&axis, op->index, segment);
                } else {
                    j_kstring("op", "append_segment");
                    retval = sb_time_axis_append_segment(&axis, segment);
                }
                j_kint("made_duration_ms", segment.duration_msec == UINT32_MAX ? -1 : (long)segment.duration_msec);
            }
            j_kerror("result", retval);
            j_kint("origin_ms", (long)sb_time_axis_get_origin_msec(&axis));
            j_end_object();
        }
        j_end_array();
        j_time_axis_mapping(&axis);
        j_end_object();
        sb_time_axis_destroy(&axis);
    }
    j_end_array();
}

/* ************************************************************************** */
/* Statistics of synthetic trajectories                                      */
/* ************************************************************************** */

static const builder_scenario_t STATS_TRAJECTORIES[] = {
    { "descent_at_verticality_threshold", 1, {
                                                 { OP_SET_START, P(0, 0, 0) },
                                                 { OP_LINE, P(0, 0, 5000), P(0, 0, 0), P(0, 0, 0), 5000 },
                                                 { OP_HOLD, P(0, 0, 0), P(0, 0, 0), P(0, 0, 0), 2000 },
                                                 { OP_LINE, P(50, 50, 0), P(0, 0, 0), P(0, 0, 0), 5000 },
                                             } },
    { "descent_just_over_verticality_threshold", 1, {
                                                        { OP_SET_START, P(0, 0, 0) },
                                                        { OP_LINE, P(0, 0, 5000), P(0, 0, 0), P(0, 0, 0), 5000 },
                                                        { OP_HOLD, P(0, 0, 0), P(0, 0, 0), P(0, 0, 0), 2000 },
                                                        { OP_LINE, P(51, 0, 0), P(0, 0, 0), P(0, 0, 0), 5000 },
                                                    } },
    { "ascending_at_the_end", 1, {
                                     { OP_SET_START, P(0, 0, 0) },
                                     { OP_LINE, P(0, 0, 5000), P(0, 0, 0), P(0, 0, 0), 5000 },
                                     { OP_LINE, P(0, 0, 2000), P(0, 0, 0), P(0, 0, 0), 3000 },
                                     { OP_LINE, P(0, 0, 3000), P(0, 0, 0), P(0, 0, 0), 2000 },
                                 } },
    { "descent_in_multiple_segments", 1, {
                                             { OP_SET_START, P(0, 0, 0) },
                                             { OP_LINE, P(0, 0, 5000), P(0, 0, 0), P(0, 0, 0), 5000 },
                                             { OP_LINE, P(0, 0, 3000), P(0, 0, 0), P(0, 0, 0), 2000 },
                                             { OP_HOLD, P(0, 0, 0), P(0, 0, 0), P(0, 0, 0), 1000 },
                                             { OP_LINE, P(0, 0, 1000), P(0, 0, 0), P(0, 0, 0), 2000 },
                                             { OP_LINE, P(0, 0, 0), P(0, 0, 0), P(0, 0, 0), 1000 },
                                         } },
    { "descent_equal_to_preferred_descent", 1, {
                                                   { OP_SET_START, P(0, 0, 0) },
                                                   { OP_LINE, P(0, 0, 5000), P(0, 0, 0), P(0, 0, 0), 5000 },
                                                   { OP_LINE, P(0, 0, 2500), P(0, 0, 0), P(0, 0, 0), 2000 },
                                                   { OP_LINE, P(0, 0, 0), P(0, 0, 0), P(0, 0, 0), 2000 },
                                               } },
    { "never_takes_off", 1, {
                                { OP_SET_START, P(0, 0, 0) },
                                { OP_HOLD, P(0, 0, 0), P(0, 0, 0), P(0, 0, 0), 10000 },
                            } },
    { "ends_in_the_air", 1, {
                                { OP_SET_START, P(0, 0, 0) },
                                { OP_LINE, P(1000, 0, 4000), P(0, 0, 0), P(0, 0, 0), 4000 },
                            } },
    { "empty", 1, { { OP_SET_START, P(0, 0, 0) } } },
};

static void j_stats_scenarios(void)
{
    static const stats_settings_t settings[] = {
        { "default", SB_TRAJECTORY_STATS_ALL, 2000, 4000, 2500, 2500, 50 },
        { "short_preferred_descent", SB_TRAJECTORY_STATS_ALL, 2000, 4000, 2500, 1500, 50 },
        { "no_preferred_descent", SB_TRAJECTORY_STATS_ALL, 2000, 4000, 2500, 0, 50 },
        { "zero_takeoff_speed", SB_TRAJECTORY_STATS_ALL, 0, 4000, 2500, 2500, 50 },
        { "infinite_takeoff_speed", SB_TRAJECTORY_STATS_ALL, INFINITY, 4000, 2500, 2500, 50 },
        { "small_acceleration", SB_TRAJECTORY_STATS_ALL, 2000, 0.5f, 2500, 2500, 50 },
        { "small_takeoff_speed", SB_TRAJECTORY_STATS_ALL, 0.5f, 4000, 2500, 2500, 50 },
    };
    sb_trajectory_builder_t builder;
    sb_trajectory_t trajectory;
    sb_trajectory_stats_calculator_t calc;
    sb_trajectory_stats_t stats;
    sb_error_t retval;
    size_t i, j;

    j_key("stats_scenarios");
    j_begin_array();
    for (i = 0; i < sizeof(STATS_TRAJECTORIES) / sizeof(STATS_TRAJECTORIES[0]); i++) {
        CHECK(sb_trajectory_builder_init(&builder, STATS_TRAJECTORIES[i].scale, 0));
        j_begin_object();
        j_kstring("scenario", STATS_TRAJECTORIES[i].name);
        j_key("ops");
        j_begin_array();
        for (j = 0; j < 8 && STATS_TRAJECTORIES[i].ops[j].type != OP_END; j++) {
            j_builder_op(&STATS_TRAJECTORIES[i].ops[j], &builder);
        }
        j_end_array();
        CHECK(sb_trajectory_init(&trajectory));
        CHECK(sb_trajectory_update_from_builder(&trajectory, &builder));
        sb_trajectory_builder_destroy(&builder);

        CHECK(sb_trajectory_stats_init(&stats));
        CHECK(sb_trajectory_stats_calculator_init(&calc, 1000));
        retval = sb_trajectory_stats_calculator_run(&calc, &trajectory, &stats);
        j_key("default_calculator");
        j_begin_object();
        j_kfloat("takeoff_speed_mm_s", calc.takeoff_speed);
        j_kfloat("acceleration_mm_s2", calc.acceleration);
        j_kfloat("min_ascent_mm", calc.min_ascent);
        j_kfloat("preferred_descent_mm", calc.preferred_descent);
        j_kfloat("verticality_threshold_mm", calc.verticality_threshold);
        j_kerror("result", retval);
        j_key("stats");
        j_stats(&stats);
        j_end_object();
        sb_trajectory_stats_calculator_destroy(&calc);
        sb_trajectory_stats_destroy(&stats);

        j_key("stats");
        j_begin_array();
        for (j = 0; j < sizeof(settings) / sizeof(settings[0]); j++) {
            CHECK(sb_trajectory_stats_init(&stats));
            CHECK(sb_trajectory_stats_calculator_init(&calc, 1000));
            calc.components = settings[j].components;
            calc.takeoff_speed = settings[j].takeoff_speed;
            calc.acceleration = settings[j].acceleration;
            calc.min_ascent = settings[j].min_ascent;
            calc.preferred_descent = settings[j].preferred_descent;
            calc.verticality_threshold = settings[j].verticality_threshold;
            retval = sb_trajectory_stats_calculator_run(&calc, &trajectory, &stats);
            sb_trajectory_stats_calculator_destroy(&calc);

            j_begin_object();
            j_kstring("settings", settings[j].name);
            j_kfloat("takeoff_speed_mm_s", settings[j].takeoff_speed);
            j_kfloat("acceleration_mm_s2", settings[j].acceleration);
            j_kfloat("min_ascent_mm", settings[j].min_ascent);
            j_kfloat("preferred_descent_mm", settings[j].preferred_descent);
            j_kfloat("verticality_threshold_mm", settings[j].verticality_threshold);
            j_kerror("result", retval);
            if (retval == SB_SUCCESS) {
                j_key("stats");
                j_stats(&stats);
            }
            j_end_object();
            sb_trajectory_stats_destroy(&stats);
        }
        j_end_array();
        j_end_object();
        SB_DECREF_STATIC(&trajectory);
    }
    j_end_array();
}

/* ************************************************************************** */
/* Yaw control                                                               */
/* ************************************************************************** */

static void j_yaw_control_scenarios(void)
{
    static const float yaws[] = { 0, 90.04f, 90.05f, -45.06f, 359.9f, 3276.7f, 3276.8f, -3276.8f, -3276.85f, -3300, 10000 };
    sb_yaw_control_t* ctrl;
    sb_yaw_player_t player;
    sb_error_t retval;
    float value;
    size_t i;

    j_key("constant_yaw");
    j_begin_array();
    for (i = 0; i < sizeof(yaws) / sizeof(yaws[0]); i++) {
        ctrl = sb_yaw_control_new();
        if (ctrl == 0) {
            fprintf(stderr, "out of memory\n");
            exit(1);
        }
        retval = sb_yaw_control_set_constant_yaw(ctrl, yaws[i]);
        j_begin_object();
        j_kfloat("yaw_deg", yaws[i]);
        j_kerror("result", retval);
        j_key("is_empty");
        j_bool(sb_yaw_control_is_empty(ctrl));
        CHECK(sb_yaw_player_init(&player, ctrl));
        CHECK(sb_yaw_player_get_yaw_at(&player, 0, &value));
        j_kfloat("yaw_at_start_deg", value);
        CHECK(sb_yaw_player_get_yaw_at(&player, 100, &value));
        j_kfloat("yaw_later_deg", value);
        CHECK(sb_yaw_player_get_yaw_rate_at(&player, 10, &value));
        j_kfloat("yaw_rate_deg_s", value);
        sb_yaw_player_destroy(&player);
        j_end_object();
        SB_XDECREF(ctrl);
    }
    j_end_array();
}

/* ************************************************************************** */

int main(int argc, char* argv[])
{
    char generator[64];

    if (argc != 2) {
        fprintf(stderr, "Usage: %s <output.json>\n", argv[0]);
        return 2;
    }

    out = fopen(argv[1], "w");
    if (out == 0) {
        perror(argv[1]);
        return 1;
    }

    snprintf(generator, sizeof(generator), "libskybrush %d.%d.%d",
        SKYBRUSH_VERSION_MAJOR, SKYBRUSH_VERSION_MINOR, SKYBRUSH_VERSION_PATCH);

    j_begin_object();
    j_kint("version", 1);
    j_kstring("generator", generator);
    j_kint("sample_interval_ms", SAMPLE_INTERVAL_MSEC);
    j_builder_scenarios();
    j_rth_scenarios();
    j_time_axis_scenarios();
    j_screenplay_scenarios();
    j_event_list_scenarios();
    j_light_program_scenarios();
    j_screenplay_queries();
    j_time_axis_editing_scenarios();
    j_stats_scenarios();
    j_yaw_control_scenarios();
    j_motion_helpers();
    j_color_helpers();
    j_end_object();
    fputc('\n', out);

    fclose(out);
    return 0;
}
