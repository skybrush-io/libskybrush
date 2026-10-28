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

static int json_depth;
static int json_count[JSON_MAX_DEPTH];
static int json_after_key;

static void json_indent(void)
{
    fputc('\n', out);
    for (int i = 0; i < json_depth; i++) {
        fputs("  ", out);
    }
}

/** Writes the separator before the next item of the current container */
static void json_before_item(void)
{
    if (json_after_key) {
        json_after_key = 0;
        return;
    }
    if (json_count[json_depth]++ > 0) {
        fputc(',', out);
        if (json_depth > JSON_BREAK_DEPTH) {
            fputc(' ', out);
        }
    }
    if (json_depth > 0 && json_depth <= JSON_BREAK_DEPTH) {
        json_indent();
    }
}

static void json_open(char bracket)
{
    json_before_item();
    fputc(bracket, out);
    json_depth++;
    json_count[json_depth] = 0;
}

static void json_close(char bracket)
{
    int had_items = json_count[json_depth] > 0;
    json_depth--;
    if (had_items && json_depth + 1 <= JSON_BREAK_DEPTH) {
        json_indent();
    }
    fputc(bracket, out);
}

#define json_begin_object() json_open('{')
#define json_end_object() json_close('}')
#define json_begin_array() json_open('[')
#define json_end_array() json_close(']')

static void json_key(const char* key)
{
    json_before_item();
    fprintf(out, "\"%s\": ", key);
    json_after_key = 1;
}

static void json_float(float value)
{
    json_before_item();
    print_float(value);
}

static void json_int(long value)
{
    json_before_item();
    fprintf(out, "%ld", value);
}

static void json_string(const char* value)
{
    json_before_item();
    fprintf(out, "\"%s\"", value);
}

static void json_bool(int value)
{
    json_before_item();
    fputs(value ? "true" : "false", out);
}

static void json_null(void)
{
    json_before_item();
    fputs("null", out);
}

static void json_error(sb_error_t error)
{
    json_before_item();
    print_error(error);
}

static void json_hex(const uint8_t* bytes, size_t length)
{
    json_before_item();
    print_hex(bytes, length);
}

static void json_vector3(sb_vector3_t vec)
{
    json_begin_array();
    json_float(vec.x);
    json_float(vec.y);
    json_float(vec.z);
    json_end_array();
}

/** Writes the X, Y and Z coordinates of a vector, ignoring its yaw */
static void json_vector3_xyz(sb_vector3_with_yaw_t vec)
{
    json_begin_array();
    json_float(vec.x);
    json_float(vec.y);
    json_float(vec.z);
    json_end_array();
}

static void json_vector3_with_yaw(sb_vector3_with_yaw_t vec)
{
    json_begin_array();
    json_float(vec.x);
    json_float(vec.y);
    json_float(vec.z);
    json_float(vec.yaw);
    json_end_array();
}

static void json_color(sb_rgb_color_t color)
{
    json_begin_array();
    json_int(color.red);
    json_int(color.green);
    json_int(color.blue);
    json_end_array();
}

static void json_rgbw_color(sb_rgbw_color_t color)
{
    json_begin_array();
    json_int(color.red);
    json_int(color.green);
    json_int(color.blue);
    json_int(color.white);
    json_end_array();
}

#define json_kfloat(key, value) (json_key(key), json_float(value))
#define json_kint(key, value) (json_key(key), json_int(value))
#define json_kstring(key, value) (json_key(key), json_string(value))
#define json_kerror(key, value) (json_key(key), json_error(value))

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
static void json_trajectory_hex(sb_trajectory_t* trajectory)
{
    json_hex(SB_BUFFER(trajectory->buffer), sb_buffer_size(&trajectory->buffer));
}

/**
 * Records the segments of a trajectory, sampling each one at a few relative
 * times. Positions are recorded in the units of the scale of the trajectory
 * as the builder quantizes them to those units.
 */
static void json_trajectory_segments(sb_trajectory_t* trajectory)
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
    json_kint("scale", (long)trajectory->scale);
    json_kint("duration_ms", (long)sb_trajectory_get_total_duration_msec(trajectory));
    json_key("segments");
    json_begin_array();
    while (sb_trajectory_player_has_more_segments(&player)) {
        segment = sb_trajectory_player_get_current_segment(&player);
        json_begin_object();
        json_kint("start_ms", (long)segment->start_time_msec);
        json_kint("duration_ms", (long)segment->duration_msec);
        json_key("position_units");
        json_begin_array();
        for (i = 0; i < sizeof(SEGMENT_SAMPLES) / sizeof(SEGMENT_SAMPLES[0]); i++) {
            CHECK(sb_trajectory_player_get_position_at(&sampler,
                segment->start_time_sec + SEGMENT_SAMPLES[i] * segment->duration_sec, &pos));
            json_begin_array();
            json_float(pos.x / scale);
            json_float(pos.y / scale);
            json_float(pos.z / scale);
            json_end_array();
        }
        json_end_array();
        json_end_object();
        CHECK(sb_trajectory_player_build_next_segment(&player));
    }
    json_end_array();
    sb_trajectory_player_destroy(&sampler);
    sb_trajectory_player_destroy(&player);
}

/** Records the position and the velocity along a trajectory at regular intervals */
static void json_trajectory_samples(sb_trajectory_t* trajectory, uint32_t from_msec, uint32_t to_msec)
{
    sb_trajectory_player_t player;
    sb_vector3_with_yaw_t vec;
    uint32_t t;

    from_msec -= from_msec % SAMPLE_INTERVAL_MSEC;

    CHECK(sb_trajectory_player_init(&player, trajectory));
    json_kint("samples_from_ms", (long)from_msec);
    json_key("position_mm");
    json_begin_array();
    for (t = from_msec; t <= to_msec; t += SAMPLE_INTERVAL_MSEC) {
        CHECK(sb_trajectory_player_get_position_at(&player, t / 1000.0f, &vec));
        json_vector3_xyz(vec);
    }
    json_end_array();
    json_key("velocity_mm_s");
    json_begin_array();
    for (t = from_msec; t <= to_msec; t += SAMPLE_INTERVAL_MSEC) {
        CHECK(sb_trajectory_player_get_velocity_at(&player, t / 1000.0f, &vec));
        json_vector3_xyz(vec);
    }
    json_end_array();
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

static void json_builder_op(const builder_op_t* op, sb_trajectory_builder_t* builder)
{
    sb_error_t retval = SB_SUCCESS;

    json_begin_object();
    switch (op->type) {
    case OP_SET_START:
        json_kstring("op", "set_start_position");
        json_key("position_mm");
        json_vector3_with_yaw(op->target);
        retval = sb_trajectory_builder_set_start_position(builder, op->target);
        break;
    case OP_LINE:
        json_kstring("op", "append_line");
        json_key("target_mm");
        json_vector3_with_yaw(op->target);
        json_kint("duration_ms", (long)op->duration_msec);
        retval = sb_trajectory_builder_append_line(builder, op->target, op->duration_msec);
        break;
    case OP_BEZIER:
        json_kstring("op", "append_cubic_bezier");
        json_key("control1_mm");
        json_vector3_with_yaw(op->control1);
        json_key("control2_mm");
        json_vector3_with_yaw(op->control2);
        json_key("target_mm");
        json_vector3_with_yaw(op->target);
        json_kint("duration_ms", (long)op->duration_msec);
        retval = sb_trajectory_builder_append_cubic_bezier(builder, op->control1, op->control2, op->target, op->duration_msec);
        break;
    case OP_HOLD:
        json_kstring("op", "hold_position_for");
        json_kint("duration_ms", (long)op->duration_msec);
        retval = sb_trajectory_builder_hold_position_for(builder, op->duration_msec);
        break;
    case OP_MOVE_TO_IN_TIME:
        json_kstring("op", "move_to_in_time");
        json_key("target_mm");
        json_vector3_with_yaw(op->target);
        json_kint("duration_ms", (long)op->duration_msec);
        json_kfloat("max_acceleration_mm_s2", op->max_acceleration);
        retval = sb_trajectory_builder_move_to_in_time(builder, op->target, op->duration_msec, op->max_acceleration);
        break;
    default:
        break;
    }
    json_kerror("result", retval);
    json_end_object();
}

static void json_builder_scenarios(void)
{
    sb_trajectory_builder_t builder;
    sb_trajectory_t trajectory;
    const builder_scenario_t* scenario;
    sb_error_t retval;
    size_t i, j;

    json_key("builder");
    json_begin_array();
    for (i = 0; i < sizeof(BUILDER_SCENARIOS) / sizeof(BUILDER_SCENARIOS[0]); i++) {
        scenario = &BUILDER_SCENARIOS[i];
        json_begin_object();
        json_kstring("scenario", scenario->name);
        json_kint("scale", scenario->scale);

        retval = sb_trajectory_builder_init(&builder, scenario->scale, 0);
        json_kerror("init_result", retval);
        if (retval == SB_SUCCESS) {
            json_key("ops");
            json_begin_array();
            for (j = 0; j < sizeof(scenario->ops) / sizeof(scenario->ops[0]) && scenario->ops[j].type != OP_END; j++) {
                json_builder_op(&scenario->ops[j], &builder);
            }
            json_end_array();

            CHECK(sb_trajectory_init(&trajectory));
            CHECK(sb_trajectory_update_from_builder(&trajectory, &builder));
            json_key("trajectory");
            json_begin_object();
            json_key("is_empty");
            json_bool(sb_trajectory_is_empty(&trajectory));
            {
                sb_vector3_with_yaw_t start_position;
                CHECK(sb_trajectory_get_start_position(&trajectory, &start_position));
                json_key("start_position_mm");
                json_vector3_xyz(start_position);
            }
            json_trajectory_segments(&trajectory);
            json_end_object();
            SB_DECREF_STATIC(&trajectory);
            sb_trajectory_builder_destroy(&builder);
        }
        json_end_object();
    }
    json_end_array();
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

static void json_rth_scenarios(void)
{
    sb_vector3_t start = { 1000, 2000, 10000 };
    const rth_scenario_t* scenario;
    sb_rth_plan_entry_t entry;
    sb_trajectory_t trajectory;
    sb_error_t retval;
    size_t i;

    json_key("rth_conversions");
    json_begin_array();
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

        json_begin_object();
        json_kstring("scenario", scenario->name);
        json_key("start_mm");
        json_vector3(start);
        json_key("entry");
        json_begin_object();
        json_kfloat("time_sec", entry.time_sec);
        json_kint("flags", (long)entry.flags);
        json_key("landing_target_mm");
        json_begin_array();
        json_float(entry.landing_target.x);
        json_float(entry.landing_target.y);
        json_end_array();
        json_kfloat("arrival_altitude_mm", entry.arrival_altitude);
        json_kfloat("duration_sec", entry.duration_sec);
        json_kfloat("pre_delay_sec", entry.pre_delay_sec);
        json_kfloat("post_delay_sec", entry.post_delay_sec);
        json_kfloat("pre_neck_mm", entry.pre_neck);
        json_kfloat("pre_neck_duration_sec", entry.pre_neck_duration_sec);
        json_kfloat("max_acceleration_mm_s2", entry.max_acceleration);
        json_kfloat("landing_velocity_mm_s", entry.landing_velocity);
        json_kfloat("landing_altitude_mm", entry.landing_altitude);
        json_end_object();

        CHECK(sb_trajectory_init(&trajectory));
        retval = sb_trajectory_update_from_rth_plan_entry(&trajectory, &entry, start);
        json_kerror("result", retval);
        if (retval == SB_SUCCESS) {
            json_kint("duration_ms", (long)sb_trajectory_get_total_duration_msec(&trajectory));
            json_trajectory_samples(&trajectory,
                entry.time_sec > 0 ? (uint32_t)(entry.time_sec * 1000) : 0,
                sb_trajectory_get_total_duration_msec(&trajectory) + MARGIN_MSEC);
        }
        json_end_object();
        SB_DECREF_STATIC(&trajectory);
    }
    json_end_array();

    /* An empty RTH plan */
    {
        sb_rth_plan_t plan;

        CHECK(sb_rth_plan_init(&plan));
        json_key("empty_rth_plan");
        json_begin_object();
        json_key("is_empty");
        json_bool(sb_rth_plan_is_empty(&plan));
        json_kint("num_entries", (long)sb_rth_plan_get_num_entries(&plan));
        json_kint("num_points", (long)sb_rth_plan_get_num_points(&plan));
        json_kerror("evaluate_at_result", sb_rth_plan_evaluate_at(&plan, 5, &entry));
        json_kfloat("default_landing_altitude_mm", sb_rth_plan_get_default_landing_altitude(&plan));
        json_end_object();
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
static void json_time_axis_mapping(const sb_time_axis_t* axis)
{
    const sb_time_segment_t* segment;
    int32_t origin_msec = sb_time_axis_get_origin_msec(axis), t, end_msec;
    float warped, rate, total_warped = sb_time_axis_get_total_warped_duration_sec(axis);
    uint32_t duration_msec = sb_time_axis_get_total_duration_msec(axis);
    size_t j;

    json_kint("num_segments", (long)sb_time_axis_num_segments(axis));
    json_key("resulting_segments");
    json_begin_array();
    for (j = 0; j < sb_time_axis_num_segments(axis); j++) {
        segment = sb_time_axis_get_segment(axis, j);
        json_begin_array();
        json_int((long)segment->duration_msec);
        json_float(segment->initial_rate);
        json_float(segment->final_rate);
        json_end_array();
    }
    json_end_array();
    json_kint("total_duration_ms", duration_msec == UINT32_MAX ? -1 : (long)duration_msec);
    json_kfloat("total_duration_sec", sb_time_axis_get_total_duration_sec(axis));
    json_kfloat("total_warped_duration_sec", total_warped);

    /* Infinite axes are sampled for a while after their origin only */
    end_msec = origin_msec + (duration_msec < 60000 ? (int32_t)duration_msec : 60000) + MARGIN_MSEC;
    json_kint("map_from_ms", (long)(origin_msec - MARGIN_MSEC));
    json_key("map_sec");
    json_begin_array();
    for (t = origin_msec - MARGIN_MSEC; t <= end_msec; t += SAMPLE_INTERVAL_MSEC) {
        json_float(sb_time_axis_map(axis, t));
    }
    json_end_array();
    json_key("map_rate");
    json_begin_array();
    for (t = origin_msec - MARGIN_MSEC; t <= end_msec; t += SAMPLE_INTERVAL_MSEC) {
        sb_time_axis_map_ex(axis, t, &rate);
        json_float(rate);
    }
    json_end_array();

    /* Warped time -> wall clock time */
    json_key("reverse_map");
    json_begin_array();
    for (j = 0; j <= 20; j++) {
        warped = (isfinite(total_warped) && total_warped > 0 ? total_warped : 10) * (j / 20.0f) * 1.2f - 1;
        json_begin_array();
        json_float(warped);
        json_int((long)sb_time_axis_reverse_map(axis, warped));
        json_end_array();
    }
    json_end_array();
}

static void json_time_axis_scenarios(void)
{
    const time_axis_scenario_t* scenario;
    const time_segment_spec_t* spec;
    sb_time_segment_t segment;
    sb_time_axis_t axis;
    size_t i, j;

    json_key("time_axes");
    json_begin_array();
    for (i = 0; i < sizeof(TIME_AXIS_SCENARIOS) / sizeof(TIME_AXIS_SCENARIOS[0]); i++) {
        scenario = &TIME_AXIS_SCENARIOS[i];
        CHECK(sb_time_axis_init(&axis));

        json_begin_object();
        json_kstring("scenario", scenario->name);
        json_kint("origin_ms", (long)scenario->origin_msec);
        /* Negative origins are rejected */
        json_kerror("set_origin_result", sb_time_axis_set_origin_msec(&axis, scenario->origin_msec));
        json_kint("resulting_origin_ms", (long)sb_time_axis_get_origin_msec(&axis));
        json_key("segments");
        json_begin_array();
        for (j = 0; j < scenario->num_segments; j++) {
            spec = &scenario->segments[j];
            json_begin_object();
            if (isnan(spec->warped_duration_sec)) {
                segment = sb_time_segment_make(spec->duration_msec, spec->initial_rate, spec->final_rate);
                json_kint("duration_ms", (long)spec->duration_msec);
            } else {
                segment = sb_time_segment_make_warped(spec->warped_duration_sec, spec->initial_rate, spec->final_rate);
                json_kfloat("warped_duration_sec", spec->warped_duration_sec);
            }
            json_kfloat("initial_rate", spec->initial_rate);
            json_kfloat("final_rate", spec->final_rate);
            json_kerror("append_result", sb_time_axis_append_segment(&axis, segment));
            json_end_object();
        }
        json_end_array();

        json_time_axis_mapping(&axis);
        json_end_object();

        sb_time_axis_destroy(&axis);
    }
    json_end_array();
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
static int json_output_components(const sb_control_output_t* output)
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

static void json_screenplay_scenarios(void)
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

    json_key("screenplays");
    json_begin_array();
    for (i = 0; i < sizeof(SCREENPLAY_SCENARIOS) / sizeof(SCREENPLAY_SCENARIOS[0]); i++) {
        scenario = &SCREENPLAY_SCENARIOS[i];
        CHECK(sb_screenplay_init(&screenplay));

        json_begin_object();
        json_kstring("scenario", scenario->name);
        json_key("scenes");
        json_begin_array();
        for (j = 0; j < scenario->num_scenes; j++) {
            spec = &scenario->scenes[j];
            CHECK(sb_screenplay_append_new_scene(&screenplay, &scene));

            json_begin_object();
            if (spec->duration_msec < 0) {
                sb_screenplay_scene_set_infinite(scene);
                json_key("duration_ms");
                json_null();
            } else {
                sb_screenplay_scene_set_duration_msec(scene, (uint32_t)spec->duration_msec);
                json_kint("duration_ms", (long)spec->duration_msec);
            }
            json_kint("origin_ms", (long)spec->origin_msec);
            json_kerror("set_origin_result", sb_screenplay_scene_set_origin_msec(scene, spec->origin_msec));

            trajectory = build_trajectory(spec->start, spec->end, spec->move_msec);
            sb_screenplay_scene_set_trajectory(scene, trajectory);
            json_key("trajectory_hex");
            json_trajectory_hex(trajectory);
            SB_XDECREF(trajectory);

            program = make_light_program(spec->light_program, spec->light_program_size);
            sb_screenplay_scene_set_light_program(scene, program);
            json_key("light_program_hex");
            json_hex(spec->light_program, spec->light_program_size);
            SB_XDECREF(program);

            json_key("yaw_deg");
            if (isnan(spec->yaw_deg)) {
                json_null();
            } else {
                yaw = sb_yaw_control_new();
                if (yaw == 0) {
                    fprintf(stderr, "out of memory\n");
                    exit(1);
                }
                CHECK(sb_yaw_control_set_constant_yaw(yaw, spec->yaw_deg));
                sb_screenplay_scene_set_yaw_control(scene, yaw);
                SB_XDECREF(yaw);
                json_float(spec->yaw_deg);
            }

            json_key("time_axis");
            json_begin_array();
            for (k = 0; k < spec->num_time_segments; k++) {
                CHECK(sb_time_axis_append_segment(sb_screenplay_scene_get_time_axis(scene),
                    sb_time_segment_make(spec->time_axis[k].duration_msec, spec->time_axis[k].initial_rate, spec->time_axis[k].final_rate)));
                json_begin_object();
                json_kint("duration_ms", (long)spec->time_axis[k].duration_msec);
                json_kfloat("initial_rate", spec->time_axis[k].initial_rate);
                json_kfloat("final_rate", spec->time_axis[k].final_rate);
                json_end_object();
            }
            json_end_array();
            json_end_object();
        }
        json_end_array();

        CHECK(sb_show_controller_init(&ctrl, &screenplay));
        json_key("samples");
        json_begin_array();
        for (k = 0, t = 0; t <= scenario->sample_until_msec || k < 3; t += SAMPLE_INTERVAL_MSEC) {
            if (t > scenario->sample_until_msec) {
                /* A few extreme timestamps after the regular samples */
                static const uint32_t extremes[] = { INT32_MAX, (uint32_t)INT32_MAX + 1, UINT32_MAX };
                t = extremes[k++];
            }
            CHECK(sb_show_controller_update_time_msec(&ctrl, t));
            output = sb_show_controller_get_current_output(&ctrl);
            sb_screenplay_get_scene_ptr_at_time_msec(&screenplay, t, &scene_index);

            json_begin_object();
            json_kint("time_ms", (long)t);
            json_kint("scene", (long)scene_index);
            json_key("position_mm");
            if (sb_control_output_get_position_if_set(output, &vec)) {
                json_vector3(vec);
            } else {
                json_null();
            }
            json_key("velocity_mm_s");
            if (sb_control_output_get_velocity_if_set(output, &vec)) {
                json_vector3(vec);
            } else {
                json_null();
            }
            json_key("yaw_deg");
            if (sb_control_output_get_yaw_if_set(output, &value)) {
                json_float(value);
            } else {
                json_null();
            }
            json_key("yaw_rate_deg_s");
            if (sb_control_output_get_yaw_rate_if_set(output, &value)) {
                json_float(value);
            } else {
                json_null();
            }
            json_key("color");
            if (sb_control_output_get_color_if_set(output, &color)) {
                json_color(color);
            } else {
                json_null();
            }
            json_kint("components", (long)json_output_components(output));
            {
                sb_control_output_time_t output_time = sb_show_controller_get_current_output_time(&ctrl);
                json_kint("output_time_ms", (long)output_time.time_msec);
                json_kint("time_in_scene_ms", (long)output_time.time_in_scene_msec);
                json_kfloat("warped_time_in_scene_sec", output_time.warped_time_in_scene_sec);
            }
            json_key("ended");
            json_bool(sb_show_controller_has_reached_end(&ctrl));
            json_end_object();
            if (k >= 3) {
                break;
            }
        }
        json_end_array();
        json_end_object();

        sb_show_controller_destroy(&ctrl);
        sb_screenplay_destroy(&screenplay);
    }
    json_end_array();
}

/* ************************************************************************** */
/* Motion helpers                                                            */
/* ************************************************************************** */

static void json_motion_helpers(void)
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

    json_key("motion");
    json_begin_object();

    json_key("travel_time_for_distance");
    json_begin_array();
    for (i = 0; i < sizeof(travel_times) / sizeof(travel_times[0]); i++) {
        json_begin_object();
        json_kfloat("distance_mm", travel_times[i][0]);
        json_kfloat("speed_mm_s", travel_times[i][1]);
        json_kfloat("acceleration_mm_s2", travel_times[i][2]);
        json_kfloat("travel_time_sec", sb_get_travel_time_for_distance(travel_times[i][0], travel_times[i][1], travel_times[i][2]));
        json_end_object();
    }
    json_end_array();

    json_key("travel_velocity_for_distance");
    json_begin_array();
    for (i = 0; i < sizeof(travel_velocities) / sizeof(travel_velocities[0]); i++) {
        json_begin_object();
        json_kfloat("distance_mm", travel_velocities[i][0]);
        json_kfloat("time_sec", travel_velocities[i][1]);
        json_kfloat("acceleration_mm_s2", travel_velocities[i][2]);
        json_kfloat("velocity_mm_s", sb_get_travel_velocity_for_distance(travel_velocities[i][0], travel_velocities[i][1], travel_velocities[i][2]));
        json_end_object();
    }
    json_end_array();

    json_key("cubic_bezier_from_velocity_constraints");
    json_begin_array();
    for (i = 0; i < sizeof(beziers) / sizeof(beziers[0]); i++) {
        retval = sb_get_cubic_bezier_from_velocity_constraints(beziers[i].start, beziers[i].start_vel,
            beziers[i].end, beziers[i].end_vel, beziers[i].duration_sec, &control1, &control2);
        json_begin_object();
        json_key("start_mm");
        json_vector3_with_yaw(beziers[i].start);
        json_key("start_velocity_mm_s");
        json_vector3_with_yaw(beziers[i].start_vel);
        json_key("end_mm");
        json_vector3_with_yaw(beziers[i].end);
        json_key("end_velocity_mm_s");
        json_vector3_with_yaw(beziers[i].end_vel);
        json_kfloat("duration_sec", beziers[i].duration_sec);
        json_kerror("result", retval);
        if (retval == SB_SUCCESS) {
            json_key("control1_mm");
            json_vector3_with_yaw(control1);
            json_key("control2_mm");
            json_vector3_with_yaw(control2);
        }
        json_end_object();
    }
    json_end_array();

    json_end_object();
}

/* ************************************************************************** */
/* Color helpers                                                             */
/* ************************************************************************** */

static void json_color_helpers(void)
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

    json_key("colors");
    json_begin_object();

    /* powf() and logf() may differ between math libraries by an amount that
     * changes the truncated integer value, hence the _approx8 suffix */
    json_key("from_color_temperature");
    json_begin_array();
    for (i = 0; i < sizeof(temperatures) / sizeof(temperatures[0]); i++) {
        json_begin_object();
        json_kfloat("temperature_k", temperatures[i]);
        json_key("color_approx8");
        json_color(sb_rgb_color_from_color_temperature(temperatures[i]));
        json_end_object();
    }
    json_end_array();

    json_key("rgb565");
    json_begin_array();
    for (i = 0; i < sizeof(colors) / sizeof(colors[0]); i++) {
        encoded = sb_rgb_color_encode_rgb565(colors[i]);
        json_begin_object();
        json_key("color");
        json_color(colors[i]);
        json_kint("encoded", encoded);
        json_key("decoded");
        json_color(sb_rgb_color_decode_rgb565(encoded));
        json_end_object();
    }
    json_end_array();

    json_key("linear_interpolation");
    json_begin_array();
    for (i = 0; i < sizeof(ratios) / sizeof(ratios[0]); i++) {
        json_begin_object();
        json_key("first");
        json_color(colors[2]);
        json_key("second");
        json_color(colors[6]);
        json_kfloat("ratio", ratios[i]);
        json_key("result");
        json_color(sb_rgb_color_linear_interpolation(colors[2], colors[6], ratios[i]));
        json_end_object();
    }
    json_end_array();

    json_key("rgbw");
    json_begin_array();
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

        json_begin_object();
        json_kstring("conversion", name);
        json_key("results");
        json_begin_array();
        for (j = 0; j < sizeof(colors) / sizeof(colors[0]); j++) {
            json_begin_object();
            json_key("color");
            json_color(colors[j]);
            json_key(approximate ? "rgbw_approx8" : "rgbw");
            json_rgbw_color(sb_rgb_color_to_rgbw(colors[j], conv));
            json_end_object();
        }
        json_end_array();
        json_end_object();
    }
    json_end_array();

    json_end_object();
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
static void json_stats(const sb_trajectory_stats_t* stats)
{
    sb_trajectory_stat_components_t valid = stats->valid_components;

    json_begin_object();
    json_key("initial_position_mm");
    json_vector3_xyz(stats->initial_pos);
    if (valid & SB_TRAJECTORY_STATS_DURATION) {
        json_kint("duration_ms", (long)stats->duration_msec);
    }
    if (valid & SB_TRAJECTORY_STATS_TAKEOFF_TIME) {
        json_kfloat("earliest_above_sec", stats->earliest_above_sec);
        json_kfloat("takeoff_time_sec", stats->takeoff_time_sec);
    }
    if (valid & SB_TRAJECTORY_STATS_START_END_DISTANCE) {
        json_kfloat("start_to_end_distance_xy_mm", stats->start_to_end_distance_xy);
    }
    if (valid & SB_TRAJECTORY_STATS_LANDING_TIME) {
        json_kfloat("landing_time_sec", stats->landing_time_sec);
        json_key("position_at_landing_mm");
        json_vector3_xyz(stats->pos_at_landing_time);
        json_key("velocity_at_landing_mm_s");
        json_vector3_xyz(stats->vel_at_landing_time);
    }
    json_end_object();
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

static void json_event(const sb_event_t* event)
{
    json_begin_array();
    json_int((long)event->time_msec);
    json_int((long)event->type);
    json_int((long)event->subtype);
    json_hex(event->payload.as_buf, sizeof(event->payload.as_buf));
    json_end_array();
}

static void json_event_list(const sb_event_list_t* events)
{
    size_t i;

    json_key("events");
    json_begin_array();
    for (i = 0; i < sb_event_list_size(events); i++) {
        json_event(sb_event_list_get_ptr_const(events, i));
    }
    json_end_array();
    json_key("is_sorted");
    json_bool(sb_event_list_is_sorted(events));
    json_key("is_empty");
    json_bool(sb_event_list_is_empty(events));
}

static void json_event_index(const sb_event_list_t* events, const sb_event_t* event)
{
    if (event) {
        json_int((long)(event - sb_event_list_get_ptr_const(events, 0)));
    } else {
        json_null();
    }
}

static void json_event_list_scenarios(void)
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

    json_key("event_lists");
    json_begin_array();
    for (i = 0; i < sizeof(EVENT_LIST_SCENARIOS) / sizeof(EVENT_LIST_SCENARIOS[0]); i++) {
        scenario = &EVENT_LIST_SCENARIOS[i];
        CHECK(sb_event_list_init(&events, 2));

        json_begin_object();
        json_kstring("scenario", scenario->name);
        json_key("ops");
        json_begin_array();
        for (j = 0; j < sizeof(scenario->ops) / sizeof(scenario->ops[0]) && scenario->ops[j].type != EV_END; j++) {
            op = &scenario->ops[j];
            memset(&event, 0, sizeof(event));
            event.time_msec = op->time_msec;
            event.type = op->event_type;
            event.subtype = op->subtype;
            event.payload.as_uint32 = op->payload;

            json_begin_object();
            switch (op->type) {
            case EV_APPEND:
            case EV_INSERT:
                json_kstring("op", op->type == EV_APPEND ? "append" : "insert");
                json_key("event");
                json_event(&event);
                retval = op->type == EV_APPEND ? sb_event_list_append(&events, &event) : sb_event_list_insert(&events, &event);
                break;
            case EV_REMOVE:
                json_kstring("op", "remove");
                json_kint("index", op->value);
                retval = sb_event_list_remove(&events, (size_t)op->value);
                break;
            case EV_SORT:
                json_kstring("op", "sort");
                sb_event_list_sort(&events);
                retval = SB_SUCCESS;
                break;
            case EV_ADD_PYRO_OFF_EVENTS:
                json_kstring("op", "add_pyro_off_events");
                json_kint("duration_ms", (long)op->time_msec);
                retval = sb_event_list_add_pyro_off_events(&events, op->time_msec);
                break;
            default:
                json_kstring("op", "adjust_timestamps_by_type");
                json_kint("type", op->event_type);
                json_kint("delta_ms", op->value);
                sb_event_list_adjust_timestamps_by_type(&events, (sb_event_type_t)op->event_type, op->value);
                retval = SB_SUCCESS;
                break;
            }
            json_kerror("result", retval);
            json_event_list(&events);
            json_end_object();
        }
        json_end_array();

        /* Playing the resulting list; the player requires a sorted list */
        if (!sb_event_list_is_sorted(&events)) {
            sb_event_list_sort(&events);
        }
        CHECK(sb_event_list_player_init(&player, &events));
        json_key("sorted_for_playback");
        json_begin_object();
        json_event_list(&events);
        json_end_object();

        json_key("next_events");
        json_begin_array();
        while ((next = sb_event_list_player_get_next_event(&player)) != NULL) {
            json_event_index(&events, next);
        }
        json_end_array();

        json_key("query_times_sec");
        json_begin_array();
        for (j = 0; j < n; j++) {
            json_float(query_times_sec[j]);
        }
        json_end_array();

        /* Consecutive queries, as during playback */
        sb_event_list_player_rewind(&player);
        json_key("next_event_not_later_than");
        json_begin_array();
        for (j = 0; j < n; j++) {
            json_event_index(&events, sb_event_list_player_get_next_event_not_later_than(&player, query_times_sec[j]));
        }
        json_end_array();

        json_key("peek_after_seek");
        json_begin_array();
        for (j = 0; j < n; j++) {
            sb_event_list_player_seek(&player, query_times_sec[j]);
            json_event_index(&events, sb_event_list_player_peek_next_event(&player));
        }
        json_end_array();

        json_end_object();
        sb_event_list_player_destroy(&player);
        SB_DECREF_STATIC(&events);
    }
    json_end_array();
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

static void json_light_program_playback(sb_light_program_t* program)
{
    sb_light_player_t player;
    unsigned long t = 0, next = 0;
    size_t steps = 0;
    int ended = 0;
    uint32_t sample;

    json_key("bytecode_hex");
    json_hex(SB_BUFFER(program->buffer), sb_buffer_size(&program->buffer));

    CHECK(sb_light_player_init(&player, program));
    json_key("changes_ms");
    json_begin_array();
    while (steps < MAX_LIGHT_CHANGES) {
        ended = sb_light_player_seek(&player, t, &next);
        json_begin_array();
        json_int((long)t);
        json_int((long)next);
        json_end_array();
        steps++;
        if (ended || next <= t) {
            break;
        }
        t = next;
    }
    json_end_array();
    json_key("ended");
    json_bool(ended);

    json_key("color");
    json_begin_array();
    for (sample = 0; sample <= LIGHT_SAMPLES_UNTIL_MSEC; sample += SAMPLE_INTERVAL_MSEC) {
        json_color(sb_light_player_get_color_at(&player, sample));
    }
    json_end_array();
    sb_light_player_destroy(&player);
}

static void json_light_program_scenarios(void)
{
    static const sb_rgb_color_t constant_colors[] = { { 10, 20, 30 }, { 0, 0, 0 }, { 255, 255, 255 } };
    sb_light_program_t* program;
    size_t i;

    json_key("light_programs");
    json_begin_array();
    for (i = 0; i < sizeof(LIGHT_PROGRAM_SCENARIOS) / sizeof(LIGHT_PROGRAM_SCENARIOS[0]); i++) {
        json_begin_object();
        json_kstring("scenario", LIGHT_PROGRAM_SCENARIOS[i].name);
        program = make_light_program(LIGHT_PROGRAM_SCENARIOS[i].bytecode, LIGHT_PROGRAM_SCENARIOS[i].size);
        json_light_program_playback(program);
        SB_XDECREF(program);
        json_end_object();
    }
    for (i = 0; i < sizeof(constant_colors) / sizeof(constant_colors[0]); i++) {
        program = sb_light_program_new();
        if (program == 0) {
            fprintf(stderr, "out of memory\n");
            exit(1);
        }
        json_begin_object();
        json_kstring("scenario", "set_constant_color");
        json_key("constant_color");
        json_color(constant_colors[i]);
        json_kerror("result", sb_light_program_set_constant_color(program, constant_colors[i]));
        json_light_program_playback(program);
        json_end_object();
        SB_XDECREF(program);
    }
    json_end_array();
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

static void json_scene_index(sb_screenplay_t* screenplay, const sb_screenplay_scene_t* scene)
{
    size_t i;

    for (i = 0; i < sb_screenplay_size(screenplay); i++) {
        if (sb_screenplay_get_scene_ptr(screenplay, i) == scene) {
            json_int((long)i);
            return;
        }
    }
    json_null();
}

static void json_screenplay_queries(void)
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

    json_key("screenplay_queries");
    json_begin_object();

    json_key("scenes");
    json_begin_array();
    for (i = 0; i < 3; i++) {
        scene = scenes[i];
        json_begin_object();
        json_kint("tag", sb_screenplay_scene_get_tag(scene));
        json_kint("flags", sb_screenplay_scene_get_flags(scene));
        json_kint("origin_ms", (long)sb_screenplay_scene_get_origin_msec(scene));
        json_key("duration_ms");
        if (sb_screenplay_scene_is_infinite(scene)) {
            json_null();
        } else {
            json_int((long)sb_screenplay_scene_get_duration_msec(scene));
        }
        json_key("is_infinite");
        json_bool(sb_screenplay_scene_is_infinite(scene));
        json_key("has_any_flag");
        json_begin_array();
        for (j = 0; j < 4; j++) {
            json_bool(sb_screenplay_scene_has_any_flag(scene, (sb_screenplay_scene_flag_mask_t)j) != 0);
        }
        json_end_array();
        json_key("has_all_flags");
        json_begin_array();
        for (j = 0; j < 4; j++) {
            json_bool(sb_screenplay_scene_has_all_flags(scene, (sb_screenplay_scene_flag_mask_t)j) != 0);
        }
        json_end_array();
        retval = sb_screenplay_scene_get_uncovered_trajectory_duration_sec(scene, &value);
        json_kerror("uncovered_trajectory_duration_result", retval);
        json_kfloat("uncovered_trajectory_duration_sec", retval == SB_SUCCESS ? value : NAN);
        json_kfloat("warped_time_remaining_from_trajectory_sec",
            sb_screenplay_scene_get_warped_time_remaining_from_trajectory_at_end_of_time_axis(scene));
        json_key("contained");
        json_bool(sb_screenplay_contains_scene(&screenplay, scene));
        json_end_object();
    }
    json_end_array();

    json_key("contains_foreign_scene");
    json_bool(sb_screenplay_contains_scene(&screenplay, foreign));

    json_key("first_scene_with_tag");
    json_begin_array();
    for (tag = 0; tag < 4; tag++) {
        json_scene_index(&screenplay, sb_screenplay_find_first_scene_with_tag(&screenplay, tag));
    }
    json_end_array();

    /* Show clock time where a scene with a given tag reaches a given warped
     * time; UINT32_MAX (or infinity) means that it never does */
    json_key("warped_times_sec");
    json_begin_array();
    for (j = 0; j < sizeof(warped_times_sec) / sizeof(warped_times_sec[0]); j++) {
        json_float(warped_times_sec[j]);
    }
    json_end_array();
    json_key("time_for_tag_and_warped_time");
    json_begin_array();
    for (tag = 0; tag < 4; tag++) {
        json_begin_object();
        json_kint("tag", tag);
        json_key("time_ms");
        json_begin_array();
        for (j = 0; j < sizeof(warped_times_sec) / sizeof(warped_times_sec[0]); j++) {
            json_int((long)sb_screenplay_get_time_msec_for_scene_tag_and_warped_time_in_scene(&screenplay, tag, warped_times_sec[j]));
        }
        json_end_array();
        json_key("time_sec");
        json_begin_array();
        for (j = 0; j < sizeof(warped_times_sec) / sizeof(warped_times_sec[0]); j++) {
            json_float(sb_screenplay_get_time_sec_for_scene_tag_and_warped_time_in_scene(&screenplay, tag, warped_times_sec[j]));
        }
        json_end_array();
        json_end_object();
    }
    json_end_array();

    /* Flags of a scene */
    json_key("flag_operations");
    json_begin_array();
    sb_screenplay_scene_set_flags(foreign, 0x0f, 0x05);
    json_int(sb_screenplay_scene_get_flags(foreign));
    sb_screenplay_scene_set_flags(foreign, 0x03, 0x02);
    json_int(sb_screenplay_scene_get_flags(foreign));
    sb_screenplay_scene_clear_flag(foreign, 0x04);
    json_int(sb_screenplay_scene_get_flags(foreign));
    sb_screenplay_scene_set_flag(foreign, 0x80);
    json_int(sb_screenplay_scene_get_flags(foreign));
    json_end_array();

    /* Setting the duration of a scene in seconds */
    json_key("set_duration_sec");
    json_begin_array();
    for (j = 0; j < sizeof(durations_sec) / sizeof(durations_sec[0]); j++) {
        sb_screenplay_scene_set_duration_msec(foreign, 1234);
        retval = sb_screenplay_scene_set_duration_sec(foreign, durations_sec[j]);
        json_begin_object();
        json_kfloat("duration_sec", durations_sec[j]);
        json_kerror("result", retval);
        json_key("resulting_duration_ms");
        if (sb_screenplay_scene_is_infinite(foreign)) {
            json_null();
        } else {
            json_int((long)sb_screenplay_scene_get_duration_msec(foreign));
        }
        json_end_object();
    }
    json_end_array();

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

        json_key("update_from_binary_file_in_memory");
        json_begin_array();
        for (j = 0; j < sizeof(inputs) / sizeof(inputs[0]); j++) {
            CHECK(sb_screenplay_init(&other));
            retval = sb_screenplay_update_from_binary_file_in_memory(&other, show_data, inputs[j].length);
            json_begin_object();
            json_kstring("input", inputs[j].name);
            json_key("show_hex");
            json_hex(show_data, inputs[j].length);
            json_kerror("result", retval);
            json_kint("num_scenes", (long)sb_screenplay_size(&other));
            if (sb_screenplay_size(&other) > 0) {
                scene = sb_screenplay_get_scene_ptr(&other, 0);
                json_key("duration_ms");
                if (sb_screenplay_scene_is_infinite(scene)) {
                    json_null();
                } else {
                    json_int((long)sb_screenplay_scene_get_duration_msec(scene));
                }
            }
            json_end_object();
            sb_screenplay_destroy(&other);
        }
        json_end_array();
    }
    free(show_data);

    json_end_object();
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

static void json_time_axis_editing_scenarios(void)
{
    const time_axis_op_t* op;
    sb_time_segment_t segment;
    sb_time_axis_t axis;
    sb_error_t retval;
    size_t i, j;

    json_key("time_axis_editing");
    json_begin_array();
    for (i = 0; i < sizeof(TIME_AXIS_EDITING_SCENARIOS) / sizeof(TIME_AXIS_EDITING_SCENARIOS[0]); i++) {
        CHECK(sb_time_axis_init(&axis));
        json_begin_object();
        json_kstring("scenario", TIME_AXIS_EDITING_SCENARIOS[i].name);
        json_key("ops");
        json_begin_array();
        for (j = 0; j < 16 && TIME_AXIS_EDITING_SCENARIOS[i].ops[j].type != TA_END; j++) {
            op = &TIME_AXIS_EDITING_SCENARIOS[i].ops[j];
            json_begin_object();
            if (op->type == TA_SET_ORIGIN_SEC) {
                json_kstring("op", "set_origin_sec");
                json_kfloat("origin_sec", op->origin_sec);
                retval = sb_time_axis_set_origin_sec(&axis, op->origin_sec);
            } else if (op->type == TA_REMOVE) {
                json_kstring("op", "remove_segment_at");
                json_kint("index", (long)op->index);
                retval = sb_time_axis_remove_segment_at(&axis, op->index);
            } else {
                if (isnan(op->warped_duration_sec)) {
                    segment = sb_time_segment_make(op->duration_msec, op->initial_rate, op->final_rate);
                    json_kint("duration_ms", op->duration_msec == UINT32_MAX ? -1 : (long)op->duration_msec);
                } else {
                    segment = sb_time_segment_make_warped(op->warped_duration_sec, op->initial_rate, op->final_rate);
                    json_kfloat("warped_duration_sec", op->warped_duration_sec);
                }
                json_kfloat("initial_rate", op->initial_rate);
                json_kfloat("final_rate", op->final_rate);
                if (op->type == TA_INSERT) {
                    json_kstring("op", "insert_segment_at");
                    json_kint("index", (long)op->index);
                    retval = sb_time_axis_insert_segment_at(&axis, op->index, segment);
                } else {
                    json_kstring("op", "append_segment");
                    retval = sb_time_axis_append_segment(&axis, segment);
                }
                json_kint("made_duration_ms", segment.duration_msec == UINT32_MAX ? -1 : (long)segment.duration_msec);
            }
            json_kerror("result", retval);
            json_kint("origin_ms", (long)sb_time_axis_get_origin_msec(&axis));
            json_end_object();
        }
        json_end_array();
        json_time_axis_mapping(&axis);
        json_end_object();
        sb_time_axis_destroy(&axis);
    }
    json_end_array();
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

static void json_stats_scenarios(void)
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

    json_key("stats_scenarios");
    json_begin_array();
    for (i = 0; i < sizeof(STATS_TRAJECTORIES) / sizeof(STATS_TRAJECTORIES[0]); i++) {
        CHECK(sb_trajectory_builder_init(&builder, STATS_TRAJECTORIES[i].scale, 0));
        json_begin_object();
        json_kstring("scenario", STATS_TRAJECTORIES[i].name);
        json_key("ops");
        json_begin_array();
        for (j = 0; j < 8 && STATS_TRAJECTORIES[i].ops[j].type != OP_END; j++) {
            json_builder_op(&STATS_TRAJECTORIES[i].ops[j], &builder);
        }
        json_end_array();
        CHECK(sb_trajectory_init(&trajectory));
        CHECK(sb_trajectory_update_from_builder(&trajectory, &builder));
        sb_trajectory_builder_destroy(&builder);

        CHECK(sb_trajectory_stats_init(&stats));
        CHECK(sb_trajectory_stats_calculator_init(&calc, 1000));
        retval = sb_trajectory_stats_calculator_run(&calc, &trajectory, &stats);
        json_key("default_calculator");
        json_begin_object();
        json_kfloat("takeoff_speed_mm_s", calc.takeoff_speed);
        json_kfloat("acceleration_mm_s2", calc.acceleration);
        json_kfloat("min_ascent_mm", calc.min_ascent);
        json_kfloat("preferred_descent_mm", calc.preferred_descent);
        json_kfloat("verticality_threshold_mm", calc.verticality_threshold);
        json_kerror("result", retval);
        json_key("stats");
        json_stats(&stats);
        json_end_object();
        sb_trajectory_stats_calculator_destroy(&calc);
        sb_trajectory_stats_destroy(&stats);

        json_key("stats");
        json_begin_array();
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

            json_begin_object();
            json_kstring("settings", settings[j].name);
            json_kfloat("takeoff_speed_mm_s", settings[j].takeoff_speed);
            json_kfloat("acceleration_mm_s2", settings[j].acceleration);
            json_kfloat("min_ascent_mm", settings[j].min_ascent);
            json_kfloat("preferred_descent_mm", settings[j].preferred_descent);
            json_kfloat("verticality_threshold_mm", settings[j].verticality_threshold);
            json_kerror("result", retval);
            if (retval == SB_SUCCESS) {
                json_key("stats");
                json_stats(&stats);
            }
            json_end_object();
            sb_trajectory_stats_destroy(&stats);
        }
        json_end_array();
        json_end_object();
        SB_DECREF_STATIC(&trajectory);
    }
    json_end_array();
}

/* ************************************************************************** */
/* Yaw control                                                               */
/* ************************************************************************** */

static void json_yaw_control_scenarios(void)
{
    static const float yaws[] = { 0, 90.04f, 90.05f, -45.06f, 359.9f, 3276.7f, 3276.8f, -3276.8f, -3276.85f, -3300, 10000 };
    sb_yaw_control_t* ctrl;
    sb_yaw_player_t player;
    sb_error_t retval;
    float value;
    size_t i;

    json_key("constant_yaw");
    json_begin_array();
    for (i = 0; i < sizeof(yaws) / sizeof(yaws[0]); i++) {
        ctrl = sb_yaw_control_new();
        if (ctrl == 0) {
            fprintf(stderr, "out of memory\n");
            exit(1);
        }
        retval = sb_yaw_control_set_constant_yaw(ctrl, yaws[i]);
        json_begin_object();
        json_kfloat("yaw_deg", yaws[i]);
        json_kerror("result", retval);
        json_key("is_empty");
        json_bool(sb_yaw_control_is_empty(ctrl));
        CHECK(sb_yaw_player_init(&player, ctrl));
        CHECK(sb_yaw_player_get_yaw_at(&player, 0, &value));
        json_kfloat("yaw_at_start_deg", value);
        CHECK(sb_yaw_player_get_yaw_at(&player, 100, &value));
        json_kfloat("yaw_later_deg", value);
        CHECK(sb_yaw_player_get_yaw_rate_at(&player, 10, &value));
        json_kfloat("yaw_rate_deg_s", value);
        sb_yaw_player_destroy(&player);
        json_end_object();
        SB_XDECREF(ctrl);
    }
    json_end_array();
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

    json_begin_object();
    json_kint("version", 1);
    json_kstring("generator", generator);
    json_kint("sample_interval_ms", SAMPLE_INTERVAL_MSEC);
    json_builder_scenarios();
    json_rth_scenarios();
    json_time_axis_scenarios();
    json_screenplay_scenarios();
    json_event_list_scenarios();
    json_light_program_scenarios();
    json_screenplay_queries();
    json_time_axis_editing_scenarios();
    json_stats_scenarios();
    json_yaw_control_scenarios();
    json_motion_helpers();
    json_color_helpers();
    json_end_object();
    fputc('\n', out);

    fclose(out);
    return 0;
}
