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
 * \file test/api_traces/generate_api_trace.c
 * \brief Generates a golden trace of the API-level behaviour of the library
 * for a Skybrush binary show file.
 *
 * Unlike the traces of the file format, which record what can be read from a
 * show file, this trace records what the library computes from it: statistics
 * with various settings, proposed takeoff and landing times, cutting the
 * trajectory, replacing its end with a landing, converting RTH plan entries to
 * trajectories and the behaviour of the light and event players. The inputs
 * of each computation are recorded in the trace as well so that other
 * implementations of the library can be checked against it.
 *
 * Usage: generate_api_trace <input.skyb> <output.json>
 */

#include <math.h>
#include <string.h>

#include <skybrush/events.h>
#include <skybrush/lights.h>
#include <skybrush/refcount.h>
#include <skybrush/rth_plan.h>
#include <skybrush/stats.h>
#include <skybrush/trajectory.h>
#include <skybrush/version.h>

#include "../traces/trace_output.h"

/** Interval between consecutive samples of the trace */
#define SAMPLE_INTERVAL_MSEC 100

/** Extra time to sample after the end of the show */
#define MARGIN_MSEC 1000

/** Time to sample before and after the time where a trajectory is cut */
#define CUT_SAMPLING_WINDOW_MSEC 1000

/** Time to sample before the original landing time when the landing is replaced */
#define LANDING_SAMPLING_LEAD_MSEC 2000

/** Light programs that do not end within this time are treated as endless */
#define MAX_LIGHT_PROGRAM_DURATION_MSEC 3600000

/** Maximum number of changes recorded from the light player */
#define MAX_LIGHT_PROGRAM_CHANGES 100000

static uint8_t* show_data;
static size_t show_length;

/* ************************************************************************** */
/* Helpers                                                                   */
/* ************************************************************************** */

/**
 * Loads the trajectory of the show into a new trajectory object that owns its
 * data. Trajectories loaded directly from the in-memory show file are views
 * into the show file, and some operations (e.g., cutting) modify the
 * trajectory in place.
 */
static sb_trajectory_t* load_owned_trajectory(void)
{
    sb_trajectory_t view;
    sb_trajectory_t* result;
    uint8_t* copy;
    size_t size;

    CHECK(sb_trajectory_init(&view));
    CHECK(sb_trajectory_update_from_binary_file_in_memory(&view, show_data, show_length));

    size = sb_buffer_size(&view.buffer);
    copy = malloc(size > 0 ? size : 1);
    if (copy == 0) {
        fprintf(stderr, "out of memory\n");
        exit(1);
    }
    memcpy(copy, SB_BUFFER(view.buffer), size);
    SB_DECREF_STATIC(&view);

    result = sb_trajectory_new();
    if (result == 0) {
        fprintf(stderr, "out of memory\n");
        exit(1);
    }
    CHECK(sb_trajectory_update_from_bytes(result, copy, size));

    return result;
}

static uint32_t round_down_to_sample(uint32_t time_msec)
{
    return time_msec - time_msec % SAMPLE_INTERVAL_MSEC;
}

/**
 * Prints the position and the velocity along a trajectory at the sample
 * timestamps between the given start and end times, inclusive.
 */
static void print_trajectory_samples(sb_trajectory_t* trajectory, uint32_t from_msec, uint32_t to_msec, const char* indent)
{
    sb_trajectory_player_t player;
    sb_vector3_with_yaw_t vec;
    uint32_t t, num_samples, i;

    from_msec = round_down_to_sample(from_msec);
    num_samples = (to_msec - from_msec) / SAMPLE_INTERVAL_MSEC + 1;

    CHECK(sb_trajectory_player_init(&player, trajectory));

    fprintf(out, "%s\"samples_from_ms\": %u,\n", indent, (unsigned)from_msec);
    fprintf(out, "%s\"position_mm\": [", indent);
    for (i = 0; i < num_samples; i++) {
        t = from_msec + i * SAMPLE_INTERVAL_MSEC;
        CHECK(sb_trajectory_player_get_position_at(&player, t / 1000.0f, &vec));
        fprintf(out, i > 0 ? ", " : "");
        print_vector3_with_yaw(vec);
    }
    fprintf(out, "],\n%s\"velocity_mm_s\": [", indent);
    for (i = 0; i < num_samples; i++) {
        t = from_msec + i * SAMPLE_INTERVAL_MSEC;
        CHECK(sb_trajectory_player_get_velocity_at(&player, t / 1000.0f, &vec));
        fprintf(out, i > 0 ? ", " : "");
        print_vector3_with_yaw(vec);
    }
    fprintf(out, "]");

    sb_trajectory_player_destroy(&player);
}

/** Prints the components of trajectory statistics that were calculated */
static void print_stats(const sb_trajectory_stats_t* stats)
{
    sb_trajectory_stat_components_t valid = stats->valid_components;

    fprintf(out, "{\"initial_position_mm\": ");
    print_vector3_with_yaw(stats->initial_pos);
    if (valid & SB_TRAJECTORY_STATS_DURATION) {
        fprintf(out, ", \"duration_ms\": %u", (unsigned)stats->duration_msec);
    }
    if (valid & SB_TRAJECTORY_STATS_TAKEOFF_TIME) {
        fprintf(out, ", \"earliest_above_sec\": ");
        print_float(stats->earliest_above_sec);
        fprintf(out, ", \"takeoff_time_sec\": ");
        print_float(stats->takeoff_time_sec);
    }
    if (valid & SB_TRAJECTORY_STATS_START_END_DISTANCE) {
        fprintf(out, ", \"start_to_end_distance_xy_mm\": ");
        print_float(stats->start_to_end_distance_xy);
    }
    if (valid & SB_TRAJECTORY_STATS_LANDING_TIME) {
        fprintf(out, ", \"landing_time_sec\": ");
        print_float(stats->landing_time_sec);
        fprintf(out, ", \"position_at_landing_mm\": ");
        print_vector3_with_yaw(stats->pos_at_landing_time);
        fprintf(out, ", \"velocity_at_landing_mm_s\": ");
        print_vector3_with_yaw(stats->vel_at_landing_time);
    }
    fprintf(out, "}");
}

/** Calculates the statistics of a trajectory with the default settings */
static void calculate_default_stats(sb_trajectory_t* trajectory, sb_trajectory_stats_t* stats)
{
    sb_trajectory_stats_calculator_t calc;

    CHECK(sb_trajectory_stats_calculator_init(&calc, 1000));
    CHECK(sb_trajectory_stats_calculator_run(&calc, trajectory, stats));
    sb_trajectory_stats_calculator_destroy(&calc);
}

/* ************************************************************************** */
/* Sections of the trace                                                     */
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

/** Settings of the statistics calculator; the first one is the default */
static const stats_settings_t STATS_SETTINGS[] = {
    { "default", SB_TRAJECTORY_STATS_ALL, 2000, 4000, 2500, 2500, 50 },
    { "no_preferred_descent", SB_TRAJECTORY_STATS_ALL, 2000, 4000, 2500, 0, 50 },
    { "long_preferred_descent", SB_TRAJECTORY_STATS_ALL, 2000, 4000, 2500, 20000, 50 },
    { "strict_verticality", SB_TRAJECTORY_STATS_ALL, 2000, 4000, 2500, 2500, 0 },
    { "lax_verticality", SB_TRAJECTORY_STATS_ALL, 2000, 4000, 2500, 2500, 2000 },
    { "slow_takeoff", SB_TRAJECTORY_STATS_ALL, 500, 500, 1000, 2500, 50 },
    { "infinite_acceleration", SB_TRAJECTORY_STATS_ALL, 2000, INFINITY, 2500, 2500, 50 },
    { "zero_acceleration", SB_TRAJECTORY_STATS_ALL, 2000, 0, 2500, 2500, 50 },
    { "duration_only", SB_TRAJECTORY_STATS_DURATION, 2000, 4000, 2500, 2500, 50 },
    { "takeoff_only", SB_TRAJECTORY_STATS_TAKEOFF_TIME, 2000, 4000, 2500, 2500, 50 },
    { "landing_only", SB_TRAJECTORY_STATS_LANDING_TIME, 2000, 4000, 2500, 2500, 50 },
    { "nothing", SB_TRAJECTORY_STATS_NONE, 2000, 4000, 2500, 2500, 50 },
};

static void print_stats_with_settings(sb_trajectory_t* trajectory)
{
    sb_trajectory_stats_calculator_t calc;
    sb_trajectory_stats_t stats;
    sb_error_t retval;
    size_t i, n = sizeof(STATS_SETTINGS) / sizeof(STATS_SETTINGS[0]);
    const stats_settings_t* settings;

    fprintf(out, "  \"stats\": [");
    for (i = 0; i < n; i++) {
        settings = &STATS_SETTINGS[i];

        CHECK(sb_trajectory_stats_init(&stats));
        CHECK(sb_trajectory_stats_calculator_init(&calc, 1000));
        calc.components = settings->components;
        calc.takeoff_speed = settings->takeoff_speed;
        calc.acceleration = settings->acceleration;
        calc.min_ascent = settings->min_ascent;
        calc.preferred_descent = settings->preferred_descent;
        calc.verticality_threshold = settings->verticality_threshold;
        retval = sb_trajectory_stats_calculator_run(&calc, trajectory, &stats);
        sb_trajectory_stats_calculator_destroy(&calc);

        fprintf(out, i > 0 ? ",\n    " : "\n    ");
        fprintf(out, "{\"settings\": \"%s\", \"components\": %d, \"takeoff_speed_mm_s\": ", settings->name, (int)settings->components);
        print_float(settings->takeoff_speed);
        fprintf(out, ", \"acceleration_mm_s2\": ");
        print_float(settings->acceleration);
        fprintf(out, ", \"min_ascent_mm\": ");
        print_float(settings->min_ascent);
        fprintf(out, ", \"preferred_descent_mm\": ");
        print_float(settings->preferred_descent);
        fprintf(out, ", \"verticality_threshold_mm\": ");
        print_float(settings->verticality_threshold);
        fprintf(out, ",\n     \"result\": ");
        print_error(retval);
        if (retval == SB_SUCCESS) {
            fprintf(out, ", \"stats\": ");
            print_stats(&stats);
        }
        fprintf(out, "}");

        sb_trajectory_stats_destroy(&stats);
    }
    fprintf(out, "\n  ],\n");
}

static void print_proposed_times(sb_trajectory_t* trajectory)
{
    /* min ascent, speed, acceleration */
    static const float takeoff[][3] = {
        { 2500, 1000, 1000 },
        { 2500, 2000, 4000 },
        { 0, 1000, 1000 },
        { 1000, 500, INFINITY },
        { 2500, 0, 1000 },
        { 100000, 1000, 1000 },
    };
    /* preferred descent, verticality threshold */
    static const float landing[][2] = {
        { 2500, 500 },
        { 2500, 50 },
        { 0, 500 },
        { 2500, 0 },
        { 20000, 50 },
    };
    size_t i;

    fprintf(out, "  \"proposed_takeoff_times\": [");
    for (i = 0; i < sizeof(takeoff) / sizeof(takeoff[0]); i++) {
        fprintf(out, i > 0 ? ",\n    " : "\n    ");
        fprintf(out, "{\"min_ascent_mm\": ");
        print_float(takeoff[i][0]);
        fprintf(out, ", \"speed_mm_s\": ");
        print_float(takeoff[i][1]);
        fprintf(out, ", \"acceleration_mm_s2\": ");
        print_float(takeoff[i][2]);
        fprintf(out, ", \"takeoff_time_sec\": ");
        print_float(sb_trajectory_propose_takeoff_time_sec(trajectory, takeoff[i][0], takeoff[i][1], takeoff[i][2]));
        fprintf(out, "}");
    }
    fprintf(out, "\n  ],\n");

    fprintf(out, "  \"proposed_landing_times\": [");
    for (i = 0; i < sizeof(landing) / sizeof(landing[0]); i++) {
        fprintf(out, i > 0 ? ",\n    " : "\n    ");
        fprintf(out, "{\"preferred_descent_mm\": ");
        print_float(landing[i][0]);
        fprintf(out, ", \"verticality_threshold_mm\": ");
        print_float(landing[i][1]);
        fprintf(out, ", \"landing_time_sec\": ");
        print_float(sb_trajectory_propose_landing_time_sec(trajectory, landing[i][0], landing[i][1]));
        fprintf(out, "}");
    }
    fprintf(out, "\n  ],\n");
}

static void print_trajectory_queries(sb_trajectory_t* trajectory)
{
    sb_vector3_with_yaw_t end;
    sb_bounding_box_t box;

    CHECK(sb_trajectory_get_end_position(trajectory, &end));
    CHECK(sb_trajectory_get_axis_aligned_bounding_box(trajectory, &box));

    fprintf(out, "  \"trajectory\": {\"duration_ms\": %u, \"end_position_mm\": ",
        (unsigned)sb_trajectory_get_total_duration_msec(trajectory));
    print_vector3_with_yaw(end);
    fprintf(out, ", \"bounding_box_mm\": [[");
    print_float(box.x.min);
    fprintf(out, ", ");
    print_float(box.x.max);
    fprintf(out, "], [");
    print_float(box.y.min);
    fprintf(out, ", ");
    print_float(box.y.max);
    fprintf(out, "], [");
    print_float(box.z.min);
    fprintf(out, ", ");
    print_float(box.z.max);
    fprintf(out, "]]},\n");
}

static void print_cuts(uint32_t duration_msec)
{
    float duration_sec = duration_msec / 1000.0f;
    float times_sec[] = {
        0, 0.05f, duration_sec / 4, duration_sec / 2, duration_sec - 0.05f, duration_sec, duration_sec + 10
    };
    sb_trajectory_t* trajectory;
    sb_vector3_with_yaw_t end;
    sb_error_t retval;
    uint32_t cut_msec, from_msec;
    size_t i;

    fprintf(out, "  \"cuts\": [");
    for (i = 0; i < sizeof(times_sec) / sizeof(times_sec[0]); i++) {
        trajectory = load_owned_trajectory();
        retval = sb_trajectory_cut_at(trajectory, times_sec[i]);

        fprintf(out, i > 0 ? ",\n    " : "\n    ");
        fprintf(out, "{\"time_sec\": ");
        print_float(times_sec[i]);
        fprintf(out, ", \"result\": ");
        print_error(retval);
        if (retval == SB_SUCCESS) {
            CHECK(sb_trajectory_get_end_position(trajectory, &end));
            fprintf(out, ", \"duration_ms\": %u, \"end_position_mm\": ",
                (unsigned)sb_trajectory_get_total_duration_msec(trajectory));
            print_vector3_with_yaw(end);
            fprintf(out, ",\n     ");
            cut_msec = (uint32_t)(times_sec[i] * 1000);
            from_msec = cut_msec > CUT_SAMPLING_WINDOW_MSEC ? cut_msec - CUT_SAMPLING_WINDOW_MSEC : 0;
            print_trajectory_samples(trajectory, from_msec, cut_msec + CUT_SAMPLING_WINDOW_MSEC, "     ");
        }
        fprintf(out, "}");

        SB_XDECREF(trajectory);
    }
    fprintf(out, "\n  ],\n");
}

typedef enum {
    LANDING_TARGET_BELOW_LANDING_POSITION,
    LANDING_TARGET_START_POSITION,
    LANDING_TARGET_ABOVE_LANDING_POSITION,
} landing_target_t;

typedef struct {
    const char* name;
    landing_target_t target;
    float landing_velocity;
    float terminal_velocity; /* NAN if the variant without terminal velocity is used */
} landing_case_t;

static const landing_case_t LANDING_CASES[] = {
    { "below_landing_position", LANDING_TARGET_BELOW_LANDING_POSITION, 500, NAN },
    { "below_landing_position_fast", LANDING_TARGET_BELOW_LANDING_POSITION, 2000, NAN },
    { "start_position", LANDING_TARGET_START_POSITION, 500, NAN },
    { "terminal_velocity", LANDING_TARGET_BELOW_LANDING_POSITION, 500, 300 },
    { "zero_terminal_velocity", LANDING_TARGET_BELOW_LANDING_POSITION, 500, 0 },
    { "zero_landing_velocity", LANDING_TARGET_BELOW_LANDING_POSITION, 0, NAN },
    { "negative_landing_velocity", LANDING_TARGET_BELOW_LANDING_POSITION, -500, NAN },
    { "above_landing_position", LANDING_TARGET_ABOVE_LANDING_POSITION, 500, NAN },
};

static void print_landings(void)
{
    sb_trajectory_t* trajectory;
    sb_trajectory_stats_t stats;
    sb_vector3_t target;
    sb_error_t retval;
    uint32_t original_landing_msec, from_msec, duration_msec;
    const landing_case_t* landing;
    size_t i;

    fprintf(out, "  \"landings\": [");
    for (i = 0; i < sizeof(LANDING_CASES) / sizeof(LANDING_CASES[0]); i++) {
        landing = &LANDING_CASES[i];
        trajectory = load_owned_trajectory();
        CHECK(sb_trajectory_stats_init(&stats));
        calculate_default_stats(trajectory, &stats);

        switch (landing->target) {
        case LANDING_TARGET_START_POSITION:
            target.x = stats.initial_pos.x;
            target.y = stats.initial_pos.y;
            target.z = stats.initial_pos.z;
            break;
        case LANDING_TARGET_ABOVE_LANDING_POSITION:
            target.x = stats.pos_at_landing_time.x;
            target.y = stats.pos_at_landing_time.y;
            target.z = stats.pos_at_landing_time.z + 1000;
            break;
        default:
            target.x = stats.pos_at_landing_time.x;
            target.y = stats.pos_at_landing_time.y;
            target.z = stats.initial_pos.z;
            break;
        }

        original_landing_msec = (uint32_t)(stats.landing_time_sec * 1000);
        if (isnan(landing->terminal_velocity)) {
            retval = sb_trajectory_replace_end_to_land_at(trajectory, &stats, target, landing->landing_velocity);
        } else {
            retval = sb_trajectory_replace_end_to_land_at_with_terminal_velocity(
                trajectory, &stats, target, landing->landing_velocity, landing->terminal_velocity);
        }

        fprintf(out, i > 0 ? ",\n    " : "\n    ");
        fprintf(out, "{\"case\": \"%s\", \"target_mm\": ", landing->name);
        print_vector3(target);
        fprintf(out, ", \"landing_velocity_mm_s\": ");
        print_float(landing->landing_velocity);
        fprintf(out, ", \"terminal_velocity_mm_s\": ");
        print_float(landing->terminal_velocity);
        fprintf(out, ", \"result\": ");
        print_error(retval);

        /* Recorded on failure as well; the trajectory must be left intact then */
        duration_msec = sb_trajectory_get_total_duration_msec(trajectory);
        fprintf(out, ",\n     \"duration_ms\": %u, \"stats\": ", (unsigned)duration_msec);
        print_stats(&stats);
        if (retval == SB_SUCCESS) {
            fprintf(out, ",\n     ");
            from_msec = original_landing_msec > LANDING_SAMPLING_LEAD_MSEC ? original_landing_msec - LANDING_SAMPLING_LEAD_MSEC : 0;
            print_trajectory_samples(trajectory, from_msec, duration_msec + MARGIN_MSEC, "     ");
        }
        fprintf(out, "}");

        sb_trajectory_stats_destroy(&stats);
        SB_XDECREF(trajectory);
    }
    fprintf(out, "\n  ],\n");
}

static void print_rth_trajectories(sb_trajectory_t* show_trajectory, sb_rth_plan_t* plan)
{
    sb_trajectory_player_t player;
    sb_trajectory_t* trajectory;
    sb_rth_plan_entry_t entry;
    sb_vector3_with_yaw_t position;
    sb_vector3_t start;
    sb_error_t retval;
    uint32_t t, duration_msec = sb_trajectory_get_total_duration_msec(show_trajectory);
    float last_entry_time = NAN;
    int first = 1;

    CHECK(sb_trajectory_player_init(&player, show_trajectory));

    /* Each distinct entry of the plan is converted once, starting from the
     * position of the drone at the time of the entry */
    fprintf(out, "  \"rth_trajectories\": [");
    for (t = 0; t <= duration_msec; t += SAMPLE_INTERVAL_MSEC) {
        CHECK(sb_rth_plan_evaluate_at(plan, t / 1000.0f, &entry));
        if (entry.time_sec == last_entry_time) {
            continue;
        }
        last_entry_time = entry.time_sec;

        CHECK(sb_trajectory_player_get_position_at(&player, entry.time_sec, &position));
        start.x = position.x;
        start.y = position.y;
        start.z = position.z;

        trajectory = sb_trajectory_new();
        if (trajectory == 0) {
            fprintf(stderr, "out of memory\n");
            exit(1);
        }
        retval = sb_trajectory_update_from_rth_plan_entry(trajectory, &entry, start);

        fprintf(out, first ? "\n    " : ",\n    ");
        first = 0;
        fprintf(out, "{\"entry_time_sec\": ");
        print_float(entry.time_sec);
        fprintf(out, ", \"start_mm\": ");
        print_vector3(start);
        fprintf(out, ", \"result\": ");
        print_error(retval);
        if (retval == SB_SUCCESS) {
            fprintf(out, ", \"duration_ms\": %u,\n     ", (unsigned)sb_trajectory_get_total_duration_msec(trajectory));
            print_trajectory_samples(trajectory, (uint32_t)(entry.time_sec * 1000),
                sb_trajectory_get_total_duration_msec(trajectory) + MARGIN_MSEC, "     ");
        }
        fprintf(out, "}");

        SB_XDECREF(trajectory);
    }
    fprintf(out, first ? "],\n" : "\n  ],\n");

    sb_trajectory_player_destroy(&player);
}

/** Records the timestamps where the light player reports a change */
static void print_light_player(sb_light_program_t* program)
{
    sb_light_player_t player;
    unsigned long t = 0, next = 0;
    size_t num_changes = 0;
    int ended = 0;

    CHECK(sb_light_player_init(&player, program));

    fprintf(out, "  \"light_player\": {\"changes_ms\": [");
    while (t <= MAX_LIGHT_PROGRAM_DURATION_MSEC && num_changes < MAX_LIGHT_PROGRAM_CHANGES) {
        ended = sb_light_player_seek(&player, t, &next);
        fprintf(out, "%s[%lu, %lu]", num_changes > 0 ? ", " : "", t, next);
        num_changes++;
        if (ended) {
            break;
        }
        t = next > t ? next : t + 1;
    }
    fprintf(out, "], \"ended\": %s},\n", ended ? "true" : "false");

    sb_light_player_destroy(&player);
}

/** Records the next event after seeking the event player to each sample time */
static void print_event_player(sb_event_list_t* events, uint32_t end_msec)
{
    sb_event_list_player_t player;
    const sb_event_t* event;
    uint32_t t;

    CHECK(sb_event_list_player_init(&player, events));

    fprintf(out, "  \"event_player\": {\"next_event_index_after_seek\": [");
    for (t = 0; t <= end_msec; t += SAMPLE_INTERVAL_MSEC) {
        sb_event_list_player_seek(&player, t / 1000.0f);
        event = sb_event_list_player_peek_next_event(&player);
        fprintf(out, t > 0 ? ", " : "");
        if (event) {
            fprintf(out, "%d", (int)(event - sb_event_list_get_ptr_const(events, 0)));
        } else {
            fprintf(out, "null");
        }
    }
    fprintf(out, "]},\n");

    sb_event_list_player_destroy(&player);
}

/* ************************************************************************** */

int main(int argc, char* argv[])
{
    sb_trajectory_t trajectory;
    sb_light_program_t* light_program;
    sb_event_list_t* events;
    sb_rth_plan_t* rth_plan;
    int has_trajectory, has_light_program, has_events, has_rth_plan;
    uint32_t end_msec = 0;

    if (argc != 3) {
        fprintf(stderr, "Usage: %s <input.skyb> <output.json>\n", argv[0]);
        return 2;
    }

    read_show_file(argv[1], &show_data, &show_length);

    CHECK(sb_trajectory_init(&trajectory));
    light_program = sb_light_program_new();
    events = sb_event_list_new(0);
    rth_plan = sb_rth_plan_new();
    if (!light_program || !events || !rth_plan) {
        fprintf(stderr, "out of memory\n");
        return 1;
    }

    has_trajectory = is_present(sb_trajectory_update_from_binary_file_in_memory(&trajectory, show_data, show_length), "trajectory");
    has_light_program = is_present(sb_light_program_update_from_binary_file_in_memory(light_program, show_data, show_length), "light program");
    has_events = is_present(sb_event_list_update_from_binary_file_in_memory(events, show_data, show_length), "event list");
    has_rth_plan = is_present(sb_rth_plan_update_from_binary_file_in_memory(rth_plan, show_data, show_length), "RTH plan");

    if (has_trajectory) {
        end_msec = sb_trajectory_get_total_duration_msec(&trajectory);
    }
    if (has_events && sb_event_list_size(events) > 0) {
        uint32_t last = sb_event_list_get_ptr_const(events, sb_event_list_size(events) - 1)->time_msec;
        end_msec = end_msec > last ? end_msec : last;
    }
    end_msec += MARGIN_MSEC;

    out = fopen(argv[2], "w");
    if (out == 0) {
        perror(argv[2]);
        return 1;
    }

    fprintf(out, "{\n");
    fprintf(out, "  \"version\": 1,\n");
    fprintf(out, "  \"fixture\": \"%s\",\n", base_name(argv[1]));
    fprintf(out, "  \"generator\": \"libskybrush %d.%d.%d\",\n",
        SKYBRUSH_VERSION_MAJOR, SKYBRUSH_VERSION_MINOR, SKYBRUSH_VERSION_PATCH);
    fprintf(out, "  \"sample_interval_ms\": %d,\n", SAMPLE_INTERVAL_MSEC);

    if (has_trajectory) {
        print_trajectory_queries(&trajectory);
        print_stats_with_settings(&trajectory);
        print_proposed_times(&trajectory);
        print_cuts(sb_trajectory_get_total_duration_msec(&trajectory));
        print_landings();
        if (has_rth_plan) {
            print_rth_trajectories(&trajectory, rth_plan);
        }
    }
    if (has_light_program) {
        print_light_player(light_program);
    }
    if (has_events) {
        print_event_player(events, end_msec);
    }

    /* Closes the object; every section above ends with a comma */
    fprintf(out, "  \"end\": true\n}\n");
    fclose(out);

    SB_XDECREF(rth_plan);
    SB_XDECREF(events);
    SB_XDECREF(light_program);
    SB_DECREF_STATIC(&trajectory);
    free(show_data);

    return 0;
}
