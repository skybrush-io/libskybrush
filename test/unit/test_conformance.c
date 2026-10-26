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
 * Conformance tests of the Skybrush binary show file format.
 *
 * The test cases are maintained together with the specification of the file
 * format and are converted into test functions at build time; see
 * test/conformance/generate_tests.py. Each block is loaded both from memory
 * and from a file descriptor, and both must yield the expected results.
 */

#include <stdio.h>
#include <unistd.h>

#include <skybrush/events.h>
#include <skybrush/formats/binary.h>
#include <skybrush/gcs_light_control.h>
#include <skybrush/lights.h>
#include <skybrush/metadata.h>
#include <skybrush/refcount.h>
#include <skybrush/rth_plan.h>
#include <skybrush/trajectory.h>
#include <skybrush/yaw_control.h>

#include "unity.h"

typedef struct {
    uint32_t time_msec;
    uint8_t type;
    uint8_t subtype;
    uint8_t payload[4];
} expected_event_t;

typedef enum {
    IN_MEMORY,
    FROM_FILE_DESCRIPTOR,
    NUM_MODES
} load_mode_t;

static const char* MODE_NAMES[NUM_MODES] = { "in-memory", "file descriptor" };

/* The bytes of the current test case and a temporary file holding them */
static uint8_t* case_bytes;
static size_t case_length;
static FILE* case_file;

static void begin_case(uint8_t* bytes, size_t length);
static void end_case(void);
static void expect_file(sb_error_t expected);
static void expect_trajectory(
    sb_error_t result, int check_duration, uint32_t duration_msec,
    const float (*positions)[4], size_t num_positions);
static void expect_light_program(
    sb_error_t result, const int (*colors)[4], size_t num_colors, long ends_at_msec);
static void expect_yaw_control(
    sb_error_t result, int check_auto_yaw, int auto_yaw, int check_offset, int offset_ddeg,
    int check_num_deltas, size_t num_deltas, const float (*yaw)[2], size_t num_yaw);
static void expect_event_list(
    sb_error_t result, int check_events, const expected_event_t* events, size_t num_events);
static void expect_rth_plan(
    sb_error_t result, int check_points, const float (*points)[2], size_t num_points,
    int check_num_entries, size_t num_entries);
static void expect_gcs_light_control_setup(
    sb_error_t result, int check_coords, int x, int y, int check_size, int width, int height,
    int check_palette, const uint8_t (*palette)[3], size_t num_colors);
static void expect_metadata(
    sb_error_t result, int check_show_id, const uint8_t* show_id, int check_drone_index,
    unsigned int drone_index);

/* The generated test cases; also defines the tolerances used below */
#include "conformance_cases.inc"

void setUp(void)
{
}

void tearDown(void)
{
}

static void begin_case(uint8_t* bytes, size_t length)
{
    case_bytes = bytes;
    case_length = length;

    case_file = tmpfile();
    TEST_ASSERT_NOT_NULL(case_file);
    TEST_ASSERT_EQUAL(length, fwrite(bytes, 1, length, case_file));
    TEST_ASSERT_EQUAL(0, fflush(case_file));
}

static void end_case(void)
{
    fclose(case_file);
    case_file = 0;
}

/** Returns a file descriptor of the temporary file, rewound to its start */
static int case_fd(void)
{
    int fd = fileno(case_file);
    TEST_ASSERT_EQUAL(0, lseek(fd, 0, SEEK_SET));
    return fd;
}

static void expect_file(sb_error_t expected)
{
    sb_binary_file_parser_t parser;
    sb_error_t retval;

    retval = sb_binary_file_parser_init_from_buffer(&parser, case_bytes, case_length);
    TEST_ASSERT_EQUAL_INT_MESSAGE(expected, retval, MODE_NAMES[IN_MEMORY]);
    if (retval == SB_SUCCESS) {
        sb_binary_file_parser_destroy(&parser);
    }

    retval = sb_binary_file_parser_init_from_file(&parser, case_fd());
    TEST_ASSERT_EQUAL_INT_MESSAGE(expected, retval, MODE_NAMES[FROM_FILE_DESCRIPTOR]);
    if (retval == SB_SUCCESS) {
        sb_binary_file_parser_destroy(&parser);
    }
}

static void expect_trajectory(
    sb_error_t result, int check_duration, uint32_t duration_msec,
    const float (*positions)[4], size_t num_positions)
{
    for (int mode = 0; mode < NUM_MODES; mode++) {
        sb_trajectory_t* trajectory = sb_trajectory_new();
        sb_trajectory_player_t player;
        sb_vector3_with_yaw_t pos;
        sb_error_t retval;

        TEST_ASSERT_NOT_NULL(trajectory);
        retval = mode == IN_MEMORY
            ? sb_trajectory_update_from_binary_file_in_memory(trajectory, case_bytes, case_length)
            : sb_trajectory_update_from_binary_file(trajectory, case_fd());
        TEST_ASSERT_EQUAL_INT_MESSAGE(result, retval, MODE_NAMES[mode]);

        if (retval == SB_SUCCESS) {
            if (check_duration) {
                TEST_ASSERT_EQUAL_UINT32_MESSAGE(duration_msec, sb_trajectory_get_total_duration_msec(trajectory), MODE_NAMES[mode]);
            }

            TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_init(&player, trajectory));
            for (size_t i = 0; i < num_positions; i++) {
                TEST_ASSERT_EQUAL(SB_SUCCESS, sb_trajectory_player_get_position_at(&player, positions[i][0] / 1000.0f, &pos));
                TEST_ASSERT_FLOAT_WITHIN_MESSAGE(POSITION_TOLERANCE_MM, positions[i][1], pos.x, MODE_NAMES[mode]);
                TEST_ASSERT_FLOAT_WITHIN_MESSAGE(POSITION_TOLERANCE_MM, positions[i][2], pos.y, MODE_NAMES[mode]);
                TEST_ASSERT_FLOAT_WITHIN_MESSAGE(POSITION_TOLERANCE_MM, positions[i][3], pos.z, MODE_NAMES[mode]);
            }
            sb_trajectory_player_destroy(&player);
        }

        SB_XDECREF(trajectory);
    }
}

static void assert_color_at(sb_light_player_t* player, const int* expected, load_mode_t mode)
{
    sb_rgb_color_t color = sb_light_player_get_color_at(player, expected[0]);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(expected[1], color.red, MODE_NAMES[mode]);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(expected[2], color.green, MODE_NAMES[mode]);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(expected[3], color.blue, MODE_NAMES[mode]);
}

/**
 * \param ends_at_msec  the time when the program ends, -1 if it never ends,
 *        -2 if it does not need to be checked
 */
static void expect_light_program(
    sb_error_t result, const int (*colors)[4], size_t num_colors, long ends_at_msec)
{
    for (int mode = 0; mode < NUM_MODES; mode++) {
        sb_light_program_t* program = sb_light_program_new();
        sb_light_player_t player;
        sb_error_t retval;
        size_t i;

        TEST_ASSERT_NOT_NULL(program);
        retval = mode == IN_MEMORY
            ? sb_light_program_update_from_binary_file_in_memory(program, case_bytes, case_length)
            : sb_light_program_update_from_binary_file(program, case_fd());
        TEST_ASSERT_EQUAL_INT_MESSAGE(result, retval, MODE_NAMES[mode]);

        if (retval == SB_SUCCESS) {
            TEST_ASSERT_EQUAL(SB_SUCCESS, sb_light_player_init(&player, program));

            /* The colors must not depend on the order of the queries */
            for (i = 0; i < num_colors; i++) {
                assert_color_at(&player, colors[i], mode);
            }
            for (i = num_colors; i > 0; i--) {
                assert_color_at(&player, colors[i - 1], mode);
            }

            if (ends_at_msec == -1) {
                TEST_ASSERT_FALSE_MESSAGE(sb_light_player_seek(&player, 10000000, 0), MODE_NAMES[mode]);
            } else if (ends_at_msec >= 0) {
                if (ends_at_msec > 0) {
                    TEST_ASSERT_FALSE_MESSAGE(sb_light_player_seek(&player, ends_at_msec - 1, 0), MODE_NAMES[mode]);
                }
                TEST_ASSERT_TRUE_MESSAGE(sb_light_player_seek(&player, ends_at_msec, 0), MODE_NAMES[mode]);
            }

            sb_light_player_destroy(&player);
        }

        SB_XDECREF(program);
    }
}

static void expect_yaw_control(
    sb_error_t result, int check_auto_yaw, int auto_yaw, int check_offset, int offset_ddeg,
    int check_num_deltas, size_t num_deltas, const float (*yaw)[2], size_t num_yaw)
{
    for (int mode = 0; mode < NUM_MODES; mode++) {
        sb_yaw_control_t* ctrl = sb_yaw_control_new();
        sb_yaw_player_t player;
        sb_error_t retval;
        float value;

        TEST_ASSERT_NOT_NULL(ctrl);
        retval = mode == IN_MEMORY
            ? sb_yaw_control_update_from_binary_file_in_memory(ctrl, case_bytes, case_length)
            : sb_yaw_control_update_from_binary_file(ctrl, case_fd());
        TEST_ASSERT_EQUAL_INT_MESSAGE(result, retval, MODE_NAMES[mode]);

        if (retval == SB_SUCCESS) {
            if (check_auto_yaw) {
                TEST_ASSERT_EQUAL_INT_MESSAGE(auto_yaw, ctrl->auto_yaw ? 1 : 0, MODE_NAMES[mode]);
            }
            if (check_offset) {
                TEST_ASSERT_EQUAL_INT_MESSAGE(offset_ddeg, ctrl->yaw_offset_ddeg, MODE_NAMES[mode]);
            }
            if (check_num_deltas) {
                TEST_ASSERT_EQUAL_UINT_MESSAGE((unsigned int)(num_deltas), (unsigned int)(ctrl->num_deltas), MODE_NAMES[mode]);
            }

            TEST_ASSERT_EQUAL(SB_SUCCESS, sb_yaw_player_init(&player, ctrl));
            for (size_t i = 0; i < num_yaw; i++) {
                TEST_ASSERT_EQUAL(SB_SUCCESS, sb_yaw_player_get_yaw_at(&player, yaw[i][0] / 1000.0f, &value));
                TEST_ASSERT_FLOAT_WITHIN_MESSAGE(YAW_TOLERANCE_DEG, yaw[i][1], value, MODE_NAMES[mode]);
            }
            sb_yaw_player_destroy(&player);
        }

        SB_XDECREF(ctrl);
    }
}

static void expect_event_list(
    sb_error_t result, int check_events, const expected_event_t* events, size_t num_events)
{
    for (int mode = 0; mode < NUM_MODES; mode++) {
        sb_event_list_t* list = sb_event_list_new(0);
        sb_error_t retval;

        TEST_ASSERT_NOT_NULL(list);
        retval = mode == IN_MEMORY
            ? sb_event_list_update_from_binary_file_in_memory(list, case_bytes, case_length)
            : sb_event_list_update_from_binary_file(list, case_fd());
        TEST_ASSERT_EQUAL_INT_MESSAGE(result, retval, MODE_NAMES[mode]);

        if (retval == SB_SUCCESS && check_events) {
            TEST_ASSERT_EQUAL_UINT_MESSAGE((unsigned int)(num_events), (unsigned int)(sb_event_list_size(list)), MODE_NAMES[mode]);
            for (size_t i = 0; i < num_events; i++) {
                const sb_event_t* event = sb_event_list_get_ptr_const(list, i);
                TEST_ASSERT_EQUAL_UINT32_MESSAGE(events[i].time_msec, event->time_msec, MODE_NAMES[mode]);
                TEST_ASSERT_EQUAL_INT_MESSAGE(events[i].type, event->type, MODE_NAMES[mode]);
                TEST_ASSERT_EQUAL_INT_MESSAGE(events[i].subtype, event->subtype, MODE_NAMES[mode]);
                TEST_ASSERT_EQUAL_UINT8_ARRAY_MESSAGE(events[i].payload, event->payload.as_buf, 4, MODE_NAMES[mode]);
            }
        }

        SB_XDECREF(list);
    }
}

static void expect_rth_plan(
    sb_error_t result, int check_points, const float (*points)[2], size_t num_points,
    int check_num_entries, size_t num_entries)
{
    for (int mode = 0; mode < NUM_MODES; mode++) {
        sb_rth_plan_t* plan = sb_rth_plan_new();
        sb_vector2_t point;
        sb_error_t retval;

        TEST_ASSERT_NOT_NULL(plan);
        retval = mode == IN_MEMORY
            ? sb_rth_plan_update_from_binary_file_in_memory(plan, case_bytes, case_length)
            : sb_rth_plan_update_from_binary_file(plan, case_fd());
        TEST_ASSERT_EQUAL_INT_MESSAGE(result, retval, MODE_NAMES[mode]);

        if (retval == SB_SUCCESS) {
            if (check_points) {
                TEST_ASSERT_EQUAL_UINT_MESSAGE((unsigned int)(num_points), (unsigned int)(sb_rth_plan_get_num_points(plan)), MODE_NAMES[mode]);
                for (size_t i = 0; i < num_points; i++) {
                    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_rth_plan_get_point(plan, i, &point));
                    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(POSITION_TOLERANCE_MM, points[i][0], point.x, MODE_NAMES[mode]);
                    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(POSITION_TOLERANCE_MM, points[i][1], point.y, MODE_NAMES[mode]);
                }
            }
            if (check_num_entries) {
                TEST_ASSERT_EQUAL_UINT_MESSAGE((unsigned int)(num_entries), (unsigned int)(sb_rth_plan_get_num_entries(plan)), MODE_NAMES[mode]);
            }
        }

        SB_XDECREF(plan);
    }
}

static void expect_gcs_light_control_setup(
    sb_error_t result, int check_coords, int x, int y, int check_size, int width, int height,
    int check_palette, const uint8_t (*palette)[3], size_t num_colors)
{
    for (int mode = 0; mode < NUM_MODES; mode++) {
        sb_gcs_light_control_setup_t setup;
        sb_error_t retval;

        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_gcs_light_control_setup_init(&setup));
        retval = mode == IN_MEMORY
            ? sb_gcs_light_control_setup_update_from_binary_file_in_memory(&setup, case_bytes, case_length)
            : sb_gcs_light_control_setup_update_from_binary_file(&setup, case_fd());
        TEST_ASSERT_EQUAL_INT_MESSAGE(result, retval, MODE_NAMES[mode]);

        if (retval == SB_SUCCESS) {
            if (check_coords) {
                TEST_ASSERT_EQUAL_INT_MESSAGE(x, setup.coords.x, MODE_NAMES[mode]);
                TEST_ASSERT_EQUAL_INT_MESSAGE(y, setup.coords.y, MODE_NAMES[mode]);
            }
            if (check_size) {
                TEST_ASSERT_EQUAL_INT_MESSAGE(width, setup.size.x, MODE_NAMES[mode]);
                TEST_ASSERT_EQUAL_INT_MESSAGE(height, setup.size.y, MODE_NAMES[mode]);
            }
            if (check_palette) {
                TEST_ASSERT_EQUAL_UINT_MESSAGE((unsigned int)(num_colors), (unsigned int)(sb_color_palette_size(&setup.palette)), MODE_NAMES[mode]);
                for (size_t i = 0; i < num_colors; i++) {
                    sb_rgb_color_t color = sb_color_palette_get_color(&setup.palette, i);
                    TEST_ASSERT_EQUAL_UINT8_MESSAGE(palette[i][0], color.red, MODE_NAMES[mode]);
                    TEST_ASSERT_EQUAL_UINT8_MESSAGE(palette[i][1], color.green, MODE_NAMES[mode]);
                    TEST_ASSERT_EQUAL_UINT8_MESSAGE(palette[i][2], color.blue, MODE_NAMES[mode]);
                }
            }
        }

        sb_gcs_light_control_setup_destroy(&setup);
    }
}

static void expect_metadata(
    sb_error_t result, int check_show_id, const uint8_t* show_id, int check_drone_index,
    unsigned int drone_index)
{
    for (int mode = 0; mode < NUM_MODES; mode++) {
        sb_show_metadata_t metadata;
        uint8_t actual_show_id[SB_SHOW_METADATA_SHOW_ID_LENGTH];
        uint16_t actual_drone_index;
        sb_error_t retval;

        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_init(&metadata));
        retval = mode == IN_MEMORY
            ? sb_show_metadata_update_from_binary_file_in_memory(&metadata, case_bytes, case_length)
            : sb_show_metadata_update_from_binary_file(&metadata, case_fd());
        TEST_ASSERT_EQUAL_INT_MESSAGE(result, retval, MODE_NAMES[mode]);

        if (retval == SB_SUCCESS) {
            if (check_show_id) {
                TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_get_show_id(&metadata, actual_show_id));
                TEST_ASSERT_EQUAL_UINT8_ARRAY_MESSAGE(show_id, actual_show_id, SB_SHOW_METADATA_SHOW_ID_LENGTH, MODE_NAMES[mode]);
            }
            if (check_drone_index) {
                TEST_ASSERT_EQUAL(SB_SUCCESS, sb_show_metadata_get_drone_index(&metadata, &actual_drone_index));
                TEST_ASSERT_EQUAL_UINT16_MESSAGE(drone_index, actual_drone_index, MODE_NAMES[mode]);
            }
        }

        sb_show_metadata_destroy(&metadata);
    }
}

int main(int argc, char* argv[])
{
    UNITY_BEGIN();

    RUN_CONFORMANCE_TESTS();

    return UNITY_END();
}
