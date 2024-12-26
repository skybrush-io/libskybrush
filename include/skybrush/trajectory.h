/*
 * This file is part of libskybrush.
 *
 * Copyright 2020-2024 CollMot Robotics Ltd.
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

#ifndef SKYBRUSH_TRAJECTORY_H
#define SKYBRUSH_TRAJECTORY_H

/**
 * @file trajectory.h
 * @brief Functions and structures to evaluate \c libskybrush trajectories at
 * arbitrary points in time
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <skybrush/basic_types.h>
#include <skybrush/buffer.h>
#include <skybrush/error.h>
#include <skybrush/poly.h>

#include <skybrush/decls.h>

__BEGIN_DECLS

/** Enum containing constants that help the interpretation of the header byte
 * before each trajectory segment in the binary representation of the
 * trajectory.
 */
typedef enum {
    SB_X_CONSTANT = 0,
    SB_X_LINEAR = 0x01,
    SB_X_BEZIER = 0x02,
    SB_X_POLY7D = 0x03,

    SB_Y_CONSTANT = 0,
    SB_Y_LINEAR = 0x04,
    SB_Y_BEZIER = 0x08,
    SB_Y_POLY7D = 0x0C,

    SB_Z_CONSTANT = 0,
    SB_Z_LINEAR = 0x10,
    SB_Z_BEZIER = 0x20,
    SB_Z_POLY7D = 0x30,

    SB_YAW_CONSTANT = 0,
    SB_YAW_LINEAR = 0x40,
    SB_YAW_BEZIER = 0x80,
    SB_YAW_POLY7D = 0xC0,
} sb_trajectory_segment_format_flags_t;

typedef enum {
    SB_TRAJECTORY_USE_YAW = 1
} sb_trajectory_flags_t;

/**
 * Structure representing a single trajectory segment in a Skybrush mission.
 */
typedef struct
{
    /** The start time of the trajectory segment since the start of the mission,
     * in milliseconds */
    uint32_t start_time_msec;

    /** The end time of the trajectory segment since the start of the mission,
     * in milliseconds */
    uint32_t end_time_msec;

    /** The duration of the trajectory segment, in milliseconds. */
    uint16_t duration_msec;

    /** The start time of the trajectory segment since the start of the mission,
     * in seconds */
    float start_time_sec;

    /** The end time of the trajectory segment since the start of the mission,
     * in seconds */
    float end_time_sec;

    /** The duration of the trajectory segment, in seconds. */
    float duration_sec;

    /** Flags storing which parts of the segment are up-to-date. */
    uint8_t flags;

    /** @brief The polynomial that describes the current trajectory segment.
     *
     * The interval [0; 1] of the polynomial is mapped to the time interval
     * between the start and end time of the trajectory.
     *
     * This polynomial is calculated lazily. Do not access this field directly.
     */
    sb_poly_4d_t poly;

    /** @brief The polynomial that describes the first derivative of the current
     * trajectory segment.
     *
     * The interval [0; 1] of the polynomial is mapped to the time interval
     * between the start and end time of the trajectory.
     *
     * This polynomial is calculated lazily. Do not access this field directly.
     */
    sb_poly_4d_t dpoly;

    /** @brief The polynomial that describes the second derivative of the current
     * trajectory segment.
     *
     * The interval [0; 1] of the polynomial is mapped to the time interval
     * between the start and end time of the trajectory.
     *
     * This polynomial is calculated lazily. Do not access this field directly.
     */
    sb_poly_4d_t ddpoly;
} sb_trajectory_segment_t;

/**
 * Structure representing the trajectory of a single drone in a Skybrush
 * mission.
 */
typedef struct sb_trajectory_s {
    sb_buffer_t buffer; /**< The buffer holding the trajectory */

    sb_vector3_with_yaw_t start; /**< The start coordinate of the trajectory */
    float scale; /**< Scaling factor for the coordinates */
    sb_bool_t use_yaw; /**< Whether the yaw coordinates are relevant */
    size_t header_length; /**< Number of bytes in the header of the buffer */
} sb_trajectory_t;

struct sb_trajectory_builder_s;

sb_error_t sb_trajectory_init_from_binary_file(sb_trajectory_t* trajectory, int fd);
sb_error_t sb_trajectory_init_from_binary_file_in_memory(
    sb_trajectory_t* trajectory, uint8_t* buf, size_t nbytes);
sb_error_t sb_trajectory_init_from_buffer(sb_trajectory_t* trajectory,
    uint8_t* buf, size_t nbytes);
sb_error_t sb_trajectory_init_from_bytes(sb_trajectory_t* trajectory,
    uint8_t* buf, size_t nbytes);
sb_error_t sb_trajectory_init_from_builder(
    sb_trajectory_t* trajectory, struct sb_trajectory_builder_s* builder);
sb_error_t sb_trajectory_init_empty(sb_trajectory_t* trajectory);
void sb_trajectory_destroy(sb_trajectory_t* trajectory);

sb_bool_t sb_trajectory_is_empty(const sb_trajectory_t* trajectory);
sb_error_t sb_trajectory_get_axis_aligned_bounding_box(
    const sb_trajectory_t* trajectory, sb_bounding_box_t* result);
sb_error_t sb_trajectory_get_end_position(
    const sb_trajectory_t* trajectory, sb_vector3_with_yaw_t* result);
sb_error_t sb_trajectory_get_start_position(
    const sb_trajectory_t* trajectory, sb_vector3_with_yaw_t* result);
uint32_t sb_trajectory_get_total_duration_msec(const sb_trajectory_t* trajectory);
float sb_trajectory_get_total_duration_sec(const sb_trajectory_t* trajectory);
float sb_trajectory_propose_takeoff_time_sec(
    const sb_trajectory_t* trajectory, float min_ascent, float speed, float acceleration);
float sb_trajectory_propose_landing_time_sec(
    const sb_trajectory_t* trajectory, float min_descent);

sb_error_t sb_trajectory_clear(sb_trajectory_t* trajectory);

/* ************************************************************************* */

/**
 * Structure representing a trajectory player that allows us to query the
 * position and velocity along a trajectory.
 */
typedef struct sb_trajectory_player_s {
    const sb_trajectory_t* trajectory; /**< The trajectory that the player plays */

    /** The current segment that is being evaluated by the player */
    struct
    {
        size_t start; /**< Start offset of the current segment */
        size_t start_of_coordinates; /**< Start offset of the coordinates in the segment */
        size_t length; /**< Length of the current segment in the buffer */
        sb_trajectory_segment_t data; /**< The current segment of the trajectory */
    } current_segment;
} sb_trajectory_player_t;

sb_error_t sb_trajectory_player_init(sb_trajectory_player_t* player, const sb_trajectory_t* trajectory);
void sb_trajectory_player_destroy(sb_trajectory_player_t* player);
sb_error_t sb_trajectory_player_build_next_segment(sb_trajectory_player_t* player);
void sb_trajectory_player_dump_current_segment(const sb_trajectory_player_t* player);
const sb_trajectory_segment_t* sb_trajectory_player_get_current_segment(
    const sb_trajectory_player_t* player);
sb_error_t sb_trajectory_player_get_position_at(
    sb_trajectory_player_t* player, float t, sb_vector3_with_yaw_t* result);
sb_error_t sb_trajectory_player_get_velocity_at(
    sb_trajectory_player_t* player, float t, sb_vector3_with_yaw_t* result);
sb_error_t sb_trajectory_player_get_acceleration_at(
    sb_trajectory_player_t* player, float t, sb_vector3_with_yaw_t* result);
sb_error_t sb_trajectory_player_get_total_duration_msec(
    sb_trajectory_player_t* player, uint32_t* duration);
sb_bool_t sb_trajectory_player_has_more_segments(const sb_trajectory_player_t* player);

/* ************************************************************************* */

/**
 * Structure that allows one to build a new trajectory from scratch.
 */
typedef struct sb_trajectory_builder_s {
    sb_buffer_t buffer; /**< Buffer holding the binary representation of the trajectory being built */
    sb_vector3_with_yaw_t last_position; /**< Last position in the trajectory */
    float scale; /**< Scaling factor for the coordinates */
} sb_trajectory_builder_t;

sb_error_t sb_trajectory_builder_init(
    sb_trajectory_builder_t* builder, uint8_t scale, uint8_t flags);
void sb_trajectory_builder_destroy(sb_trajectory_builder_t* builder);

sb_error_t sb_trajectory_builder_set_start_position(
    sb_trajectory_builder_t* builder, sb_vector3_with_yaw_t start);
sb_error_t sb_trajectory_builder_append_line(
    sb_trajectory_builder_t* builder, sb_vector3_with_yaw_t target,
    uint32_t duration_msec);
sb_error_t sb_trajectory_builder_hold_position_for(
    sb_trajectory_builder_t* builder, uint32_t duration_msec);

__END_DECLS

#endif
