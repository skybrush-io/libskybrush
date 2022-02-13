/*
 * This file is part of libskybrush.
 *
 * Copyright 2020-2022 CollMot Robotics Ltd.
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

    /** @brief The polynomial that describes the current trajectory segment.
     *
     * The interval [0; 1] of the polynomial is mapped to the time interval
     * between the start and end time of the trajectory.
     */
    sb_poly_4d_t poly;

    /** @brief The polynomial that describes the first derivative of the current
     * trajectory segment.
     *
     * The interval [0; 1] of the polynomial is mapped to the time interval
     * between the start and end time of the trajectory.
     */
    sb_poly_4d_t dpoly;

    /** @brief The polynomial that describes the second derivative of the current
     * trajectory segment.
     *
     * The interval [0; 1] of the polynomial is mapped to the time interval
     * between the start and end time of the trajectory.
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

/**
 * Initializes a trajectory object from the contents of a Skybrush file in
 * binary format.
 *
 * \param trajectory  the trajectory to initialize
 * \param fd  handle to the low-level file object to initialize the trajectory from
 *
 * \return \c SB_SUCCESS if the object was initialized successfully,
 *         \c SB_ENOENT if the file did not contain a trajectory block,
 *         \c SB_EREAD for read errors
 */
sb_error_t sb_trajectory_init_from_binary_file(sb_trajectory_t* trajectory, int fd);

/**
 * Initializes a trajectory object from the contents of a Skybrush file in
 * binary format, already loaded into memory.
 *
 * \param trajectory  the trajectory to initialize
 * \param buf   the buffer holding the loaded Skybrush file in binary format
 * \param nbytes  the length of the buffer
 *
 * \return \c SB_SUCCESS if the object was initialized successfully,
 *         \c SB_ENOENT if the memory block did not contain a trajectory
 */
sb_error_t sb_trajectory_init_from_binary_file_in_memory(
    sb_trajectory_t* trajectory, uint8_t* buf, size_t nbytes);

/**
 * Initializes a trajectory object from the contents of a memory buffer.
 *
 * \param trajectory  the trajectory to initialize
 * \param buf   the buffer holding the encoded trajectory object
 * \param nbytes  the length of the buffer
 *
 * \return \c SB_SUCCESS if the object was initialized successfully,
 *         \c SB_ENOENT if the memory buffer did not contain a trajectory
 */
sb_error_t sb_trajectory_init_from_buffer(sb_trajectory_t* trajectory,
    uint8_t* buf, size_t nbytes);

/**
 * Initializes an empty trajectory.
 */
sb_error_t sb_trajectory_init_empty(sb_trajectory_t* trajectory);

/**
 * Destroys a trajectory object and releases all memory that it owns.
 */
void sb_trajectory_destroy(sb_trajectory_t* trajectory);

/**
 * Clears the trajectory object and removes all segments from it. Also releases
 * any memory that the trajectory owns.
 */
sb_error_t sb_trajectory_clear(sb_trajectory_t* trajectory);

/**
 * Returns whether the trajectory is empty (i.e. has no start position yet).
 */
sb_bool_t sb_trajectory_is_empty(const sb_trajectory_t* trajectory);

/**
 * Returns the axis-aligned bounding box of the trajectory.
 */
sb_error_t sb_trajectory_get_axis_aligned_bounding_box(
    const sb_trajectory_t* trajectory, sb_bounding_box_t* result);

/**
 * Returns the end position of the trajectory.
 */
sb_error_t sb_trajectory_get_end_position(
    const sb_trajectory_t* trajectory, sb_vector3_with_yaw_t* result);

/**
 * Returns the start position of the trajectory.
 */
sb_error_t sb_trajectory_get_start_position(
    const sb_trajectory_t* trajectory, sb_vector3_with_yaw_t* result);

/**
 * Returns the total duration of the trajectory, in milliseconds.
 */
uint32_t sb_trajectory_get_total_duration_msec(const sb_trajectory_t* trajectory);

/**
 * Returns the total duration of the trajectory, in seconds.
 */
float sb_trajectory_get_total_duration_sec(const sb_trajectory_t* trajectory);

/**
 * Proposes a takeoff time for the trajectory.
 *
 * The function assumes that the trajectory is specified in some common
 * coordinate system, the drone is initially placed at the first point of the
 * trajectory and it can take off by moving along the Z axis with a constant
 * speed until it reaches a specified altitude _relative to the first point_
 * of the trajectory.
 *
 * \param  trajectory  the trajectory to process
 * \param  min_ascent  the minimum ascent to perform during the takeoff
 * \param  speed       the assumed speed of the takeoff, in Z units per second
 * \return the proposed time when the takeoff command has to be sent to the
 *         drone, or infinity if the trajectory never reaches an altitude that
 *         is above the starting point by the given ascent
 */
float sb_trajectory_propose_takeoff_time_sec(
    const sb_trajectory_t* trajectory, float min_ascent, float speed);

/**
 * Proposes a landing time for the trajectory.
 *
 * The function assumes that the trajectory is specified in some common
 * coordinate system, the drone must land at the last point of the trajectory,
 * it can land by moving downwards along the Z axis with a constant speed and
 * it must start landing above a specified altitude _relative to the last point_
 * of the trajectory.
 *
 * \param  trajectory  the trajectory to process
 * \param  min_descent the minimum descent to perform during the landing
 * \return the proposed time when the landing command has to be sent to the
 *         drone, or negative infinity if the trajectory never reaches an
 *         altitude that is above the last point by the given descent
 */
float sb_trajectory_propose_landing_time_sec(
    const sb_trajectory_t* trajectory, float min_descent);

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
        size_t length; /**< Length of the current segment in the buffer */
        sb_trajectory_segment_t data; /**< The current segment of the trajectory */
    } current_segment;
} sb_trajectory_player_t;

/* ************************************************************************* */

/**
 * Initializes a trajectory player that plays the given trajectory.
 */
sb_error_t sb_trajectory_player_init(sb_trajectory_player_t* player, const sb_trajectory_t* trajectory);

/**
 * Destroys a trajectory player object and releases all memory that it owns.
 */
void sb_trajectory_player_destroy(sb_trajectory_player_t* player);

/**
 * Builds the next segment in the trajectory player. Used to move on to the next
 * segment during an iteration over the segments of the trajectory.
 */
sb_error_t sb_trajectory_player_build_next_segment(sb_trajectory_player_t* player);

/**
 * Dumps the details of the current trajectory segment for debugging purposes.
 */
void sb_trajectory_player_dump_current_segment(const sb_trajectory_player_t* player);

/**
 * Returns a pointer to the current trajectory segment of thr trajectory player.
 */
const sb_trajectory_segment_t* sb_trajectory_player_get_current_segment(
    const sb_trajectory_player_t* player);

/**
 * Returns the position on the trajectory associated to the player at the given
 * time instant.
 */
sb_error_t sb_trajectory_player_get_position_at(
    sb_trajectory_player_t* player, float t, sb_vector3_with_yaw_t* result);

/**
 * Returns the velocity on the trajectory associated to the player at the given
 * time instant.
 */
sb_error_t sb_trajectory_player_get_velocity_at(
    sb_trajectory_player_t* player, float t, sb_vector3_with_yaw_t* result);

/**
 * Returns the acceleration on the trajectory associated to the player at the given
 * time instant.
 */
sb_error_t sb_trajectory_player_get_acceleration_at(
    sb_trajectory_player_t* player, float t, sb_vector3_with_yaw_t* result);

/**
 * Returns the total duration of the trajectory associated to the player, in seconds.
 */
sb_error_t sb_trajectory_player_get_total_duration_msec(
    sb_trajectory_player_t* player, uint32_t* duration);

/**
 * Returns whether the trajectory player has more segments to play. Used to detect
 * the end of iteration when iterating over the segments of the trajectory.
 */
sb_bool_t sb_trajectory_player_has_more_segments(const sb_trajectory_player_t* player);

__END_DECLS

#endif
