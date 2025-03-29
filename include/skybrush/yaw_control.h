/*
 * This file is part of libskybrush.
 *
 * Copyright 2020-2025 CollMot Robotics Ltd.
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

#ifndef SKYBRUSH_YAW_CONTROL_H
#define SKYBRUSH_YAW_CONTROL_H

#include <skybrush/buffer.h>

#include <skybrush/decls.h>
#include <skybrush/error.h>

__BEGIN_DECLS

/**
 * @file yaw_control.h
 * @brief Handling of yaw control in Skybrush missions.
 */

/**
 * @brief Structure describing a single relative yaw setpoint.
 */
typedef struct sb_yaw_setpoint_s {
    /** The timestamp when the setpoint should start, in seconds */
    float start_time_sec;

    /** The timestamp when the setpoint should start, in milliseconds */
    uint32_t start_time_msec;

    /** The net duration of the setpoint, in seconds */
    float duration_sec;

    /** The net duration of the setpoint, in milliseconds */
    uint16_t duration_msec;

    /** The timestamp when the setpoint should end, in seconds */
    float end_time_sec;

    /** The timestamp when the setpoint should end, in milliseconds */
    uint32_t end_time_msec;

    /** The starting yaw of the setpoint, in 1/10th of degrees */
    int32_t start_yaw_ddeg;

    /** The starting yaw of the setpoint, in degrees */
    float start_yaw_deg;

    /** The amount of yaw change during the setpoint, in 1/10th of degrees */
    int16_t yaw_change_ddeg;

    /** The amount of yaw change during the setpoint, in degrees */
    float yaw_change_deg;

    /** The ending yaw of the setpoint, in 1/10th of degrees */
    int32_t end_yaw_ddeg;

    /** The ending yaw of the setpoint, in degrees */
    float end_yaw_deg;
} sb_yaw_setpoint_t;

/**
 * @brief Structure representing the yaw control deltas in a Skybrush mission.
 */
typedef struct sb_yaw_control_s {
    sb_buffer_t buffer; /**< The buffer holding the yaw control data */

    size_t header_length; /**< Number of bytes in the header of the buffer */
    size_t num_deltas; /**< Number of yaw deltas in the yaw control object */
    sb_bool_t auto_yaw; /**< Whether auto yaw mode is in use */
    int16_t yaw_offset_ddeg; /**< The yaw offset used by the yaw control object, in 1/10th of degrees */
} sb_yaw_control_t;

sb_error_t sb_yaw_control_init_from_binary_file(sb_yaw_control_t* ctrl, int fd);
sb_error_t sb_yaw_control_init_from_binary_file_in_memory(sb_yaw_control_t* ctrl, uint8_t* buf, size_t nbytes);
sb_error_t sb_yaw_control_init_from_buffer(sb_yaw_control_t* ctrl, uint8_t* buf, size_t nbytes);
sb_error_t sb_yaw_control_init_empty(sb_yaw_control_t* ctrl);
void sb_yaw_control_destroy(sb_yaw_control_t* ctrl);
sb_bool_t sb_yaw_control_is_empty(const sb_yaw_control_t* ctrl);

/* ************************************************************************* */

/**
 * Structure representing a yaw control player that allows us to query the
 * yaw and yaw rate along the yaw control curve.
 */
typedef struct sb_yaw_player_s {
    const sb_yaw_control_t* ctrl; /**< The yaw control object that the player plays */

    /** The current setpoint that is being evaluated by the player */
    struct
    {
        size_t start; /**< Start offset of the current setpoint */
        size_t length; /**< Length of the current setpoint in the buffer */
        sb_yaw_setpoint_t data; /**< The current setpoint of the yaw control object */
    } current_setpoint;
} sb_yaw_player_t;

sb_error_t sb_yaw_player_init(sb_yaw_player_t* player, const sb_yaw_control_t* ctrl);
void sb_yaw_player_destroy(sb_yaw_player_t* player);
sb_error_t sb_yaw_player_build_next_setpoint(sb_yaw_player_t* player);
void sb_yaw_player_dump_current_segment(const sb_yaw_player_t* player);
const sb_yaw_setpoint_t* sb_yaw_player_get_current_setpoint(const sb_yaw_player_t* player);
sb_error_t sb_yaw_player_get_yaw_at(sb_yaw_player_t* player, float t, float* result);
sb_error_t sb_yaw_player_get_yaw_rate_at(sb_yaw_player_t* player, float t, float* result);
sb_error_t sb_yaw_player_get_total_duration_msec(sb_yaw_player_t* player, uint32_t* duration);
sb_bool_t sb_yaw_player_has_more_setpoints(const sb_yaw_player_t* player);

__END_DECLS

#endif
