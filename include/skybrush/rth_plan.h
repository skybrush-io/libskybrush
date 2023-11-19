/*
 * This file is part of libskybrush.
 *
 * Copyright 2020-2023 CollMot Robotics Ltd.
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

#ifndef SKYBRUSH_RTH_PLAN_H
#define SKYBRUSH_RTH_PLAN_H

#include <skybrush/decls.h>
#include <skybrush/error.h>
#include <skybrush/trajectory.h>

__BEGIN_DECLS

/**
 * @file rth_plan.h
 * @brief Handling of collective return-to-home plans in Skybrush missions.
 */

/**
 * @brief Enum containing the types of the possible actions that we support
 * during a collective return-to-home maneuver.
 */
typedef enum {
    /** Same as previous entry */
    SB_RTH_ACTION_SAME_AS_PREVIOUS = 0,

    /** Land in place */
    SB_RTH_ACTION_LAND = 1,

    /** Go to the target of the action, keeping altitude */
    SB_RTH_ACTION_GO_TO_KEEPING_ALTITUDE = 2,

    /** Go to the target of the action in a straight line, optionally preceded
     * by a vertical neck */
    SB_RTH_ACTION_GO_TO_WITH_ALTITUDE = 3
} sb_rth_action_t;

/**
 * @brief Structure describing the return-to-home action corresponding to a
 * given time instant during the mission.
 */
typedef struct sb_rth_plan_entry_s {
    /** The timestamp when the action should start */
    float time_sec;

    /** The action type to perform */
    sb_rth_action_t action;

    /** The net duration of the action, in seconds */
    float duration_sec;

    /** The (horizontal) target of the action; coordinates are in millimeters */
    sb_vector2_t target;

    /** The altitude of the target, if neeeded; coordinates are in millimeters */
    float target_altitude;

    /** Optional delay to add \em before the action, in seconds */
    float pre_delay_sec;

    /** Optional delay to add \em after the action, in seconds */
    float post_delay_sec;

    /** Optional vertical neck to add \em before the action, after the pre delay,
     * in millimeters. Applicable only to SB_RTH_ACTION_GO_TO_WITH_ALTITUDE */
    float pre_neck_mm;

    /** The duration of the pre-neck phase, in seconds */
    float pre_neck_duration_sec;
} sb_rth_plan_entry_t;

/**
 * @brief Structure representing a return-to-home plan in a Skybrush mission.
 */
typedef struct sb_rth_plan_s {
    uint8_t* buffer; /**< Pointer to the buffer holding the RTH plan */
    size_t buffer_length; /**< Number of bytes in the buffer */
    sb_bool_t owner; /**< Whether the object owns the buffer */

    float scale; /**< Scaling factor for the coordinates */
    size_t header_length; /**< Number of bytes in the header of the buffer */
    size_t num_points; /**< Number of points in the RTH plan */
} sb_rth_plan_t;

sb_error_t sb_rth_plan_init_from_binary_file(sb_rth_plan_t* plan, int fd);
sb_error_t sb_rth_plan_init_from_binary_file_in_memory(
    sb_rth_plan_t* plan, uint8_t* buf, size_t nbytes);
sb_error_t sb_rth_plan_init_from_buffer(sb_rth_plan_t* plan,
    uint8_t* buf, size_t nbytes);
sb_error_t sb_rth_plan_init_empty(sb_rth_plan_t* plan);
void sb_rth_plan_destroy(sb_rth_plan_t* plan);
size_t sb_rth_plan_get_num_entries(const sb_rth_plan_t* plan);
size_t sb_rth_plan_get_num_points(const sb_rth_plan_t* plan);
sb_error_t sb_rth_plan_get_point(const sb_rth_plan_t* plan, size_t index, sb_vector2_t* point);
sb_bool_t sb_rth_plan_is_empty(const sb_rth_plan_t* plan);
sb_error_t sb_rth_plan_evaluate_at(const sb_rth_plan_t* plan, float time, sb_rth_plan_entry_t* result);

sb_error_t sb_trajectory_init_from_rth_plan_entry(
    sb_trajectory_t* trajectory,
    const sb_rth_plan_entry_t* entry,
    sb_vector3_with_yaw_t start);

__END_DECLS

#endif
