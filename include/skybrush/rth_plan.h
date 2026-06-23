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

#ifndef SKYBRUSH_RTH_PLAN_H
#define SKYBRUSH_RTH_PLAN_H

#include <skybrush/decls.h>
#include <skybrush/error.h>
#include <skybrush/refcount.h>
#include <skybrush/trajectory.h>

__BEGIN_DECLS

/**
 * @file rth_plan.h
 * @brief Handling of collective return-to-home plans in Skybrush missions.
 */

/**
 * Enum representing which of the RTH plan entry parameter should be considered
 * when evaluating the RTH plan at a given time instant.
 */
typedef enum {
    SB_RTH_PLAN_ENTRY_HAS_NECK = (1 << 0),
    SB_RTH_PLAN_ENTRY_HAS_TARGET_XY = (1 << 1),
    SB_RTH_PLAN_ENTRY_HAS_TARGET_Z = (1 << 2),
    SB_RTH_PLAN_ENTRY_HAS_LANDING_ALTITUDE = (1 << 3),
    SB_RTH_PLAN_ENTRY_HAS_PRE_DELAY = (1 << 4),
    SB_RTH_PLAN_ENTRY_HAS_POST_DELAY = (1 << 5),
} sb_rth_plan_entry_flag_t;

/**
 * @brief Structure describing the return-to-home action corresponding to a
 * given time instant during the mission.
 */
typedef struct sb_rth_plan_entry_s {
    /** The timestamp when the action should start */
    float time_sec;

    /**
     * The duration of the main segment of the action, in seconds, _not_ including
     * the pre-delay, post-delay, neck and landing phases.
     */
    float duration_sec;

    /**
     * Flags indicating which parameters are valid for this entry.
     *
     * The following flags are always set because we provide default values for them
     * even if the binary representation of the RTH plan did not specify them:
     *
     * - \c SB_RTH_PLAN_ENTRY_HAS_LANDING_ALTITUDE (default value comes from the plan)
     * - \c SB_RTH_PLAN_ENTRY_HAS_PRE_DELAY (default value is 0)
     * - \c SB_RTH_PLAN_ENTRY_HAS_POST_DELAY (default value is 0)
     * - \c SB_RTH_PLAN_ENTRY_HAS_NECK (default value is a neck with zero duration and length)
     */
    sb_rth_plan_entry_flag_t flags;

    /**
     * The XY landing target of the action, if the \c SB_RTH_PLAN_ENTRY_HAS_TARGET_XY
     * bit is set in the flags.
     */
    sb_vector2_t landing_target;

    /**
     * The arrival altitude above the target, if the \c SB_RTH_PLAN_ENTRY_HAS_TARGET_Z
     * bit is set in the flags.
     */
    float arrival_altitude;

    /** Delay to add \em before the action, in seconds */
    float pre_delay_sec;

    /** Optional delay to add \em after the action, in seconds */
    float post_delay_sec;

    /** Optional vertical neck to add \em before the action */
    float pre_neck;

    /** The duration of the pre-neck phase, in seconds */
    float pre_neck_duration_sec;

    /**
     * Maximum acceleration constraint to apply during the action, on a best-effort
     * basis, in units per second squared. Infinity, zero, NaN or a negative value
     * means no constraint.
     */
    float max_acceleration;

    /**
     * Optional landing velocity for the descent from the arrival altitude above the
     * target to the target itself. Zero, negative, NaN or infinity means that no
     * descent will be added.
     */
    float landing_velocity;

    /**
     * The landing altitude of the action.
     */
    float landing_altitude;
} sb_rth_plan_entry_t;

/**
 * @brief Structure representing a return-to-home plan in a Skybrush mission.
 */
typedef struct sb_rth_plan_s {
    SB_REFCOUNTED;
    sb_buffer_t buffer; /**< Buffer holding the binary representation of the RTH plan */
    float scale; /**< Scaling factor for the coordinates */
    size_t header_length; /**< Number of bytes in the header of the buffer */
    size_t num_points; /**< Number of points in the RTH plan */
    float max_acceleration; /**< Maximum acceleration during RTH actions */
    float landing_velocity; /**< Optional landing velocity for RTH actions */
    float landing_altitude; /**< Default landing altitude for RTH actions */
} sb_rth_plan_t;

void sb_rth_plan_entry_clear(sb_rth_plan_entry_t* entry, const sb_rth_plan_t* plan, float time_sec);

sb_rth_plan_t* sb_rth_plan_new(void);
sb_error_t sb_rth_plan_init(sb_rth_plan_t* plan);

float sb_rth_plan_get_default_acceleration_limit(const sb_rth_plan_t* plan);
float sb_rth_plan_get_default_landing_altitude(const sb_rth_plan_t* plan);
float sb_rth_plan_get_default_landing_velocity(const sb_rth_plan_t* plan);
size_t sb_rth_plan_get_num_entries(const sb_rth_plan_t* plan);
size_t sb_rth_plan_get_num_points(const sb_rth_plan_t* plan);
sb_error_t sb_rth_plan_get_point(const sb_rth_plan_t* plan, size_t index, sb_vector2_t* point);
sb_bool_t sb_rth_plan_has_default_landing_velocity(const sb_rth_plan_t* plan);
sb_bool_t sb_rth_plan_is_empty(const sb_rth_plan_t* plan);

sb_error_t sb_rth_plan_evaluate_at(const sb_rth_plan_t* plan, float time, sb_rth_plan_entry_t* result);

void sb_rth_plan_clear_default_landing_velocity(sb_rth_plan_t* plan);
void sb_rth_plan_set_default_landing_altitude(sb_rth_plan_t* plan, float landing_altitude);
void sb_rth_plan_set_default_landing_velocity(sb_rth_plan_t* plan, float landing_velocity);
void sb_rth_plan_set_default_acceleration_limit(sb_rth_plan_t* plan, float max_acceleration);

sb_error_t sb_rth_plan_update_from_binary_file(sb_rth_plan_t* plan, int fd);
sb_error_t sb_rth_plan_update_from_binary_file_in_memory(
    sb_rth_plan_t* plan, uint8_t* buf, size_t nbytes);
sb_error_t sb_rth_plan_update_from_buffer(sb_rth_plan_t* plan,
    uint8_t* buf, size_t nbytes);
sb_error_t sb_rth_plan_update_from_bytes(sb_rth_plan_t* plan,
    uint8_t* buf, size_t nbytes);

sb_error_t sb_trajectory_update_from_rth_plan_entry(
    sb_trajectory_t* trajectory, const sb_rth_plan_entry_t* entry,
    sb_vector3_t start);

__END_DECLS

#endif
