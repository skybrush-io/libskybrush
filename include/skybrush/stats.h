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

#ifndef SKYBRUSH_STATS_H
#define SKYBRUSH_STATS_H

/**
 * @file stats.h
 * @brief Functions and structures related to \c libskybrush trajectory statistics
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <skybrush/basic_types.h>
#include <skybrush/buffer.h>
#include <skybrush/error.h>
#include <skybrush/poly.h>

#include <skybrush/decls.h>

struct sb_trajectory_s;

__BEGIN_DECLS

/* ************************************************************************* */

/**
 * Structure holding basic statistics about a trajectory that can be gathered
 * while iterating over it once.
 */
typedef struct sb_trajectory_stats_s {
    /** Total duration, in milliseconds */
    uint32_t duration_msec;

    /** Total duration, in seconds */
    float duration_sec;

    /** Earliest time above the takeoff altitude, in seconds */
    float earliest_above_sec;

    /**
     * Proposed takeoff time, in seconds. Infinity if it has not been
     * calculated or when the entire trajectory is below the takeoff altitude.
     */
    float takeoff_time_sec;

    /**
     * Proposed landing time, in seconds. Infinity if it has not been
     * calculated.
     */
    float landing_time_sec;

    /**
     * Position at landing time. Infinity if it has not been calculated.
     */
    sb_vector3_with_yaw_t pos_at_landing_time;

    /**
     * Velocity at landing time. Infinity if it has not been calculated.
     */
    sb_vector3_with_yaw_t vel_at_landing_time;

    /** Distance between first and last point of trajectory, in the XY plane */
    float start_to_end_distance_xy;
} sb_trajectory_stats_t;

/* ************************************************************************* */

sb_error_t sb_trajectory_stats_init(sb_trajectory_stats_t* stats);
void sb_trajectory_stats_destroy(sb_trajectory_stats_t* stats);

/* ************************************************************************* */

/**
 * \brief Flags that specify what to calculate in the trajectory statistics.
 */
typedef enum {
    SB_TRAJECTORY_STATS_NONE = 0,
    SB_TRAJECTORY_STATS_DURATION = 1,
    SB_TRAJECTORY_STATS_START_END_DISTANCE = 2,
    SB_TRAJECTORY_STATS_TAKEOFF_TIME = 4,
    SB_TRAJECTORY_STATS_LANDING_TIME = 8,

    /* clang-format off */
    SB_TRAJECTORY_STATS_ALL = (
        SB_TRAJECTORY_STATS_DURATION |
        SB_TRAJECTORY_STATS_START_END_DISTANCE |
        SB_TRAJECTORY_STATS_TAKEOFF_TIME |
        SB_TRAJECTORY_STATS_LANDING_TIME
    )
    /* clang-format on */
} sb_trajectory_stat_components_t;

/**
 * Structure containing the configuration of the parameters of the
 * trajectory statistics calculation.
 */
typedef struct sb_trajectory_stats_calculator_s {
    /**
     * Specifies which components of the statistics to calculate.
     */
    sb_trajectory_stat_components_t components;

    /**
     * Assumed takeoff speed of the drone, in units per second.
     */
    float takeoff_speed;

    /**
     * Assumed acceleration of the drone, in units per second squared,
     * used vertically during takeoff.
     */
    float acceleration;

    /**
     * Minimum ascent required for a takeoff.
     */
    float min_ascent;

    /**
     * Preferred length of the descent in the landing phase when the drone is
     * in an autonomous landing mode (and not following the trajectory).
     */
    float preferred_descent;

    /**
     * Threshold in the XY plane that is used to decide whether a
     * trajectory segment is vertical.
     */
    float verticality_threshold;
} sb_trajectory_stats_calculator_t;

/* ************************************************************************* */

sb_error_t sb_trajectory_stats_calculator_init(sb_trajectory_stats_calculator_t* calc, float scale);
void sb_trajectory_stats_calculator_destroy(sb_trajectory_stats_calculator_t* calc);
void sb_trajectory_stats_calculator_set_components(
    sb_trajectory_stats_calculator_t* calc, sb_trajectory_stat_components_t components);
sb_error_t sb_trajectory_stats_calculator_run(
    const sb_trajectory_stats_calculator_t* calc,
    const struct sb_trajectory_s* trajectory,
    sb_trajectory_stats_t* result);

__END_DECLS

#endif
