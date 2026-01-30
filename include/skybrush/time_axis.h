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

#ifndef SKYBRUSH_TIME_AXIS_H
#define SKYBRUSH_TIME_AXIS_H

/**
 * @file time_warping.h
 * @brief Functions and structures to define a time axis where time flows at different
 * rates in relation to wall clock time (which is assumed to flow at a constant rate).
 */

#include <skybrush/decls.h>
#include <skybrush/error.h>
#include <stddef.h>
#include <stdint.h>

__BEGIN_DECLS

/**
 * @brief Structure representing a single segment of a time axis where time
 * can flow at different rates in relation to wall clock time.
 *
 * The combination of \c initial_time_derivative_sec and \c final_rate_sec can be used
 * to create sections that have constant speed (when both values are equal) or where
 * time accelerates or decelerates (when the two values are different). For instance,
 * warped time can be made to stop at the end of a segment by setting \c final_rate_sec
 * to zero.
 */
typedef struct sb_time_segment_s {
    /**
     * Duration of this segment, in milliseconds, in wall clock time.
     * \c UINT32_MAX means infinite duration.
     */
    uint32_t duration_msec;

    /**
     * Initial time scaling factor for this entry, in seconds/seconds.
     *
     * A value of 1.0 means real time (i.e. one second in warped time is one second in
     * wall clock time). A value of 2.0 means double speed (i.e. one second in
     * warped time is half a second in wall clock time) and so on.
     *
     * Must be non-negative, i.e. we do not allow warped time to flow backwards.
     * (There are limits to our own superpowers after all.)
     */
    float initial_rate;

    /**
     * Final time scaling factor for this entry, in seconds/seconds.
     *
     * A value of 1.0 means real time (i.e. one second in warped time is one second in
     * wall clock time). A value of 2.0 means double speed (i.e. one second in
     * warped time is half a second in wall clock time) and so on.
     *
     * Must be non-negative, i.e. we do not allow warped time to flow backwards.
     * (There are limits to our own superpowers after all.)
     */
    float final_rate;
} sb_time_segment_t;

/* Constructors for common time-segment patterns */

sb_time_segment_t sb_time_segment_make(uint32_t duration_msec, float initial_rate, float final_rate);
sb_time_segment_t sb_time_segment_make_realtime(uint32_t duration_msec);
sb_time_segment_t sb_time_segment_make_constant_rate(uint32_t duration_msec, float rate);
sb_time_segment_t sb_time_segment_make_slowdown_from(uint32_t duration_msec, float initial_rate);
sb_time_segment_t sb_time_segment_make_slowdown_from_realtime(uint32_t duration_msec);
sb_time_segment_t sb_time_segment_make_spinup_to(uint32_t duration_msec, float final_rate);
sb_time_segment_t sb_time_segment_make_spinup_to_realtime(uint32_t duration_msec);

uint32_t sb_time_segment_get_duration_in_wall_clock_time_msec(const sb_time_segment_t* segment);
float sb_time_segment_get_duration_in_wall_clock_time_sec(const sb_time_segment_t* segment);
float sb_time_segment_get_duration_in_warped_time_sec(const sb_time_segment_t* segment);

/**
 * @brief Structure representing a time axis where time can flow at different
 * rates in relation to wall clock time.
 *
 * A time axis consists of a sequence of multiple time segments where each segment has
 * a duration, an initial and a final time scaling factor. The combination of these
 * parameters allows to create sections where time flows at constant speed, accelerates
 * (with constant acceleration) or decelerates (with constant deceleration).
 *
 * The time axis also has an origin, which is the wall clock time corresponding to
 * warped time zero.
 *
 * Wall clock time is measured in millseconds from some arbitrary epoch (e.g. system
 * start time), while warped time is measured in seconds from the origin. The choice
 * for the wall clock time being in milliseconds was chosen pragmatically to make the
 * integration easier with systems like ArduPilot where time is naturally measured in
 * milliseconds.
 */
typedef struct sb_time_axis_s {
    /**
     * Origin of the time axis, in milliseconds. This is the wall clock time
     * corresponding to warped time zero. May be negative if the origin is before
     * the epoch of the wall clock time.
     */
    int32_t origin_msec;

    sb_time_segment_t* stor_begin; /**< Pointer to the first time segment */
    sb_time_segment_t* end; /**< Pointer to one past the last used time segment */
    sb_time_segment_t* stor_end; /**< Pointer to one past the last allocated time segment */
} sb_time_axis_t;

sb_error_t sb_time_axis_init(sb_time_axis_t* axis);
void sb_time_axis_destroy(sb_time_axis_t* axis);

void sb_time_axis_swap(sb_time_axis_t* axis, sb_time_axis_t* other);

size_t sb_time_axis_capacity(const sb_time_axis_t* axis);
const sb_time_segment_t* sb_time_axis_get_segment(
    const sb_time_axis_t* axis, size_t index);
size_t sb_time_axis_num_segments(const sb_time_axis_t* axis);

int32_t sb_time_axis_get_origin_msec(const sb_time_axis_t* axis);
float sb_time_axis_get_origin_sec(const sb_time_axis_t* axis);
void sb_time_axis_set_origin_msec(sb_time_axis_t* axis, int32_t origin_msec);
sb_error_t sb_time_axis_set_origin_sec(sb_time_axis_t* axis, float origin_sec);

uint32_t sb_time_axis_get_total_duration_msec(const sb_time_axis_t* axis);
float sb_time_axis_get_total_duration_sec(const sb_time_axis_t* axis);

void sb_time_axis_clear(sb_time_axis_t* axis);
sb_error_t sb_time_axis_append_segment(
    sb_time_axis_t* axis, sb_time_segment_t segment);
sb_error_t sb_time_axis_insert_segment_at(
    sb_time_axis_t* axis, size_t index, sb_time_segment_t segment);
sb_error_t sb_time_axis_remove_segment_at(
    sb_time_axis_t* axis, size_t index);

float sb_time_axis_map(const sb_time_axis_t* axis, int32_t wall_clock_time_msec);
float sb_time_axis_map_ex(const sb_time_axis_t* axis, int32_t wall_clock_time_msec, float* out_rate);

__END_DECLS

#endif // SKYBRUSH_TIME_AXIS_H
