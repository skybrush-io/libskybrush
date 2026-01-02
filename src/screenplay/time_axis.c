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

#include <assert.h>
#include <math.h>
#include <skybrush/memory.h>
#include <skybrush/time_axis.h>
#include <string.h>

static sb_error_t sb_i_time_segment_validate(const sb_time_segment_t* seg);
static sb_error_t sb_i_time_axis_grow_if_needed(sb_time_axis_t* axis);

#define ASSERT_PRECONDITIONS() \
    assert(axis != 0);         \
    assert(axis->stor_begin != 0);

/* ************************************************************************** */
/* Convenient constructors for common sb_time_segment_t patterns              */
/* ************************************************************************** */

/**
 * @brief Creates a new time segment with a specific duration and initial and
 * final rate.
 *
 * @param duration_sec The duration of the segment
 * @param initial_rate The initial rate of the segment
 * @param final_rate The final rate of the segment
 */
sb_time_segment_t sb_time_segment_make(float duration_sec, float initial_rate, float final_rate)
{
    sb_time_segment_t seg;
    seg.duration_sec = duration_sec;
    seg.initial_rate = initial_rate;
    seg.final_rate = final_rate;
    return seg;
}

/**
 * @brief Creaates a new time segment with a specific duration running in real-time.
 *
 * @param duration_sec The duration of the segment
 */
sb_time_segment_t sb_time_segment_make_realtime(float duration_sec)
{
    return sb_time_segment_make(duration_sec, 1.0f, 1.0f);
}

/**
 * @brief Creates a new time segment with a specific duration running at a given
 * constant rate.
 *
 * @param duration_sec The duration of the segment
 * @param rate the constant rate of the segment
 */
sb_time_segment_t sb_time_segment_make_constant_rate(float duration_sec, float rate)
{
    return sb_time_segment_make(duration_sec, rate, rate);
}

/**
 * @brief Creates a new time segment with a specific duration starting at a given
 * initial rate, slowing down to a standstill.
 *
 * @param initial_rate the starting rate of the segment
 */
sb_time_segment_t sb_time_segment_make_slowdown_from(float duration_sec, float initial_rate)
{
    return sb_time_segment_make(duration_sec, initial_rate, 0.0f);
}

/**
 * @brief Creates a new time segment with a specific duration starting from a standstill,
 * speeding up to a given final rate.
 *
 * @param final_rate the final rate of the segment
 */
sb_time_segment_t sb_time_segment_make_spinup_to(float duration_sec, float final_rate)
{
    return sb_time_segment_make(duration_sec, 0.0f, final_rate);
}

/**
 * @brief Creates a new time segment with a specific duration starting real-time
 * (rate = 1.0) and slowing down to a standstill.
 */
sb_time_segment_t sb_time_segment_make_slowdown_from_realtime(float duration_sec)
{
    return sb_time_segment_make_slowdown_from(duration_sec, 1.0f);
}

/**
 * @brief Creates a new time segment with a specific duration starting from a standstill,
 * speeding up to real-time.
 */
sb_time_segment_t sb_time_segment_make_spinup_to_realtime(float duration_sec)
{
    return sb_time_segment_make_spinup_to(duration_sec, 1.0f);
}

/**
 * @brief Appends a new segment to the time axis.
 *
 * @param axis Pointer to the time axis structure.
 * @param segment The time segment to append. It will be copied into the time axis.
 * @return \c SB_SUCCESS on success, or an error code on failure.
 */
sb_error_t sb_time_axis_append_segment(
    sb_time_axis_t* axis, sb_time_segment_t segment)
{
    return sb_time_axis_insert_segment_at(axis, sb_time_axis_num_segments(axis), segment);
}

/**
 * @brief Returns the duration of the time segment in wall clock time, in seconds.
 *
 * @param segment Pointer to the time segment.
 * @return Duration of the segment in wall clock time, in seconds.
 */
float sb_time_segment_get_duration_in_wall_clock_time_sec(const sb_time_segment_t* segment)
{
    return segment->duration_sec;
}

/**
 * @brief Returns the duration of the time segment in warped time, in seconds.
 *
 * @param segment Pointer to the time segment.
 * @return Duration of the segment in warped time, in seconds.
 */
float sb_time_segment_get_duration_in_warped_time_sec(const sb_time_segment_t* segment)
{
    float avg_rate = (segment->initial_rate + segment->final_rate) * 0.5f;
    return sb_time_segment_get_duration_in_wall_clock_time_sec(segment) * avg_rate;
}

/* ********************************************************************************** */

static sb_error_t sb_i_time_segment_validate(const sb_time_segment_t* seg)
{
    if (seg == NULL) {
        return SB_EINVAL; /* LCOV_EXCL_LINE */
    }

    /* Reject NaN values */
    if (isnan(seg->duration_sec) || isnan(seg->initial_rate) || isnan(seg->final_rate)) {
        return SB_EINVAL;
    }

    /* Duration may be infinite, but must not be negative. Rates must be non-negative. */
    if (seg->duration_sec < 0.0f || seg->initial_rate < 0.0f || seg->final_rate < 0.0f) {
        return SB_EINVAL;
    }

    return SB_SUCCESS;
}

/* ********************************************************************************** */

/**
 * @brief Initializes a time axis structure.
 *
 * @param axis Pointer to the time axis structure to initialize.
 * @return \c SB_SUCCESS on success, or an error code on failure.
 */
sb_error_t sb_time_axis_init(sb_time_axis_t* axis)
{
    const size_t initial_size = 0;
    const size_t alloc_size = initial_size > 4 ? initial_size : 4;

    axis->stor_begin = sb_calloc(sb_time_segment_t, alloc_size);
    if (axis->stor_begin == 0) {
        return SB_ENOMEM; /* LCOV_EXCL_LINE */
    }

    axis->stor_end = axis->stor_begin + alloc_size;
    axis->end = axis->stor_begin + initial_size;

    return SB_SUCCESS;
}

/**
 * @brief Destroys a time axis structure, freeing any allocated resources.
 *
 * @param axis Pointer to the time axis structure to destroy.
 */
void sb_time_axis_destroy(sb_time_axis_t* axis)
{
    if (axis->stor_begin != 0) {
        sb_free(axis->stor_begin);
        axis->stor_begin = 0;
    }

    axis->stor_end = 0;
    axis->end = 0;
}

/**
 * @brief Returns the number of segments in the time axis.
 *
 * @param axis Pointer to the time axis structure.
 * @return Number of segments in the time axis.
 */
size_t sb_time_axis_num_segments(const sb_time_axis_t* axis)
{
    ASSERT_PRECONDITIONS();
    return axis->end - axis->stor_begin;
}

/**
 * @brief Returns the current capacity (number of allocated slots) of the time axis
 * storage.
 *
 * @param axis Pointer to the time axis structure.
 * @return Number of allocated slots in the time axis structure.
 */
size_t sb_time_axis_capacity(const sb_time_axis_t* axis)
{
    ASSERT_PRECONDITIONS();
    return axis->stor_end - axis->stor_begin;
}

/**
 * @brief Returns a pointer to the time segment at the given index.
 *
 * Do not modify the returned segment. If you want to tweak a segment, remove it first
 * from the time axis, and add a new one.
 *
 * @param axis Pointer to the time axis structure.
 * @param index Index of the time segment to retrieve.
 * @return Pointer to the time segment at the given index, or NULL if the index
 *         is out of bounds.
 */
const sb_time_segment_t* sb_time_axis_get_segment(const sb_time_axis_t* axis, size_t index)
{
    ASSERT_PRECONDITIONS();
    size_t num_segments = sb_time_axis_num_segments(axis);
    return index >= num_segments ? NULL : &axis->stor_begin[index];
}

/**
 * @brief Clears all segments from the time axis.
 *
 * @param axis Pointer to the time axis structure.
 */
void sb_time_axis_clear(sb_time_axis_t* axis)
{
    ASSERT_PRECONDITIONS();
    axis->end = axis->stor_begin;
}

/**
 * @brief Inserts a new segment at the given index in the time axis.
 * @param axis Pointer to the time axis structure.
 * @param index Index at which to insert the new segment.
 * @param segment The time segment to insert. It will be copied into the time axis.
 * @return \c SB_SUCCESS on success, or an error code on failure.
 */
sb_error_t sb_time_axis_insert_segment_at(
    sb_time_axis_t* axis, size_t index, sb_time_segment_t segment)
{
    ASSERT_PRECONDITIONS();

    size_t num_segments = sb_time_axis_num_segments(axis);
    if (index > num_segments) {
        return SB_EINVAL;
    }

    SB_CHECK(sb_i_time_segment_validate(&segment));
    SB_CHECK(sb_i_time_axis_grow_if_needed(axis));

    /* Shift segments to make room for the new one */
    sb_time_segment_t* src = &axis->stor_begin[index];
    size_t num_to_move = (axis->end - src);
    if (num_to_move > 0) {
        sb_time_segment_t* dest = &axis->stor_begin[index + 1];
        memmove(dest, src, num_to_move * sizeof(sb_time_segment_t));
    }

    /* Insert the new segment */
    *src = segment;
    axis->end++;

    return SB_SUCCESS;
}

/**
 * @brief Removes the segment at the given index from the time axis.
 *
 * @param axis Pointer to the time axis structure.
 * @param index Index of the time segment to remove.
 * @return \c SB_SUCCESS on success, or an error code on failure.
 */
sb_error_t sb_time_axis_remove_segment_at(sb_time_axis_t* axis, size_t index)
{
    ASSERT_PRECONDITIONS();

    size_t num_segments = sb_time_axis_num_segments(axis);
    if (index >= num_segments) {
        return SB_EINVAL;
    }

    /* Shift segments to fill the gap */
    sb_time_segment_t* dest = &axis->stor_begin[index];
    sb_time_segment_t* src = &axis->stor_begin[index + 1];
    size_t num_to_move = (axis->end - src);
    if (num_to_move > 0) {
        memmove(dest, src, num_to_move * sizeof(sb_time_segment_t));
    }

    axis->end--;

    return SB_SUCCESS;
}

/**
 * @brief Maps some time instant in wall clock time to the corresponding warped time.
 *
 * @param axis Pointer to the time axis structure.
 * @param wall_clock_time_sec Wall clock time in seconds.
 * @return Corresponding warped time in seconds.
 */
float sb_time_axis_map(const sb_time_axis_t* axis, float wall_clock_time_sec)
{
    float accumulated_warped_time_sec = 0.0f;
    float warped_time_in_segment;
    const sb_time_segment_t* seg;
    size_t num_segments = sb_time_axis_num_segments(axis);

    ASSERT_PRECONDITIONS();

    /* We assume that the time axis starts at T=0 and we have at least one segment.
     * Otherwise the time axis is simply real-time.
     */
    if (wall_clock_time_sec < 0 || num_segments == 0) {
        return wall_clock_time_sec;
    }

    /* Infinity is mapped to infinity */
    if (!isfinite(wall_clock_time_sec)) {
        return wall_clock_time_sec;
    }

    for (size_t i = 0; i < num_segments; ++i) {
        seg = sb_time_axis_get_segment(axis, i);
        float seg_wall_clock_duration_sec = sb_time_segment_get_duration_in_wall_clock_time_sec(seg);

        if (seg_wall_clock_duration_sec >= wall_clock_time_sec) {
            /* The target time is within this segment */
            if (seg->initial_rate == seg->final_rate) {
                /* Constant rate segment */
                warped_time_in_segment = wall_clock_time_sec * seg->initial_rate;
            } else if (seg_wall_clock_duration_sec > 0) {
                /* Linearly changing rate segment */
                warped_time_in_segment = seg->initial_rate * wall_clock_time_sec + ((seg->final_rate - seg->initial_rate) / (2.0f * seg_wall_clock_duration_sec)) * wall_clock_time_sec * wall_clock_time_sec;
            } else {
                /* Zero-duration segment, prevent division by zero.
                 * Hard to cover with unit tests, but theoretically possible. */
                warped_time_in_segment = 0.0f; /* LCOV_EXCL_LINE */
            }

            return accumulated_warped_time_sec + warped_time_in_segment;
        } else {
            /* Move to the next segment */
            wall_clock_time_sec -= seg_wall_clock_duration_sec;
            float seg_warped_duration = sb_time_segment_get_duration_in_warped_time_sec(seg);
            accumulated_warped_time_sec += seg_warped_duration;
        }
    }

    /* Reached last segment. Pretend that time keeps on flowing with the final rate
     * of the last segment.
     */
    assert(num_segments > 0);
    seg = sb_time_axis_get_segment(axis, num_segments - 1);
    warped_time_in_segment = wall_clock_time_sec * seg->final_rate;
    return accumulated_warped_time_sec + warped_time_in_segment;
}

/* ********************************************************************************** */

static sb_error_t sb_i_time_axis_grow_if_needed(sb_time_axis_t* axis)
{
    if (axis->end == axis->stor_end) {
        size_t capacity = sb_time_axis_capacity(axis);
        size_t length = sb_time_axis_num_segments(axis);
        size_t new_capacity = (capacity == 0) ? 4 : (capacity * 2);

        sb_time_segment_t* new_storage = sb_realloc(axis->stor_begin, sb_time_segment_t, new_capacity);
        if (new_storage == 0) {
            return SB_ENOMEM; /* LCOV_EXCL_LINE */
        }

        /* Compute offset before modifying stor_begin, then update pointers */
        axis->stor_begin = new_storage;
        axis->stor_end = new_storage + new_capacity;
        axis->end = new_storage + length;
    }

    return SB_SUCCESS;
}
