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
#include <string.h>

#include <skybrush/buffer.h>
#include <skybrush/motion.h>
#include <skybrush/trajectory.h>
#include <skybrush/utils.h>

#include "../parsing.h"
#include "./builder.h"

#define HEADER_LENGTH 9
#define MAX_DURATION_MSEC 60000

static sb_error_t sb_i_trajectory_builder_scale_coordinate(
    sb_trajectory_builder_t* builder, float coordinate, int16_t* scaled_coordinate);

/**
 * @brief Creates a new trajectory builder.
 *
 * @param builder The builder to initialize
 * @param scale The scale of the trajectory
 * @param flags Additional flags to be written in the trajectory header; see
 *     \ref sb_trajectory_flags_t for more details
 * @return \c SB_EINVAL if the scale is too large or zero, \c SB_SUCCESS otherwise
 */
sb_error_t sb_trajectory_builder_init(sb_trajectory_builder_t* builder, uint8_t scale, uint8_t flags)
{
    uint8_t* header;

    if (scale == 0 || scale > 127) {
        return SB_EINVAL;
    }

    SB_CHECK(sb_buffer_init(&builder->buffer, HEADER_LENGTH));
    memset(&builder->last_position, 0, sizeof(builder->last_position));
    builder->scale = scale;

    header = SB_BUFFER(builder->buffer);
    header[0] = scale;
    if (flags & SB_TRAJECTORY_USE_YAW) {
        header[0] |= 128;
    }

    return SB_SUCCESS;
}

/**
 * @brief Creates a new trajectory builder from an existing trajectory.
 *
 * The trajectory builder and the trajectory will share the same underlying
 * memory buffer. It is the responsibility of the caller to ensure that there
 * is enough space in the memory buffer to contain the entire trajectory even
 * when new segments are appended.
 *
 * @param builder The builder to initialize
 * @param trajectory The existing trajectory to initialize the builder from
 * @param last_position The last position of the trajectory, if known, \c NULL otherwise
 */
sb_error_t sb_trajectory_builder_init_from_trajectory(sb_trajectory_builder_t* builder,
    sb_trajectory_t* trajectory, const sb_vector3_with_yaw_t* last_position)
{
    /* Construct a view into the entire buffer of the trajectory, _including_
     * the allocated-but-not-used space at the end */
    sb_buffer_init_view(&builder->buffer, SB_BUFFER(trajectory->buffer),
        sb_buffer_capacity(&trajectory->buffer));

    /* If the original buffer had extra space at the end, shrink the buffer of
     * the builder in a similar manner */
    SB_CHECK(sb_buffer_resize(&builder->buffer, sb_buffer_size(&trajectory->buffer)));

    if (last_position) {
        builder->last_position = *last_position;
    } else {
        SB_CHECK(sb_trajectory_get_end_position(trajectory, &builder->last_position));
    }
    builder->scale = trajectory->scale;

    return SB_SUCCESS;
}

/**
 * @brief Destroys a trajectory builder.
 */
void sb_trajectory_builder_destroy(sb_trajectory_builder_t* builder)
{
    sb_buffer_destroy(&builder->buffer);
    memset(&builder->last_position, 0, sizeof(builder->last_position));
}

/**
 * @brief Gets the last position of the trajectory being built.
 *
 * @param builder the trajectory builder
 * @return the last position of the trajectory
 */
sb_vector3_with_yaw_t sb_trajectory_builder_get_last_position(
    const sb_trajectory_builder_t* builder)
{
    return builder->last_position;
}

/**
 * @brief Sets the start point of the trajectory being built.
 *
 * @param builder the trajectory builder
 * @param start the start point of the trajectory
 */
sb_error_t sb_trajectory_builder_set_start_position(
    sb_trajectory_builder_t* builder, sb_vector3_with_yaw_t start)
{
    /* Don't allow the user to change the start position when at least one
     * segment was already written */
    if (sb_buffer_size(&builder->buffer) != HEADER_LENGTH) {
        return SB_FAILURE;
    }

    size_t offset = 1;

    SB_CHECK(sb_i_trajectory_builder_write_coordinate(builder, &offset, start.x));
    SB_CHECK(sb_i_trajectory_builder_write_coordinate(builder, &offset, start.y));
    SB_CHECK(sb_i_trajectory_builder_write_coordinate(builder, &offset, start.z));
    SB_CHECK(sb_i_trajectory_builder_write_angle(builder, &offset, start.yaw));

    builder->last_position = start;

    return SB_SUCCESS;
}

/**
 * @brief Appends a new cubic Bézier segment to the trajectory being built.
 *
 * @param builder the trajectory builder
 * @param control1 the first control point of the cubic Bézier segment to move along
 * @param control2 the second control point of the cubic Bézier segment to move along
 * @param target the target to move to
 * @param duration_msec the duration of the segment, in milliseconds
 * @return \c SB_SUCCESS on success, error code otherwise
 */
sb_error_t sb_trajectory_builder_append_cubic_bezier(
    sb_trajectory_builder_t* builder, const sb_vector3_with_yaw_t control1,
    const sb_vector3_with_yaw_t control2, const sb_vector3_with_yaw_t target,
    uint32_t duration_msec)
{
    if (duration_msec == 0) {
        return SB_EINVAL;
    }

    if (duration_msec > MAX_DURATION_MSEC) {
        // TODO: split into smaller parts instead of throwing error if ever needed
        return SB_EINVAL;
    }

    size_t offset = sb_buffer_size(&builder->buffer);
    uint8_t* flags_ptr;

    SB_CHECK(sb_buffer_extend_with_zeros(&builder->buffer, 27));

    /* We will always need 1 byte for the header */
    flags_ptr = SB_BUFFER(builder->buffer) + offset;
    offset++;
    *flags_ptr = 0;

    sb_write_uint16(SB_BUFFER(builder->buffer), &offset, duration_msec);
    if (builder->last_position.x != control1.x || control1.x != control2.x || control2.x != target.x) {
        *flags_ptr |= SB_X_BEZIER;
        SB_CHECK(sb_i_trajectory_builder_write_coordinate(builder, &offset, control1.x));
        SB_CHECK(sb_i_trajectory_builder_write_coordinate(builder, &offset, control2.x));
        SB_CHECK(sb_i_trajectory_builder_write_coordinate(builder, &offset, target.x));
    }
    if (builder->last_position.y != control1.y || control1.y != control2.y || control2.y != target.y) {
        *flags_ptr |= SB_Y_BEZIER;
        SB_CHECK(sb_i_trajectory_builder_write_coordinate(builder, &offset, control1.y));
        SB_CHECK(sb_i_trajectory_builder_write_coordinate(builder, &offset, control2.y));
        SB_CHECK(sb_i_trajectory_builder_write_coordinate(builder, &offset, target.y));
    }
    if (builder->last_position.z != control1.z || control1.z != control2.z || control2.z != target.z) {
        *flags_ptr |= SB_Z_BEZIER;
        SB_CHECK(sb_i_trajectory_builder_write_coordinate(builder, &offset, control1.z));
        SB_CHECK(sb_i_trajectory_builder_write_coordinate(builder, &offset, control2.z));
        SB_CHECK(sb_i_trajectory_builder_write_coordinate(builder, &offset, target.z));
    }
    if (builder->last_position.yaw != control1.yaw || control1.yaw != control2.yaw || control2.yaw != target.yaw) {
        *flags_ptr |= SB_YAW_BEZIER;
        SB_CHECK(sb_i_trajectory_builder_write_angle(builder, &offset, control1.yaw));
        SB_CHECK(sb_i_trajectory_builder_write_angle(builder, &offset, control2.yaw));
        SB_CHECK(sb_i_trajectory_builder_write_angle(builder, &offset, target.yaw));
    }

    /* Trim the unneeded bytes from the end */
    SB_CHECK(sb_buffer_resize(&builder->buffer, offset));

    builder->last_position = target;

    return SB_SUCCESS;
}

/**
 * @brief Appends a new straight-line segment to the trajectory being built.
 *
 * @param builder the trajectory builder
 * @param target the target to move to
 * @param duration_msec the duration of the segment, in milliseconds
 * @return \c SB_SUCCESS on success, error code otherwise
 */
sb_error_t sb_trajectory_builder_append_line(
    sb_trajectory_builder_t* builder, const sb_vector3_with_yaw_t target,
    uint32_t duration_msec)
{
    if (duration_msec == 0) {
        return SB_EINVAL;
    }

    if (duration_msec > MAX_DURATION_MSEC) {
        /* If duration_msec > 60000, split the segment into multiple sub-segments */
        sb_vector3_with_yaw_t midpoint;
        uint32_t half_duration_msec = duration_msec >> 1;

        midpoint.x = (builder->last_position.x + target.x) / 2;
        midpoint.y = (builder->last_position.y + target.y) / 2;
        midpoint.z = (builder->last_position.z + target.z) / 2;
        midpoint.yaw = (builder->last_position.yaw + target.yaw) / 2;

        /* Here we know that half_duration_msec and duration_msec - half_duration_msec
         * are both positive because duration_msec was checked to be greater than
         * \c MAX_DURATION_MSEC
         */
        assert(half_duration_msec > 0);
        assert(duration_msec > half_duration_msec);
        SB_CHECK(sb_trajectory_builder_append_line(builder, midpoint, half_duration_msec));
        SB_CHECK(sb_trajectory_builder_append_line(builder, target, duration_msec - half_duration_msec));

        return SB_SUCCESS;
    }

    size_t offset = sb_buffer_size(&builder->buffer);
    uint8_t* flags_ptr;

    SB_CHECK(sb_buffer_extend_with_zeros(&builder->buffer, 11));

    /* We will always need 1 byte for the header */
    flags_ptr = SB_BUFFER(builder->buffer) + offset;
    offset++;
    *flags_ptr = 0;

    sb_write_uint16(SB_BUFFER(builder->buffer), &offset, duration_msec);
    if (builder->last_position.x != target.x) {
        *flags_ptr |= SB_X_LINEAR;
        SB_CHECK(sb_i_trajectory_builder_write_coordinate(builder, &offset, target.x));
    }
    if (builder->last_position.y != target.y) {
        *flags_ptr |= SB_Y_LINEAR;
        SB_CHECK(sb_i_trajectory_builder_write_coordinate(builder, &offset, target.y));
    }
    if (builder->last_position.z != target.z) {
        *flags_ptr |= SB_Z_LINEAR;
        SB_CHECK(sb_i_trajectory_builder_write_coordinate(builder, &offset, target.z));
    }
    if (builder->last_position.yaw != target.yaw) {
        *flags_ptr |= SB_YAW_LINEAR;
        SB_CHECK(sb_i_trajectory_builder_write_angle(builder, &offset, target.yaw));
    }

    /* Trim the unneeded bytes from the end */
    SB_CHECK(sb_buffer_resize(&builder->buffer, offset));

    builder->last_position = target;

    return SB_SUCCESS;
}

/**
 * @brief Appends a new constant segment to the trajectory being built.
 *
 * When the duration is larger than the maximum allowed length of a single
 * segment, the new segment will be split appropriately.
 *
 * @param builder the trajectory builder
 * @param duration_msec the duration of the segment, in milliseconds
 */
sb_error_t sb_trajectory_builder_hold_position_for(
    sb_trajectory_builder_t* builder, uint32_t duration_msec)
{
    uint16_t current_duration_msec;

    while (duration_msec > 0) {
        current_duration_msec = duration_msec > MAX_DURATION_MSEC ? MAX_DURATION_MSEC : duration_msec;
        duration_msec -= current_duration_msec;

        assert(current_duration_msec > 0);

        SB_CHECK(sb_trajectory_builder_append_line(builder, builder->last_position, current_duration_msec));
    }

    return SB_SUCCESS;
}

/**
 * Appends multiple segments to the trajectory being built in a way that they
 * take the trajectory from its current position to the given target position in a given
 * fixed time while trying to respnect the given maximum acceleration constraint.
 *
 * The trajectory builder will append multiple segments: one that speeds up from
 * zero velocity to a required travel velocity, one or more segments that moves the
 * drone at the required travel velocity and one that slows down from the travel
 * velocity to zero velocity.
 *
 * the given initial velocity vector to the travel velocity
 * @param builder the trajectory builder
 * @param target  the target to move to
 * @param duration_msec the total duration of the transition, in milliseconds. Must be
 *        positive.
 * @param max_acceleration  maximum acceleration, in units per second per second.
 *        May not be respected if the duration is too short to reach the required
 *        travel velocity. Must be positive and finite; if it is not, it will be treated
 *        as infinite.
 */
sb_error_t sb_trajectory_builder_move_to_in_time(
    sb_trajectory_builder_t* builder, sb_vector3_with_yaw_t target,
    uint32_t duration_msec, float max_acceleration)
{
    sb_vector3_with_yaw_t p, q, initial_vel, travel_vel, c1, c2;
    float duration_sec;
    float distance;
    float v2, t2, t1;
    uint32_t t1_msec;

    if (duration_msec == 0) {
        return SB_EINVAL;
    }

    if (!isfinite(max_acceleration) || max_acceleration <= 0) {
        /* max acceleration is infinite so we can just use straight-line segments */
        return sb_trajectory_builder_append_line(builder, target, duration_msec);
    }

    if (duration_msec < 3) {
        /* duration is too short so we can use a single straight-line segment */
        return sb_trajectory_builder_append_line(builder, target, duration_msec);
    }

    /* calculate required travel velocity */
    p = builder->last_position;
    distance = hypotf(hypotf(target.x - p.x, target.y - p.y), target.z - p.z);
    duration_sec = duration_msec / 1000.0f;
    v2 = sb_get_travel_velocity_for_distance(distance, duration_sec, max_acceleration);

    /* pre-fill initial and final velocity constant */
    initial_vel.x = initial_vel.y = initial_vel.z = 0;
    initial_vel.yaw = (target.yaw - p.yaw) / duration_sec;

    /* calculate duration of constant-velocity segment */
    t2 = duration_sec - 2 * v2 / max_acceleration;
    if (t2 < 0) {
        /* max acceleration is not enough. We will have a zero-duration segment and
         * uniform acceleration from zero to the required travel velocity and then
         * back to zero */
        t2 = 0;
        v2 = 2 * distance / duration_sec;
    }
    t1 = (duration_sec - t2) / 2.0f;
    t1_msec = (uint32_t)(t1 * 1000);
    if (t1_msec < 1) {
        t1_msec = 1;
        t1 = t1_msec * 1000.0f;
    }

    /* add segment to accelerate smoothly from zero to travel velocity */
    p = builder->last_position;
    travel_vel.x = v2 * (target.x - p.x) / distance;
    travel_vel.y = v2 * (target.y - p.y) / distance;
    travel_vel.z = v2 * (target.z - p.z) / distance;
    travel_vel.yaw = initial_vel.yaw;

    q = p;
    q.x += travel_vel.x * t1 / 2.0f;
    q.y += travel_vel.y * t1 / 2.0f;
    q.z += travel_vel.z * t1 / 2.0f;
    q.yaw += travel_vel.yaw * t1;

    SB_CHECK(
        sb_get_cubic_bezier_from_velocity_constraints(
            /* start = */ p, /* start_vel = */ initial_vel,
            /* end = */ q, /* end_vel = */ travel_vel,
            /* duration_sec = */ t1,
            &c1, &c2));
    SB_CHECK(sb_trajectory_builder_append_cubic_bezier(builder, c1, c2, q, t1_msec));

    /* add constant-velocity segment */
    p = target;
    p.x -= travel_vel.x * t1 / 2.0f;
    p.y -= travel_vel.y * t1 / 2.0f;
    p.z -= travel_vel.z * t1 / 2.0f;
    p.yaw -= travel_vel.yaw * t1;
    if (duration_msec > 2 * t1_msec) {
        /* Only add the constant-velocity segment if it has a positive duration */
        SB_CHECK(sb_trajectory_builder_append_line(builder, p, duration_msec - 2 * t1_msec));
    }

    /* add segment to decelerate smoothly from travel velocity to zero */
    q = target;
    SB_CHECK(
        sb_get_cubic_bezier_from_velocity_constraints(
            /* start = */ p, /* start_vel = */ travel_vel,
            /* end = */ q, /* end_vel = */ initial_vel,
            /* duration_sec = */ t1,
            &c1, &c2));
    SB_CHECK(sb_trajectory_builder_append_cubic_bezier(builder, c1, c2, q, t1_msec));

    return SB_SUCCESS;
}

/* ************************************************************************** */

/**
 * @brief Finalizes the trajectory being built and converts it into a trajectory
 * object.
 *
 * @param trajectory the trajectory to update
 * @param builder the trajectory builder to update the trajectory from. The
 *     builder will be reset to an uninitialized state.
 * @return error code
 */
sb_error_t sb_trajectory_update_from_builder(
    sb_trajectory_t* trajectory, sb_trajectory_builder_t* builder)
{
    uint8_t* data;
    uint8_t header;

    assert(!sb_buffer_is_view(&builder->buffer));

    /* ask the buffer to release the ownership of its underlying allocated memory block
     * and convert itself into a view */
    data = sb_buffer_ensure_view(&builder->buffer);
    header = data[0];

    /* pass on the ownership of 'data' to the trajectory */
    SB_CHECK(sb_trajectory_update_from_bytes(trajectory, data, sb_buffer_size(&builder->buffer)));

    /* ownership of 'data' now belongs to the trajectory so we can
     * re-initialize the buffer of the builder */
    sb_buffer_destroy(&builder->buffer);
    SB_CHECK(sb_buffer_init(&builder->buffer, HEADER_LENGTH));

    /* preserve the header byte of the builder buffer */
    data = SB_BUFFER(builder->buffer);
    data[0] = header;

    return SB_SUCCESS;
}

/* ************************************************************************** */

static sb_error_t sb_i_trajectory_builder_scale_coordinate(
    sb_trajectory_builder_t* builder, float coordinate, int16_t* scaled_coordinate)
{
    float scaled = floorf(coordinate / builder->scale);
    if (scaled < INT16_MIN || scaled > INT16_MAX) {
        return SB_EINVAL;
    } else {
        *scaled_coordinate = scaled;
        return SB_SUCCESS;
    }
}

sb_error_t sb_i_trajectory_builder_write_angle(
    sb_trajectory_builder_t* builder, size_t* offset, float angle)
{
    int16_t scaled = fmodf(angle, 360) * 10.0f;
    if (scaled < 0) {
        scaled += 3600;
    }

    sb_write_int16(SB_BUFFER(builder->buffer), offset, scaled);
    return SB_SUCCESS;
}

sb_error_t sb_i_trajectory_builder_write_coordinate(
    sb_trajectory_builder_t* builder, size_t* offset, float coordinate)
{
    int16_t scaled;

    SB_CHECK(sb_i_trajectory_builder_scale_coordinate(builder, coordinate, &scaled));
    sb_write_int16(SB_BUFFER(builder->buffer), offset, scaled);

    return SB_SUCCESS;
}
