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

#include <math.h>
#include <string.h>

#include <skybrush/trajectory.h>

#include "../parsing.h"

#define HEADER_LENGTH 9
#define MAX_DURATION_MSEC 60000

static sb_error_t sb_i_trajectory_builder_scale_coordinate(
    sb_trajectory_builder_t* builder, float coordinate, int16_t* scaled_coordinate);
static sb_error_t sb_i_trajectory_builder_write_angle(
    sb_trajectory_builder_t* builder, size_t* offset, float angle);
static sb_error_t sb_i_trajectory_builder_write_coordinate(
    sb_trajectory_builder_t* builder, size_t* offset, float coordinate);

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
 * @brief Destroys a trajectory builder.
 */
void sb_trajectory_builder_destroy(sb_trajectory_builder_t* builder)
{
    sb_buffer_destroy(&builder->buffer);
    memset(&builder->last_position, 0, sizeof(builder->last_position));
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
 * @brief Appends a new straight-line segment to the trajectory being built.
 *
 * @param builder the trajectory builder
 * @param target the target to move to
 * @param duration_msec the duration of the segment, in milliseconds
 */
sb_error_t sb_trajectory_builder_append_line(
    sb_trajectory_builder_t* builder, const sb_vector3_with_yaw_t target,
    uint32_t duration_msec)
{
    if (duration_msec > MAX_DURATION_MSEC) {
        /* If duration_msec > 60000, split the segment into multiple sub-segments */
        sb_vector3_with_yaw_t midpoint;
        uint32_t half_duration_msec = duration_msec >> 1;

        midpoint.x = (builder->last_position.x + target.x) / 2;
        midpoint.y = (builder->last_position.y + target.y) / 2;
        midpoint.z = (builder->last_position.z + target.z) / 2;
        midpoint.yaw = (builder->last_position.yaw + target.yaw) / 2;

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
        *flags_ptr |= 1;
        SB_CHECK(sb_i_trajectory_builder_write_coordinate(builder, &offset, target.x));
    }
    if (builder->last_position.y != target.y) {
        *flags_ptr |= (1 << 2);
        SB_CHECK(sb_i_trajectory_builder_write_coordinate(builder, &offset, target.y));
    }
    if (builder->last_position.z != target.z) {
        *flags_ptr |= (1 << 4);
        SB_CHECK(sb_i_trajectory_builder_write_coordinate(builder, &offset, target.z));
    }
    if (builder->last_position.yaw != target.yaw) {
        *flags_ptr |= (1 << 6);
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

        SB_CHECK(sb_trajectory_builder_append_line(builder, builder->last_position, current_duration_msec));
    }

    return SB_SUCCESS;
}
/**
 * @brief Finalizes the trajectory being built and converts it into a trajectory
 * object.
 *
 * @param trajectory the trajectory to initialize
 * @param builder the trajectory builder to initialize the trajectory from. The
 *     builder will be reset to an uninitialized state.
 * @return error code
 */
sb_error_t sb_trajectory_init_from_builder(
    sb_trajectory_t* trajectory, sb_trajectory_builder_t* builder)
{
    uint8_t* buf = SB_BUFFER(builder->buffer);
    uint8_t header = buf[0];

    SB_CHECK(sb_trajectory_init_from_bytes(trajectory, buf, sb_buffer_size(&builder->buffer)));

    /* ownership of the memory buffer now belongs to the trajectory so we can
     * re-initialize the builder */
    SB_CHECK(sb_buffer_init(&builder->buffer, HEADER_LENGTH));
    buf = SB_BUFFER(builder->buffer);
    buf[0] = header;

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

static sb_error_t sb_i_trajectory_builder_write_angle(
    sb_trajectory_builder_t* builder, size_t* offset, float angle)
{
    int16_t scaled = fmodf(angle, 360) * 10.0f;
    if (scaled < 0) {
        scaled += 3600;
    }

    sb_write_int16(SB_BUFFER(builder->buffer), offset, scaled);
    return SB_SUCCESS;
}

static sb_error_t sb_i_trajectory_builder_write_coordinate(
    sb_trajectory_builder_t* builder, size_t* offset, float coordinate)
{
    int16_t scaled;

    SB_CHECK(sb_i_trajectory_builder_scale_coordinate(builder, coordinate, &scaled));
    sb_write_int16(SB_BUFFER(builder->buffer), offset, scaled);

    return SB_SUCCESS;
}
