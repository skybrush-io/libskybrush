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

/**
 * @file yaw_control.c
 * @brief Handling of yaw control in Skybrush missions.
 */

#include <assert.h>
#include <float.h>
#include <math.h>
#include <string.h>

#include <skybrush/formats/binary.h>
#include <skybrush/memory.h>
#include <skybrush/utils.h>
#include <skybrush/yaw_control.h>

#include "../parsing.h"

#define SIZE_OF_SETPOINT (sizeof(uint16_t) + sizeof(int16_t))
#define OFFSET_OF_SETPOINT(index) (ctrl->header_length + (index)*SIZE_OF_SETPOINT)

sb_error_t sb_i_yaw_control_init_from_parser(sb_yaw_control_t* ctrl, sb_binary_file_parser_t* parser);

/**
 * Parses a yaw or yaw change value from the memory block that defines the yaw control object,
 * keeping its raw (ddeg) unit.
 *
 * The offset is automatically advanced after reading the value.
 */
static int16_t sb_i_yaw_control_parse_yaw(const sb_yaw_control_t* ctrl, size_t* offset);

/**
 * Parses a duration from the memory block that defines the yaw control curve,
 * keeping its raw (msec) unit.
 *
 * The offset is automatically advanced after reading the duration.
 */
static uint16_t sb_i_yaw_control_parse_duration(const sb_yaw_control_t* ctrl, size_t* offset);

/**
 * Parses the header of the memory block that defines the yaw control curve.
 */
static size_t sb_i_yaw_control_parse_header(sb_yaw_control_t* ctrl);

/**
 * Builds the current yaw setpoint from the wrapped buffer, starting from
 * the given offset, assuming that the start time and yaw of the current
 * setpoint has to be at the given parameters.
 */
static sb_error_t sb_i_yaw_player_build_current_setpoint(
    sb_yaw_player_t* player, size_t offset, uint32_t start_time_msec,
    int32_t start_yaw_ddeg);

/**
 * Resets the internal state of the yaw player and rewinds it to time zero.
 */
static sb_error_t sb_i_yaw_player_rewind(sb_yaw_player_t* player);

/**
 * Finds the setpoint in the yaw setpoint list that contains the given time.
 * Returns the relative time into the setpoint such that rel_t = 0 is the
 * start of the segment and rel_t = 1 is the end of the segment. It is
 * guaranteed that the returned relative time is between 0 and 1, inclusive.
 */
static sb_error_t sb_i_yaw_player_seek_to_time(sb_yaw_player_t* player, float t, float* rel_t);

/**
 * Instructs the yaw control object to take ownership of its inner memory buffer.
 */
static void sb_i_yaw_control_take_ownership(sb_yaw_control_t* ctrl);

/*****************************************************************************/

/**
 * Destroys a yaw control object and releases all memory that it owns.
 */
void sb_yaw_control_destroy(sb_yaw_control_t* ctrl)
{
    if (ctrl->owner) {
        sb_free(ctrl->buffer);
    }

    ctrl->buffer = 0;
    ctrl->buffer_length = 0;
    ctrl->owner = 0;

    ctrl->header_length = 0;
    ctrl->num_setpoints = 0;
    ctrl->auto_yaw = 0;
    ctrl->yaw_offset_ddeg = 0;
}

/**
 * Initializes a yaw control object from the contents of a Skybrush file in
 * binary format.
 *
 * \param ctrl  the yaw control object to initialize
 * \param fd  handle to the low-level file object to initialize the object from
 *
 * \return \c SB_SUCCESS if the object was initialized successfully,
 *         \c SB_ENOENT if the file did not contain a yaw control block,
 *         \c SB_EREAD for read errors
 */
sb_error_t sb_yaw_control_init_from_binary_file(sb_yaw_control_t* ctrl, int fd)
{
    sb_binary_file_parser_t parser;
    sb_error_t retval;

    SB_CHECK(sb_binary_file_parser_init_from_file(&parser, fd));
    retval = sb_i_yaw_control_init_from_parser(ctrl, &parser);
    sb_binary_file_parser_destroy(&parser);

    return retval;
}

/**
 * Initializes a yaw control object from the contents of a Skybrush file in
 * binary format, already loaded into memory.
 *
 * \param ctrl  the yaw control object to initialize
 * \param buf   the buffer holding the loaded Skybrush file in binary format
 * \param nbytes  the length of the buffer
 *
 * \return \c SB_SUCCESS if the object was initialized successfully,
 *         \c SB_ENOENT if the memory block did not contain a yaw control block
 */
sb_error_t sb_yaw_control_init_from_binary_file_in_memory(
    sb_yaw_control_t* ctrl, uint8_t* buf, size_t nbytes)
{
    sb_binary_file_parser_t parser;
    sb_error_t retval;

    SB_CHECK(sb_binary_file_parser_init_from_buffer(&parser, buf, nbytes));
    retval = sb_i_yaw_control_init_from_parser(ctrl, &parser);
    sb_binary_file_parser_destroy(&parser);

    return retval;
}

sb_error_t sb_i_yaw_control_init_from_parser(sb_yaw_control_t* ctrl, sb_binary_file_parser_t* parser)
{
    sb_error_t retval;
    sb_binary_block_t block;
    uint8_t* buf;

    SB_CHECK(sb_binary_file_find_first_block_by_type(parser, SB_BINARY_BLOCK_YAW_CONTROL));

    block = sb_binary_file_get_current_block(parser);

    buf = sb_calloc(uint8_t, block.length);
    if (buf == 0) {
        return SB_ENOMEM; /* LCOV_EXCL_LINE */
    }

    retval = sb_binary_file_read_current_block(parser, buf);
    if (retval != SB_SUCCESS) {
        sb_free(buf);
        return retval;
    }

    retval = sb_yaw_control_init_from_buffer(ctrl, buf, block.length);
    if (retval != SB_SUCCESS) {
        sb_free(buf);
        return retval;
    }

    sb_i_yaw_control_take_ownership(ctrl);

    return SB_SUCCESS;
}

/**
 * Initializes a yaw control object from the contents of a memory buffer.
 *
 * \param ctrl  the yaw control object to initialize
 * \param buf   the buffer holding the encoded yaw control object
 * \param nbytes  the length of the buffer
 *
 * \return \c SB_SUCCESS if the object was initialized successfully,
 *         \c SB_ENOENT if the memory buffer did not contain a yaw control object
 */
sb_error_t sb_yaw_control_init_from_buffer(sb_yaw_control_t* ctrl, uint8_t* buf, size_t nbytes)
{
    ctrl->buffer = buf;
    ctrl->buffer_length = nbytes;
    ctrl->owner = 0;

    ctrl->header_length = sb_i_yaw_control_parse_header(ctrl);

    return SB_SUCCESS;
}

/**
 * Initializes an empty yaw control object.
 *
 * \param ctrl  the yaw control object to initialize
 */
sb_error_t sb_yaw_control_init_empty(sb_yaw_control_t* ctrl)
{
    ctrl->buffer = 0;
    ctrl->buffer_length = 0;
    ctrl->owner = 0;

    ctrl->header_length = 0;
    ctrl->num_setpoints = 0;
    ctrl->auto_yaw = 0;
    ctrl->yaw_offset_ddeg = 0;

    return SB_SUCCESS;
}

/**
 * @brief Returns whether the yaw control object is empty (i.e. has no setpoints).
 *
 * @param ctrl   the yaw control object
 * @return true if the yaw control object has no entries, false otherwise
 */
sb_bool_t sb_yaw_control_is_empty(const sb_yaw_control_t* ctrl)
{
    return ctrl->num_setpoints == 0;
}

/* ************************************************************************** */

static size_t sb_i_yaw_control_parse_header(sb_yaw_control_t* ctrl)
{
    uint8_t* buf = ctrl->buffer;
    size_t offset;

    assert(buf != 0);

    ctrl->auto_yaw = (sb_bool_t)((buf[0] & 0x01));

    offset = 1;
    ctrl->yaw_offset_ddeg = sb_i_yaw_control_parse_yaw(ctrl, &offset);

    ctrl->num_setpoints = (size_t)((ctrl->buffer_length - offset) / SIZE_OF_SETPOINT);

    return offset; /* size of the header */
}

static void sb_i_yaw_control_take_ownership(sb_yaw_control_t* ctrl)
{
    ctrl->owner = 1;
}

static int16_t sb_i_yaw_control_parse_yaw(const sb_yaw_control_t* ctrl, size_t* offset)
{
    return sb_parse_int16(ctrl->buffer, offset);
}

static uint16_t sb_i_yaw_control_parse_duration(const sb_yaw_control_t* ctrl, size_t* offset)
{
    return sb_parse_uint16(ctrl->buffer, offset);
}

/* ************************************************************************** */

/**
 * Initializes a yaw player that plays the given yaw control object.
 */
sb_error_t sb_yaw_player_init(sb_yaw_player_t* player, const sb_yaw_control_t* ctrl)
{
    if (ctrl == 0) {
        return SB_EINVAL;
    }

    memset(player, 0, sizeof(sb_yaw_player_t));

    player->ctrl = ctrl;

    sb_i_yaw_player_rewind(player);

    return SB_SUCCESS;
}

/**
 * Destroys a yaw player object and releases all memory that it owns.
 */
void sb_yaw_player_destroy(sb_yaw_player_t* player)
{
    memset(player, 0, sizeof(sb_yaw_player_t));
}

/**
 * Builds the next setpoint in the yaw player. Used to move on to the next
 * setpoint during an iteration over the setpoints of the yaw control object.
 */
sb_error_t sb_yaw_player_build_next_setpoint(sb_yaw_player_t* player)
{
    return sb_i_yaw_player_build_current_setpoint(
        player,
        player->current_setpoint.start + player->current_setpoint.length,
        player->current_setpoint.data.end_time_msec,
        player->current_setpoint.data.end_yaw_ddeg);
}

/* LCOV_EXCL_START */

/**
 * Dumps the details of the current yaw setpoint for debugging purposes.
 */
void sb_yaw_player_dump_current_setpoint(const sb_yaw_player_t* player)
{
#ifdef LIBSKYBRUSH_DEBUG
    sb_vector3_with_yaw_t pos, vel, acc;
    const sb_yaw_setpoint_t* current = sb_yaw_player_get_current_setpoint(player);

    printf("Start offset = %ld bytes\n", (long int)player->current_setpoint.start);
    printf("Length = %ld bytes\n", (long int)player->current_setpoint.length);
    printf("Start time = %.3fs\n", current->start_time_sec);
    printf("Duration = %.3fs\n", current->duration_sec);
    printf("Start yaw = %.3fdeg\n", current->start_yaw_deg);
    printf("Yaw change = %.3fdeg\n", current->yaw_change_deg);
#endif
}

/* LCOV_EXCL_STOP */

/**
 * Returns a pointer to the current yaw setpoint of the yaw player.
 */
const sb_yaw_setpoint_t* sb_yaw_player_get_current_setpoint(
    const sb_yaw_player_t* player)
{
    return &player->current_setpoint.data;
}

/**
 * Returns the yaw value associated to the player at the given time instant, in degrees.
 */
sb_error_t sb_yaw_player_get_yaw_at(sb_yaw_player_t* player, float t, float* result)
{
    float rel_t;

    SB_CHECK(sb_i_yaw_player_seek_to_time(player, t, &rel_t));

    if (result) {
        sb_yaw_setpoint_t* setpoint = &player->current_setpoint.data;
        *result = setpoint->start_yaw_deg + setpoint->yaw_change_deg * rel_t;
    }

    return SB_SUCCESS;
}

/**
 * Returns the yaw rate associated to the player at the given time instant.
 */
sb_error_t sb_yaw_player_get_yaw_rate_at(sb_yaw_player_t* player, float t, float* result)
{
    float rel_t;

    SB_CHECK(sb_i_yaw_player_seek_to_time(player, t, &rel_t));

    if (result) {
        sb_yaw_setpoint_t* setpoint = &player->current_setpoint.data;
        if (setpoint->duration_sec) {
            *result = setpoint->yaw_change_deg / setpoint->duration_sec;
        } else {
            // TODO(vasarhelyi): we should return largest possible yaw rate here
            *result = INFINITY;
        }
    }

    return SB_SUCCESS;
}

/**
 * Returns the total duration of the yaw control curve associated to the player, in seconds.
 */
sb_error_t sb_yaw_player_get_total_duration_msec(sb_yaw_player_t* player, uint32_t* duration)
{
    uint32_t result = 0;

    SB_CHECK(sb_i_yaw_player_rewind(player));

    while (sb_yaw_player_has_more_setpoints(player)) {
        result += player->current_setpoint.data.duration_msec;
        SB_CHECK(sb_yaw_player_build_next_setpoint(player));
    }

    if (duration) {
        *duration = result;
    }

    return SB_SUCCESS;
}

/**
 * Returns whether the yaw player has more setpoints to play. Used to detect
 * the end of iteration when iterating over the setpoints of the yaw control object.
 */
sb_bool_t sb_yaw_player_has_more_setpoints(const sb_yaw_player_t* player)
{
    return player->current_setpoint.length > 0;
}

/* ************************************************************************** */

static sb_error_t sb_i_yaw_player_seek_to_time(sb_yaw_player_t* player, float t, float* rel_t)
{
    size_t offset;

    if (t <= 0) {
        t = 0;
    }

    while (1) {
        sb_yaw_setpoint_t* setpoint = &player->current_setpoint.data;

        if (setpoint->start_time_sec > t) {
            /* time that the user asked for is before the current setpoint. We simply
             * rewind and start from scratch */
            SB_CHECK(sb_i_yaw_player_rewind(player));
            assert(player->current_setpoint.data.start_time_msec == 0);
        } else if (setpoint->end_time_sec < t) {
            offset = player->current_setpoint.start;
            SB_CHECK(sb_yaw_player_build_next_setpoint(player));
            if (!sb_yaw_player_has_more_setpoints(player)) {
                /* reached end of yaw control */
            } else {
                /* assert that we really moved forward in the buffer */
                assert(player->current_setpoint.start > offset);
                /* make production builds happy by referencing offset even if
                 * asserts are disabled */
                ((void)offset);
            }
        } else {
            if (rel_t) {
                if (!isfinite(t)) {
                    *rel_t = 1;
                } else if (fabsf(setpoint->duration_sec) > 1.0e-6f) {
                    *rel_t = (t - setpoint->start_time_sec) / setpoint->duration_sec;
                } else {
                    *rel_t = 0.5;
                }
            }
            return SB_SUCCESS;
        }
    }
}

static sb_error_t sb_i_yaw_player_build_current_setpoint(
    sb_yaw_player_t* player, size_t offset, uint32_t start_time_msec,
    int32_t start_yaw)
{
    const sb_yaw_control_t* ctrl = player->ctrl;
    size_t buffer_length = ctrl->buffer_length;
    sb_yaw_setpoint_t* data = &player->current_setpoint.data;

    /* Initialize the current setpoint */
    memset(&player->current_setpoint, 0, sizeof(player->current_setpoint));
    player->current_setpoint.start = offset;

    /* Store the start time as instructed */
    data->start_time_msec = start_time_msec;
    data->start_time_sec = start_time_msec / 1000.0f;

    /* Store the start yaw as instructed */
    data->start_yaw_ddeg = start_yaw;
    data->start_yaw_deg = data->start_yaw_ddeg / 10.0f;

    if (offset >= buffer_length) {
        /* We are beyond the end of the buffer, indicating that there are
         * no more setpoints in the buffer; we keep last yaw forever */
        data->duration_msec = UINT32_MAX - data->start_time_msec;
        data->duration_sec = INFINITY;
        data->end_time_msec = UINT32_MAX;
        data->end_time_sec = INFINITY;
        data->end_yaw_deg = data->start_yaw_deg;
        data->end_yaw_ddeg = data->start_yaw_ddeg;

        return SB_SUCCESS;
    }

    /* Parse duration and calculate end time */
    data->duration_msec = sb_i_yaw_control_parse_duration(ctrl, &offset);
    data->duration_sec = data->duration_msec / 1000.0f;
    data->end_time_msec = data->start_time_msec + data->duration_msec;
    data->end_time_sec = data->end_time_msec / 1000.0f;

    /* Parse yaw change and calculate end yaw */
    data->yaw_change_ddeg = sb_i_yaw_control_parse_yaw(ctrl, &offset);
    data->yaw_change_deg = data->yaw_change_ddeg / 10.0f;
    data->end_yaw_ddeg = data->start_yaw_ddeg + data->yaw_change_ddeg;
    data->end_yaw_deg = data->end_yaw_ddeg / 10.0f;

    player->current_setpoint.length = offset - player->current_setpoint.start;

    return SB_SUCCESS;
}

static sb_error_t sb_i_yaw_player_rewind(sb_yaw_player_t* player)
{
    return sb_i_yaw_player_build_current_setpoint(
        player, player->ctrl->header_length, 0, player->ctrl->yaw_offset_ddeg);
}
