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
 * @file rth_plan.c
 * @brief Handling of collective return-to-home plans in Skybrush missions.
 */

#include <assert.h>
#include <float.h>
#include <math.h>
#include <string.h>

#include <skybrush/formats/binary.h>
#include <skybrush/memory.h>
#include <skybrush/rth_plan.h>

#include "../parsing.h"

/**
 * @def MAX_DURATION
 * @brief Maximum duration allowed for RTH actions
 *
 * This is the largest integer that can be fitted into a single-precision float
 * without loss of precision.
 */
#define MAX_DURATION 16777216

#define OFFSET_OF_POINT(index) (plan->header_length + (index)*2 * sizeof(int16_t))
#define OFFSET_OF_ENTRY_TABLE (OFFSET_OF_POINT(plan->num_points))
#define OFFSET_OF_FIRST_ENTRY (OFFSET_OF_ENTRY_TABLE + sizeof(uint16_t))

/**
 * @brief Returns whether the given RTH action has an associated target coordinate.
 */
static sb_bool_t sb_i_rth_action_has_target(sb_rth_action_t action);

sb_error_t sb_i_rth_plan_init_from_parser(sb_rth_plan_t* plan, sb_binary_file_parser_t* parser);

/**
 * Parses a coordinate from the memory block that defines the RTH plan,
 * scaling it up with the appropriate scaling factor as needed.
 *
 * The offset is automatically advanced after reading the coordinate.
 */
static float sb_i_rth_plan_parse_coordinate(const sb_rth_plan_t* plan, size_t* offset);

/**
 * Parses the header of the memory block that defines the RTH plan.
 */
static size_t sb_i_rth_plan_parse_header(sb_rth_plan_t* plan);

/**
 * Instructs the RTH plan object to take ownership of its inner memory buffer.
 */
static void sb_i_rth_plan_take_ownership(sb_rth_plan_t* plan);

/**
 * Destroys an RTH plan object and releases all memory that it owns.
 */
void sb_rth_plan_destroy(sb_rth_plan_t* plan)
{
    if (plan->owner) {
        sb_free(plan->buffer);
    }

    plan->buffer = 0;
    plan->buffer_length = 0;
    plan->owner = 0;

    plan->header_length = 0;
    plan->num_points = 0;
    plan->scale = 1;
}

/**
 * Initializes an RTH plan object from the contents of a Skybrush file in
 * binary format.
 *
 * \param plan  the RTH plan to initialize
 * \param fd  handle to the low-level file object to initialize the plan from
 *
 * \return \c SB_SUCCESS if the object was initialized successfully,
 *         \c SB_ENOENT if the file did not contain a trajectory block,
 *         \c SB_EREAD for read errors
 */
sb_error_t sb_rth_plan_init_from_binary_file(sb_rth_plan_t* plan, int fd)
{
    sb_binary_file_parser_t parser;
    sb_error_t retval;

    SB_CHECK(sb_binary_file_parser_init_from_file(&parser, fd));
    retval = sb_i_rth_plan_init_from_parser(plan, &parser);
    sb_binary_file_parser_destroy(&parser);

    return retval;
}

/**
 * Initializes an RTH plan object from the contents of a Skybrush file in
 * binary format, already loaded into memory.
 *
 * \param plan  the RTH plan to initialize
 * \param buf   the buffer holding the loaded Skybrush file in binary format
 * \param nbytes  the length of the buffer
 *
 * \return \c SB_SUCCESS if the object was initialized successfully,
 *         \c SB_ENOENT if the memory block did not contain a trajectory
 */
sb_error_t sb_rth_plan_init_from_binary_file_in_memory(
    sb_rth_plan_t* plan, uint8_t* buf, size_t nbytes)
{
    sb_binary_file_parser_t parser;
    sb_error_t retval;

    SB_CHECK(sb_binary_file_parser_init_from_buffer(&parser, buf, nbytes));
    retval = sb_i_rth_plan_init_from_parser(plan, &parser);
    sb_binary_file_parser_destroy(&parser);

    return retval;
}

sb_error_t sb_i_rth_plan_init_from_parser(sb_rth_plan_t* plan, sb_binary_file_parser_t* parser)
{
    sb_error_t retval;
    sb_binary_block_t block;
    uint8_t* buf;

    SB_CHECK(sb_binary_file_find_first_block_by_type(parser, SB_BINARY_BLOCK_RTH_PLAN));

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

    retval = sb_rth_plan_init_from_buffer(plan, buf, block.length);
    if (retval != SB_SUCCESS) {
        sb_free(buf);
        return retval;
    }

    sb_i_rth_plan_take_ownership(plan);

    return SB_SUCCESS;
}

/**
 * Initializes an RTH plan object from the contents of a memory buffer.
 *
 * \param plan  the RTH plan to initialize
 * \param buf   the buffer holding the encoded RTH plan object
 * \param nbytes  the length of the buffer
 *
 * \return \c SB_SUCCESS if the object was initialized successfully,
 *         \c SB_ENOENT if the memory buffer did not contain a trajectory
 */
sb_error_t sb_rth_plan_init_from_buffer(sb_rth_plan_t* plan, uint8_t* buf, size_t nbytes)
{
    plan->buffer = buf;
    plan->buffer_length = nbytes;
    plan->owner = 0;

    plan->header_length = sb_i_rth_plan_parse_header(plan);

    return SB_SUCCESS;
}

/**
 * Initializes an empty RTH plan.
 *
 * \param plan  the RTH plan to initialize
 */
sb_error_t sb_rth_plan_init_empty(sb_rth_plan_t* plan)
{
    plan->buffer = 0;
    plan->buffer_length = 0;
    plan->owner = 0;

    plan->scale = 1;
    plan->header_length = 0;
    plan->num_points = 0;

    return SB_SUCCESS;
}

/**
 * @brief Returns the number of entries in the RTH plan.
 *
 * @param plan the RTH plan
 * @return the number of entries in the RTH plan
 */
size_t sb_rth_plan_get_num_entries(const sb_rth_plan_t* plan)
{
    size_t offset = OFFSET_OF_ENTRY_TABLE;
    return offset + 2 <= plan->buffer_length ? sb_parse_uint16(plan->buffer, &offset) : 0;
}

/**
 * @brief Returns the number of unique points in the RTH plan.
 *
 * @param plan the RTH plan
 * @return the number of unique points in the RTH plan
 */
size_t sb_rth_plan_get_num_points(const sb_rth_plan_t* plan)
{
    return plan->num_points;
}

/**
 * @brief Returns the point in the RTH plan with the given index.
 *
 * @param plan   the RTH plan
 * @param index  the index of the point
 * @param point  the point will be returned here
 */
sb_error_t sb_rth_plan_get_point(const sb_rth_plan_t* plan, size_t index, sb_vector2_t* point)
{
    size_t num_points = sb_rth_plan_get_num_points(plan);
    size_t offset;

    if (index >= num_points) {
        return SB_EINVAL;
    }

    offset = OFFSET_OF_POINT(index);
    point->x = sb_i_rth_plan_parse_coordinate(plan, &offset);
    point->y = sb_i_rth_plan_parse_coordinate(plan, &offset);

    return SB_SUCCESS;
}

sb_error_t sb_rth_plan_evaluate_at(const sb_rth_plan_t* plan, float time, sb_rth_plan_entry_t* result)
{
    size_t i, num_entries = sb_rth_plan_get_num_entries(plan);
    size_t offset = OFFSET_OF_FIRST_ENTRY;
    uint32_t time_s = 0;
    sb_bool_t found = 0;
    sb_rth_plan_entry_t entry;
    uint32_t point_index = 0;

    memset(&entry, 0, sizeof(entry));
    entry.action = SB_RTH_ACTION_LAND;

    if (time < 0) {
        found = 1;
    }

    for (i = 0; i < num_entries && !found; i++) {
        uint8_t flags, encoded_action;
        uint32_t time_diff_s;
        uint32_t duration;

        flags = plan->buffer[offset++];

        /* Parse time difference from previous entry to this one */
        SB_CHECK(sb_parse_varuint32(plan->buffer, plan->buffer_length, &offset, &time_diff_s));

        /* Overflow check */
        if (time_diff_s + time_s < time_s) {
            return SB_EOVERFLOW;
        }

        /* Okay, no overflow, we can increase time_s */
        time_s += time_diff_s;

        /* Now, decode the rest of the entry */

        /* Parse flags */
        encoded_action = (flags >> 4) & 0x03;
        switch (encoded_action) {
        case 0:
            /* this means "same as before" so don't touch entry.action */
            break;

        case 1:
            entry.action = SB_RTH_ACTION_LAND;
            break;

        case 2:
            entry.action = SB_RTH_ACTION_GO_TO_KEEPING_ALTITUDE;
            break;

        default:
            return SB_EPARSE;
        }

        if (encoded_action != SB_RTH_ACTION_SAME_AS_PREVIOUS) {
            /* If the action has a target, parse the target and the duration */
            if (sb_i_rth_action_has_target(entry.action)) {
                SB_CHECK(sb_parse_varuint32(plan->buffer, plan->buffer_length, &offset, &point_index));
                SB_CHECK(sb_parse_varuint32(plan->buffer, plan->buffer_length, &offset, &duration));
            } else {
                point_index = 0;
                duration = 0;
            }

            /* Range check for the duration */
            if (duration > MAX_DURATION) {
                return SB_EOVERFLOW;
            }

            entry.duration_sec = duration;

            /* If the action has a pre-delay, parse it */
            if (flags & 0x02) {
                SB_CHECK(sb_parse_varuint32(plan->buffer, plan->buffer_length, &offset, &duration));
                if (duration > MAX_DURATION) {
                    return SB_EOVERFLOW;
                }
                entry.pre_delay_sec = duration;
            } else {
                entry.pre_delay_sec = 0;
            }

            /* If the action has a post-delay, parse it */
            if (flags & 0x01) {
                SB_CHECK(sb_parse_varuint32(plan->buffer, plan->buffer_length, &offset, &duration));
                if (duration > MAX_DURATION) {
                    return SB_EOVERFLOW;
                }
                entry.post_delay_sec = duration;
            } else {
                entry.post_delay_sec = 0;
            }
        }

        /* Check if the time of this entry is at least as large as the the
         * instant that the user is looking for. If it is not, continue the
         * iteration with the next entry */
        if (time_s >= time) {
            break;
        }
    }

    if (sb_i_rth_action_has_target(entry.action)) {
        SB_CHECK(sb_rth_plan_get_point(plan, point_index, &entry.target));
    } else {
        memset(&entry.target, 0, sizeof(entry.target));
    }
    if (result) {
        *result = entry;
    }

    return SB_SUCCESS;
}

/* ************************************************************************** */

static sb_bool_t sb_i_rth_action_has_target(sb_rth_action_t action)
{
    return action == SB_RTH_ACTION_GO_TO_KEEPING_ALTITUDE;
}

static size_t sb_i_rth_plan_parse_header(sb_rth_plan_t* plan)
{
    uint8_t* buf = plan->buffer;
    size_t offset;

    assert(buf != 0);

    plan->scale = (float)((buf[0] & 0x7f));

    offset = 1;
    plan->num_points = sb_parse_uint16(buf, &offset);

    return offset; /* size of the header */
}

static void sb_i_rth_plan_take_ownership(sb_rth_plan_t* plan)
{
    plan->owner = 1;
}

static float sb_i_rth_plan_parse_coordinate(const sb_rth_plan_t* plan, size_t* offset)
{
    return sb_parse_int16(plan->buffer, offset) * plan->scale;
}
