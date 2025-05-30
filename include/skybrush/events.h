/*
 * This file is part of libskybrush.
 *
 * Copyright 2025 CollMot Robotics Ltd.
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

#ifndef SKYBRUSH_EVENTS_H
#define SKYBRUSH_EVENTS_H

#include <skybrush/basic_types.h>
#include <skybrush/decls.h>
#include <skybrush/error.h>
#include <stdint.h>
#include <stdlib.h>

__BEGIN_DECLS

/**
 * @file events.h
 * @brief Handling of time-triggered events in Skybrush missions.
 */

/**
 * @brief Types of events that can be placed on the timeline.
 */
typedef enum {
    SB_EVENT_TYPE_NONE = 0, /**< No event */
    SB_EVENT_TYPE_PYRO = 1, /**< Pyro trigger event */
    SB_EVENT_TYPE_MAX /**< Maximum number of event types */
} sb_event_type_t;

typedef uint8_t sb_event_subtype_t;

/**
 * @brief Structure describing a single event that is to be triggered at
 * a specific point on the timeline.
 */
typedef struct sb_event_s {
    /** The timestamp when the event should be triggered, in milliseconds */
    uint32_t time_msec;

    /** The type of the event */
    sb_event_type_t type;

    /**
     * The subtype of the event.
     *
     * Its interpretation depends on the event type; for instance, for pyro
     * events it is the index of the pyro channel that should be triggered.
     */
    sb_event_subtype_t subtype;

    /**
     * The payload of the event, if applicable.
     */
    union {
        /** The payload as an array of four bytes */
        uint8_t as_buf[4];

        /** The payload as a 32-bit unsigned integer */
        uint32_t as_uint32;

        /** The payload as an IEEE float */
        float as_float;
    } payload;
} sb_event_t;

/**
 * @brief Structure representing the list of events in a Skybrush mission.
 *
 * It is assumed that the number of events is relatively small so they can be
 * parsed in advance. It is also assumed that they are ordered in increasing
 * order of timestamps.
 */
typedef struct sb_event_list_s {
    sb_event_t* entries; /**< The list of events */
    size_t num_entries; /**< The current number of events in the list */
    size_t max_entries; /**< The maximum number of events that the list can hold */
} sb_event_list_t;

sb_error_t sb_event_list_init(sb_event_list_t* events, size_t max_events);
void sb_event_list_destroy(sb_event_list_t* events);

void sb_event_list_clear(sb_event_list_t* events);
size_t sb_event_list_capacity(const sb_event_list_t* events);
sb_bool_t sb_event_list_is_empty(const sb_event_list_t* events);
size_t sb_event_list_size(const sb_event_list_t* events);

sb_event_t* sb_event_list_get_ptr(sb_event_list_t* events, size_t index);
const sb_event_t* sb_event_list_get_ptr_const(const sb_event_list_t* events, size_t index);

sb_error_t sb_event_list_append(sb_event_list_t* events, const sb_event_t* event);

sb_error_t sb_event_list_update_from_binary_file(sb_event_list_t* events, int fd);
sb_error_t sb_event_list_update_from_binary_file_in_memory(sb_event_list_t* events, uint8_t* buf, size_t nbytes);
sb_error_t sb_event_list_update_from_buffer(sb_event_list_t* events, uint8_t* buf, size_t nbytes);

sb_bool_t sb_event_list_is_sorted(const sb_event_list_t* events);
void sb_event_list_sort(sb_event_list_t* events);

void sb_event_list_adjust_timestamps_by_type(
    sb_event_list_t* events, sb_event_type_t type, int32_t delta_msec);

/* ************************************************************************* */

/**
 * Structure representing an event player that allows us to query the upcoming
 * list of events in a lookahead window.
 */
typedef struct sb_event_list_player_s {
    const sb_event_list_t* events; /**< The event list */
    size_t current_index; /**< The index of the current event */
} sb_event_list_player_t;

sb_error_t sb_event_list_player_init(sb_event_list_player_t* player, const sb_event_list_t* events);
void sb_event_list_player_destroy(sb_event_list_player_t* player);
const sb_event_t* sb_event_list_player_get_next_event(sb_event_list_player_t* player);
const sb_event_t* sb_event_list_player_get_next_event_not_later_than(
    sb_event_list_player_t* player, float t);
const sb_event_t* sb_event_list_player_peek_next_event(const sb_event_list_player_t* player);
void sb_event_list_player_rewind(sb_event_list_player_t* player);
void sb_event_list_player_seek(sb_event_list_player_t* player, float t);

__END_DECLS

#endif
