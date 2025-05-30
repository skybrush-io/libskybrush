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

/**
 * @file events.c
 * @brief Handling of timeline events in Skybrush missions.
 */

#include <math.h>
#include <skybrush/events.h>
#include <skybrush/formats/binary.h>
#include <skybrush/memory.h>
#include <string.h>

#include "../parsing.h"

static sb_error_t sb_i_event_list_extend_from_bytes(sb_event_list_t* events, uint8_t* buf, size_t nbytes, sb_bool_t owned);
static sb_error_t sb_i_event_list_extend_from_parser(sb_event_list_t* events, sb_binary_file_parser_t* parser);
static sb_error_t sb_i_event_list_ensure_has_free_space(sb_event_list_t* events);

/**
 * \brief Initializes an event list.
 *
 * \param events  the event list to initialize
 * @param max_events  the maximum number of events that can be stored in the
 *        list. Use \ref sb_event_list_resize() to change the capacity of the list
 *        later.
 * @return \c SB_SUCCESS if the list was initialized successfully,
 *         \c SB_ENOMEM if memory allocation failed
 */
sb_error_t sb_event_list_init(sb_event_list_t* events, size_t max_events)
{
    if (max_events < 1) {
        max_events = 1;
    }

    events->entries = sb_calloc(sb_event_t, max_events);
    if (events->entries == 0) {
        return SB_ENOMEM; /* LCOV_EXCL_LINE */
    }

    events->num_entries = 0;
    events->max_entries = max_events;

    return SB_SUCCESS;
}

/**
 * \brief Destroys an event list, freeing all associated memory.
 *
 * \param events  the event list to destroy
 */
void sb_event_list_destroy(sb_event_list_t* events)
{
    if (events->entries != 0) {
        sb_free(events->entries);
        events->entries = 0;
    }

    events->num_entries = 0;
    events->max_entries = 0;
}

/**
 * \brief Removes all events from the event list.
 *
 * \param events  the event list to clear
 */
void sb_event_list_clear(sb_event_list_t* events)
{
    events->num_entries = 0;
}

/**
 * \brief Returns the capacity of the event list.
 *
 * The capacity is the maximum number of events that can be stored in the
 * list without having to reallocate memory.
 *
 * \param events  the event list to query
 * \return the capacity of the event list
 */
size_t sb_event_list_capacity(const sb_event_list_t* events)
{
    return events->max_entries;
}

/**
 * \brief Returns the number of events in the event list.
 *
 * \param events  the event list to query
 * \return the number of events in the event list
 */
size_t sb_event_list_size(const sb_event_list_t* events)
{
    return events->num_entries;
}

/**
 * \brief Returns whether the event list is empty (i.e. has no events).
 *
 * \param events  the event list to clear
 * \return true if the event list has no entries, false otherwise
 */
sb_bool_t sb_event_list_is_empty(const sb_event_list_t* events)
{
    return sb_event_list_size(events) == 0;
}

/**
 * \brief Returns a pointer to the event at the given index in the event list.
 *
 * You can modify the event via the provided pointer, but you should not
 * change its timestamp in a way that would make the event list unsorted.
 *
 * \param events  the event list to query
 * \param index   the index of the event to return
 * \return a pointer to the event at the given index
 */
sb_event_t* sb_event_list_get_ptr(sb_event_list_t* events, size_t index)
{
    if (index >= events->num_entries) {
        return NULL;
    } else {
        return &events->entries[index];
    }
}

/**
 * \brief Returns a pointer to the event at the given index in the event list (const variant).
 *
 * The event returned by the provided pointer should not be modified.
 *
 * \param events  the event list to query
 * \param index   the index of the event to return
 * \return a pointer to the event at the given index
 */
const sb_event_t* sb_event_list_get_ptr_const(const sb_event_list_t* events, size_t index)
{
    if (index >= events->num_entries) {
        return NULL;
    } else {
        return &events->entries[index];
    }
}

/**
 * @brief Appends a new event to the end of the event list.
 *
 * The timestamp of the event must not be earlier than the timestamp of the
 * last event in the list.
 *
 * @param events  the event list to append the event to
 * @param event   the event to append. It will be copied into the list.
 * @return \c SB_SUCCESS if the event was appended successfully,
 *         \c SB_ENOMEM if a memory allocation failed,
 *         \c SB_EINVAL if the event's timestamp is earlier than the last event
 */
sb_error_t sb_event_list_append(sb_event_list_t* events, const sb_event_t* event)
{
    if (events->num_entries > 0) {
        sb_event_t* last_event = &events->entries[events->num_entries - 1];
        if (event->time_msec < last_event->time_msec) {
            return SB_EINVAL;
        }
    }

    SB_CHECK(sb_i_event_list_ensure_has_free_space(events));

    events->entries[events->num_entries] = *event;
    events->num_entries++;

    return SB_SUCCESS;
}

/**
 * Initializes an event list from the contents of a Skybrush file in
 * binary format.
 *
 * \param events  the event list to initialize
 * \param fd  handle to the low-level file object to initialize the list from
 *
 * \return \c SB_SUCCESS if the list was initialized successfully,
 *         \c SB_ENOENT if the file did not contain an event list block,
 *         \c SB_EREAD for read errors
 */
sb_error_t sb_event_list_update_from_binary_file(sb_event_list_t* events, int fd)
{
    sb_binary_file_parser_t parser;
    sb_error_t retval;

    SB_CHECK(sb_binary_file_parser_init_from_file(&parser, fd));
    sb_event_list_clear(events);
    retval = sb_i_event_list_extend_from_parser(events, &parser);
    sb_binary_file_parser_destroy(&parser);

    return retval;
}

/**
 * Initializes an event list from the contents of a Skybrush file in binary
 * format, already loaded into memory.
 *
 * \param events  the event list to initialize
 * \param buf     the buffer holding the loaded Skybrush file in binary format
 * \param nbytes  the length of the buffer
 *
 * \return \c SB_SUCCESS if the object was initialized successfully,
 *         \c SB_ENOENT if the memory block did not contain an event list block
 */
sb_error_t sb_event_list_update_from_binary_file_in_memory(
    sb_event_list_t* events, uint8_t* buf, size_t nbytes)
{
    sb_binary_file_parser_t parser;
    sb_error_t retval;

    SB_CHECK(sb_binary_file_parser_init_from_buffer(&parser, buf, nbytes));
    sb_event_list_clear(events);
    retval = sb_i_event_list_extend_from_parser(events, &parser);
    sb_binary_file_parser_destroy(&parser);

    return retval;
}

/* ************************************************************************** */

/**
 * \brief Ensures that the event list has enough free space to store a new
 * event.
 *
 * If the list does not have enough free space, it will be resized to
 * accommodate the new event.
 *
 * \param events  the event list to check
 * \return \c SB_SUCCESS if there is enough free space,
 *         \c SB_ENOMEM if memory allocation failed
 */
static sb_error_t sb_i_event_list_ensure_has_free_space(sb_event_list_t* events)
{
    size_t free_space = events->max_entries - events->num_entries;

    if (free_space == 0) {
        size_t new_capacity = events->max_entries * 2;
        sb_event_t* new_entries = sb_realloc(events->entries, sb_event_t, new_capacity);
        if (new_entries == 0) {
            return SB_ENOMEM; /* LCOV_EXCL_LINE */
        }

        events->entries = new_entries;
        events->max_entries = new_capacity;
    }

    return SB_SUCCESS;
}

static sb_error_t sb_i_event_list_extend_from_bytes(sb_event_list_t* events,
    uint8_t* buf, size_t nbytes, sb_bool_t owned)
{
    /* Each entry in the serialized representation of the event list is 10 bytes
     * as follows:
     * - timestamp (4 bytes, little endian)
     * - event type (1 byte)
     * - event subtype (1 byte)
     * - payload (4 bytes) */
    uint8_t* end = buf + nbytes;

    while (buf + 10 <= end) {
        sb_event_t event;
        size_t offset = 0;

        event.time_msec = sb_parse_uint32(buf, &offset);
        event.type = (sb_event_type_t)sb_parse_uint8(buf, &offset);
        if (event.type >= SB_EVENT_TYPE_MAX) {
            return SB_EINVAL;
        }

        event.subtype = (sb_event_subtype_t)sb_parse_uint8(buf, &offset);
        memcpy(event.payload.as_buf, buf + offset, sizeof(event.payload.as_buf));

        SB_CHECK(sb_event_list_append(events, &event));

        buf += 10;
    }

    return SB_SUCCESS;
}

static sb_error_t sb_i_event_list_extend_from_parser(sb_event_list_t* events,
    sb_binary_file_parser_t* parser)
{
    sb_error_t retval;
    uint8_t* buf;
    size_t size;
    sb_bool_t owned;

    SB_CHECK(sb_binary_file_find_first_block_by_type(parser, SB_BINARY_BLOCK_EVENT_LIST));
    SB_CHECK(sb_binary_file_read_current_block_ex(parser, &buf, &size, &owned));

    retval = sb_i_event_list_extend_from_bytes(events, buf, size, owned);

    if (owned) {
        sb_free(buf);
    }

    return retval;
}

/* ************************************************************************** */

sb_error_t sb_event_list_player_init(sb_event_list_player_t* player,
    const sb_event_list_t* events)
{
    player->events = events;
    player->current_index = 0;
    return SB_SUCCESS;
}

void sb_event_list_player_destroy(sb_event_list_player_t* player)
{
    /* nop */
}

const sb_event_t* sb_event_list_player_peek_next_event(const sb_event_list_player_t* player)
{
    return sb_event_list_get_ptr_const(player->events, player->current_index);
}

const sb_event_t* sb_event_list_player_get_next_event(sb_event_list_player_t* player)
{
    const sb_event_t* result = sb_event_list_player_peek_next_event(player);
    if (result != NULL) {
        player->current_index++;
    }
    return result;
}

const sb_event_t* sb_event_list_player_get_next_event_not_later_than(sb_event_list_player_t* player, float t)
{
    const sb_event_t* result = sb_event_list_player_peek_next_event(player);
    if (result != NULL) {
        if (result->time_msec <= t * 1000) {
            player->current_index++;
        } else {
            result = NULL;
        }
    }
    return result;
}

void sb_event_list_player_rewind(sb_event_list_player_t* player)
{
    sb_event_list_player_seek(player, 0);
}

void sb_event_list_player_seek(sb_event_list_player_t* player, float t)
{
    if (!isfinite(t) || t <= 0) {
        player->current_index = 0;
        return;
    }

    if (t > UINT32_MAX / 1000) {
        player->current_index = player->events->num_entries;
        return;
    }

    uint32_t t_msec = t * 1000;

    player->current_index = 0;

    /* TODO(ntamas): use binary search if this becomes a bottleneck */
    while (player->current_index < player->events->num_entries) {
        const sb_event_t* event = sb_event_list_get_ptr_const(player->events, player->current_index);
        if (event->time_msec >= t_msec) {
            break;
        }
        player->current_index++;
    }
}
