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

static int sb_i_event_cmp_timestamps(const void* first, const void* second);

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
 * @brief Returns whether the event list is sorted by timestamp.
 *
 * @param events  the event list to query
 * @return sb_bool_t  true if the event list is sorted, false otherwise
 */
sb_bool_t sb_event_list_is_sorted(const sb_event_list_t* events)
{
    if (events->num_entries < 2) {
        return 1; /* An empty list or a list with one event is sorted */
    }

    for (size_t i = 1; i < events->num_entries; i++) {
        if (events->entries[i].time_msec < events->entries[i - 1].time_msec) {
            return 0;
        }
    }

    return 1;
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
 * @brief Inserts a new event in the event list.
 *
 * The timestamp of the event determines the location where the event will be
 * inserted in the event list: it will be inserted after the \em latest event
 * that has a timestamp \em smaller than or equal to the event being inserted.
 * last event in the list.
 *
 * @param events  the event list to insert the event into
 * @param event   the event to insert. It will be copied into the list.
 * @return \c SB_SUCCESS if the event was inserted successfully,
 *         \c SB_ENOMEM if a memory allocation failed,
 *         \c SB_EINVAL if the event's timestamp is earlier than the last event
 */
sb_error_t sb_event_list_insert(sb_event_list_t* events, const sb_event_t* event)
{
    size_t n = events->num_entries;
    size_t left = 0, right = n;
    size_t insert_pos = 0;

    SB_CHECK(sb_i_event_list_ensure_has_free_space(events));

    // Binary search for the correct insertion point
    while (left < right) {
        size_t mid = left + (right - left) / 2;
        if (events->entries[mid].time_msec <= event->time_msec) {
            left = mid + 1;
        } else {
            right = mid;
        }
    }
    insert_pos = left;

    // Shift events after insert_pos to the right
    if (insert_pos < n) {
        memmove(
            &events->entries[insert_pos + 1],
            &events->entries[insert_pos],
            (n - insert_pos) * sizeof(sb_event_t));
    }
    events->entries[insert_pos] = *event;
    events->num_entries++;

    return SB_SUCCESS;
}

/**
 * @brief Removes an event from the event list at the given index.
 *
 * @param events  the event list to remove the event from
 * @param index   the index of the event to remove
 * @return \c SB_SUCCESS if the event was removed successfully,
 *         \c SB_EINVAL if the index is out of bounds
 */
sb_error_t sb_event_list_remove(sb_event_list_t* events, size_t index)
{
    if (index >= events->num_entries) {
        return SB_EINVAL;
    }

    // Shift events after index to the left
    if (index < events->num_entries - 1) {
        memmove(
            &events->entries[index],
            &events->entries[index + 1],
            (events->num_entries - index - 1) * sizeof(sb_event_t));
    }
    events->num_entries--;

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

static int sb_i_event_cmp_timestamps(const void* first, const void* second)
{
    const sb_event_t* event1 = (const sb_event_t*)first;
    const sb_event_t* event2 = (const sb_event_t*)second;

    if (event1->time_msec < event2->time_msec) {
        return -1;
    } else if (event1->time_msec > event2->time_msec) {
        return 1;
    } else {
        return 0; /* Timestamps are equal */
    }
}

/**
 * @brief Sorts the event list by timestamp.
 *
 * The sorting is not guaranteed to be stable, i.e. events with the same
 * timestamp may be reordered.
 *
 * @param events  the event list to sort
 */
void sb_event_list_sort(sb_event_list_t* events)
{
    qsort(events->entries, events->num_entries, sizeof(sb_event_t), sb_i_event_cmp_timestamps);
}

static sb_bool_t is_pyro_on_event(const sb_event_t* event)
{
    return event->type == SB_EVENT_TYPE_PYRO && event->payload.as_uint32 != UINT32_MAX;
}

static sb_bool_t is_pyro_off_event(const sb_event_t* event)
{
    return event->type == SB_EVENT_TYPE_PYRO && event->payload.as_uint32 == UINT32_MAX;
}

/**
 * @brief Helper function to add "pyro off" events to the event list.
 *
 * This function adds "pyro off" events to the event list for all "pyro on"
 * events that have no corresponding "off" event in the event list or where the
 * "off" event is farther ahead in time than the given duration. The "off"
 * events are added after the "on" events with the given duration.
 *
 * @param events     the event list to adjust
 * @param time_msec  duration of each pyro ignition event
 */
sb_error_t sb_event_list_add_pyro_off_events(sb_event_list_t* events, uint32_t time_msec)
{
    size_t i, j, n = sb_event_list_size(events);
    sb_event_t *event, *other_event;

    for (i = 0; i < n; i++) {
        event = sb_event_list_get_ptr(events, i);
        if (is_pyro_on_event(event)) {
            // Check if there is already a matching "off" event
            sb_bool_t found_off_event = 0;
            for (j = i + 1; j < n; j++) {
                other_event = sb_event_list_get_ptr(events, j);
                if (is_pyro_off_event(other_event) && other_event->subtype == event->subtype) {
                    found_off_event = 1;
                    break;
                }
            }

            // If a matching "off" event was found, check if it is too far in the future
            if (found_off_event && other_event->time_msec > event->time_msec + time_msec) {
                // Remove the found "off" event
                SB_CHECK(sb_event_list_remove(events, j));
                found_off_event = 0; // Reset flag since we removed the event
                n--; // Adjust the size of the list since we removed an event
            }

            // If no matching "off" event was found, create one
            if (!found_off_event) {
                sb_event_t off_event;
                off_event.time_msec = event->time_msec + time_msec;
                off_event.type = SB_EVENT_TYPE_PYRO;
                off_event.subtype = event->subtype;
                off_event.payload.as_uint32 = UINT32_MAX; // Indicating "off"

                SB_CHECK(sb_event_list_insert(events, &off_event));

                // No need to adjust 'i' because the event list is sorted and
                // the addition above is _after_ the current "on" event.

                n++; // Adjust the size of the list since we added an event
            }
        }
    }

    return SB_SUCCESS;
}

/**
 * @brief Adjusts the timestamps of events with a given type in the list by a given delta.
 *
 * This function can be used to adjust the timestamps of pyro events in the list
 * if the pyro device needs to be activated a given number of milliseconds earlier
 * than the desired time of the event.
 *
 * Note that even with negative adjustments the timestamp may not be earlier than
 * zero seconds. Timestamps that would become negative will be clamped to zero.
 *
 * @param events      the event list to adjust
 * @param type        the type of events to adjust
 * @param delta_msec  the number of milliseconds to adjust the timestamps by
 *                    (can be negative to move the events earlier)
 */
void sb_event_list_adjust_timestamps_by_type(
    sb_event_list_t* events, sb_event_type_t type, int32_t delta_msec)
{
    size_t n = sb_event_list_size(events);
    sb_event_t* event;

    for (size_t i = 0; i < n; i++) {
        event = sb_event_list_get_ptr(events, i);
        if (event->type == type) {
            if (delta_msec > 0) {
                /* If the adjustment would overflow, clamp to UINT32_MAX */
                if (event->time_msec <= UINT32_MAX - delta_msec) {
                    event->time_msec += delta_msec;
                } else {
                    event->time_msec = UINT32_MAX;
                }
            } else if (delta_msec < 0) {
                /* If the adjustment is negative, ensure we don't go below zero */
                if (event->time_msec >= (uint32_t)(-delta_msec)) {
                    event->time_msec += delta_msec;
                } else {
                    event->time_msec = 0;
                }
            }
        }
    }

    sb_event_list_sort(events);
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
