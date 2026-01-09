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

#include "skybrush/error.h"
#include <math.h>
#include <skybrush/refcount.h>
#include <skybrush/screenplay.h>
#include <skybrush/time_axis.h>

static void sb_i_screenplay_chapter_destroy(sb_screenplay_chapter_t* chapter);

/**
 * @brief Initializes a screenplay chapter with default values.
 *
 * @param chapter  the screenplay chapter to initialize
 * @return \c SB_SUCCESS if the chapter was initialized successfully,
 *         \c SB_ENOMEM if a memory allocation failed
 */
sb_error_t sb_screenplay_chapter_init(sb_screenplay_chapter_t* chapter)
{
    SB_CHECK(sb_time_axis_init(&chapter->time_axis));

    chapter->duration_msec = UINT32_MAX; // Infinite duration by default

    chapter->trajectory = NULL;
    chapter->light_program = NULL;
    chapter->yaw_control = NULL;
    chapter->events = NULL;

    SB_REF_INIT(chapter, sb_i_screenplay_chapter_destroy);

    return SB_SUCCESS;
}

/**
 * @brief Destroys a screenplay chapter and releases its resources.
 *
 * @param chapter  the screenplay chapter to destroy
 */
static void sb_i_screenplay_chapter_destroy(sb_screenplay_chapter_t* chapter)
{
    sb_screenplay_chapter_reset(chapter);
    sb_time_axis_destroy(&chapter->time_axis);
}

/**
 * @brief Returns a pointer to the time axis of the given screenplay chapter.
 *
 * @param chapter  the screenplay chapter to query
 * @return a pointer to the time axis of the chapter
 */
sb_time_axis_t* sb_screenplay_chapter_get_time_axis(sb_screenplay_chapter_t* chapter)
{
    return &chapter->time_axis;
}

/**
 * @brief Returns the trajectory for a chapter.
 *
 * @param chapter    the chapter to query
 * @return the trajectory of the chapter; may be NULL
 */
sb_trajectory_t* sb_screenplay_chapter_get_trajectory(sb_screenplay_chapter_t* chapter)
{
    return chapter->trajectory;
}

/**
 * @brief Returns the light program for a chapter.
 *
 * @param chapter    the chapter to query
 * @return the light program of the chapter; may be NULL
 */
sb_light_program_t* sb_screenplay_chapter_get_light_program(sb_screenplay_chapter_t* chapter)
{
    return chapter->light_program;
}

/**
 * @brief Returns the yaw control object for a chapter.
 *
 * @param chapter    the chapter to query
 * @return the yaw control object of the chapter; may be NULL
 */
sb_yaw_control_t* sb_screenplay_chapter_get_yaw_control(sb_screenplay_chapter_t* chapter)
{
    return chapter->yaw_control;
}

/**
 * @brief Returns the event list for a chapter.
 *
 * @param chapter  the chapter to query
 * @return the event list of the chapter; may be NULL
 */
sb_event_list_t* sb_screenplay_chapter_get_events(sb_screenplay_chapter_t* chapter)
{
    return chapter->events;
}

/**
 * @brief Returns the duration of a chapter in milliseconds.
 *
 * @param chapter  the chapter to query
 * @return the duration of the chapter in milliseconds; \c UINT32_MAX if the duration is infinite
 */
uint32_t sb_screenplay_chapter_get_duration_msec(
    const sb_screenplay_chapter_t* chapter)
{
    return chapter->duration_msec;
}

/**
 * @brief Returns the duration of a chapter in seconds.
 *
 * @param chapter  the chapter to query
 * @return the duration of the chapter in seconds; \c INFINITY if the duration is infinite
 */
float sb_screenplay_chapter_get_duration_sec(
    const sb_screenplay_chapter_t* chapter)
{
    if (chapter->duration_msec == UINT32_MAX) {
        return INFINITY;
    } else {
        return chapter->duration_msec / 1000.0f;
    }
}

/**
 * @brief Sets the trajectory for a chapter.
 *
 * @param chapter    the chapter to modify
 * @param trajectory the trajectory to set; may be NULL
 */
void sb_screenplay_chapter_set_trajectory(
    sb_screenplay_chapter_t* chapter, sb_trajectory_t* trajectory)
{
    SB_XINCREF(trajectory);
    SB_XDECREF(chapter->trajectory);
    chapter->trajectory = trajectory;
}

/**
 * @brief Sets the light program for a chapter.
 *
 * @param chapter    the chapter to modify
 * @param light_program the light program to set; may be NULL
 */
void sb_screenplay_chapter_set_light_program(
    sb_screenplay_chapter_t* chapter, sb_light_program_t* light_program)
{
    SB_XINCREF(light_program);
    SB_XDECREF(chapter->light_program);
    chapter->light_program = light_program;
}

/**
 * @brief Sets the yaw control object for a chapter.
 *
 * @param chapter    the chapter to modify
 * @param yaw_control the yaw control object to set; may be NULL
 */
void sb_screenplay_chapter_set_yaw_control(
    sb_screenplay_chapter_t* chapter, sb_yaw_control_t* yaw_control)
{
    SB_XINCREF(yaw_control);
    SB_XDECREF(chapter->yaw_control);
    chapter->yaw_control = yaw_control;
}

/**
 * @brief Sets the event list for a chapter.
 *
 * @param chapter  the chapter to modify
 * @param events   the event list to set; may be NULL
 */
void sb_screenplay_chapter_set_events(
    sb_screenplay_chapter_t* chapter, sb_event_list_t* events)
{
    SB_XINCREF(events);
    SB_XDECREF(chapter->events);
    chapter->events = events;
}

/**
 * @brief Sets the duration of a chapter in milliseconds.
 *
 * @param chapter       the chapter to modify
 * @param duration_msec the duration to set in milliseconds; use \c UINT32_MAX for infinite duration
 * @return \c SB_SUCCESS if the duration was set successfully
 *         \c SB_EINVAL if the duration is invalid
 */
sb_error_t sb_screenplay_chapter_set_duration_msec(
    sb_screenplay_chapter_t* chapter, uint32_t duration_msec)
{
    chapter->duration_msec = duration_msec;
    return SB_SUCCESS;
}

/**
 * @brief Sets the duration of a chapter in seconds.
 *
 * @param chapter       the chapter to modify
 * @param duration_sec  the duration to set in seconds; use \c INFINITY for infinite duration
 * @return \c SB_SUCCESS if the duration was set successfully
 *         \c SB_EINVAL if the duration is invalid or too large to fit in a \c uint32_t
 */
sb_error_t sb_screenplay_chapter_set_duration_sec(
    sb_screenplay_chapter_t* chapter, float duration_sec)
{
    if (isinf(duration_sec) && duration_sec > 0) {
        chapter->duration_msec = UINT32_MAX;
    } else if (duration_sec < 0 || !isfinite(duration_sec)) {
        return SB_EINVAL;
    } else {
        float duration_msec_f = duration_sec * 1000.0f;

        // Largest uint32_t that can be represented exactly in a 32-bit
        // float is 4294967040 (2^32 - 2^8). Any value larger than this
        // should fail.
        if (duration_msec_f > 4294967040) {
            return SB_EINVAL;
        }

        uint32_t duration_msec_u = (uint32_t)(duration_msec_f + 0.5f);
        if (duration_msec_u == UINT32_MAX) {
            return SB_EINVAL;
        }

        chapter->duration_msec = duration_msec_u;
    }
    return SB_SUCCESS;
}

/**
 * @brief Resets the screenplay chapter to its default state.
 *
 * All associated objects of the screenplay chapter will be cleared. The time axis
 * will be reset to its initial state with no segments and origin at 0 ms. The duration
 * will be set to infinite.
 *
 * @param chapter  the screenplay chapter to reset
 */
void sb_screenplay_chapter_reset(sb_screenplay_chapter_t* chapter)
{
    sb_screenplay_chapter_set_trajectory(chapter, NULL);
    sb_screenplay_chapter_set_light_program(chapter, NULL);
    sb_screenplay_chapter_set_yaw_control(chapter, NULL);
    sb_screenplay_chapter_set_events(chapter, NULL);
    sb_screenplay_chapter_set_duration_msec(chapter, UINT32_MAX);
    sb_time_axis_clear(&chapter->time_axis);
}

/**
 * @brief Updates a screenplay chapter from a binary show file in memory.
 *
 * This function updates the trajectory, light program, yaw control and event list
 * of the given screenplay chapter by parsing the provided binary show file data.
 * If any of these components are not present in the show file, the corresponding
 * component in the chapter will be set to NULL. The trajectory is considered mandatory;
 * if it cannot be loaded, an error will be returned.
 *
 * The duration of the screenplay chapter will be set to infinity. The time axis
 * will be reset. In other words, assume that `sb_screenplay_chapter_reset()` is called
 * before loading the components from the show file.
 *
 * @param chapter    the screenplay chapter to update
 * @param show_data  pointer to the binary show file data in memory. May be NULL.
 *        The ownership of the data is not taken; the caller is responsible for
 *        managing the memory of the data and to keep it alive while the screenplay
 *        chapter is using it.
 * @param length     length of the binary show file data in bytes
 */
sb_error_t sb_screenplay_chapter_update_from_binary_file_in_memory(
    sb_screenplay_chapter_t* chapter, uint8_t* show_data, size_t length)
{
    sb_error_t retval;
    sb_trajectory_t* trajectory = NULL;
    sb_light_program_t* light_program = NULL;
    sb_yaw_control_t* yaw_control = NULL;
    sb_event_list_t* event_list = NULL;

    sb_screenplay_chapter_reset(chapter);
    if (!show_data || length == 0) {
        return SB_SUCCESS;
    }

    // ---------------------------------------------------------------------------------
    // Loading trajectory
    // ---------------------------------------------------------------------------------

    trajectory = sb_trajectory_new();
    if (!trajectory) {
        /* LCOV_EXCL_START */
        retval = SB_ENOMEM;
        goto exit;
        /* LCOV_EXCL_STOP */
    }

    retval = sb_trajectory_update_from_binary_file_in_memory(trajectory, show_data, length);
    if (retval) {
        // Error while loading trajectory or no trajectory in show file
        goto exit;
    }

    sb_screenplay_chapter_set_trajectory(chapter, trajectory);

    // ---------------------------------------------------------------------------------
    // Loading light program
    // ---------------------------------------------------------------------------------

    light_program = sb_light_program_new();
    if (!light_program) {
        /* LCOV_EXCL_START */
        retval = SB_ENOMEM;
        goto exit;
        /* LCOV_EXCL_STOP */
    }

    retval = sb_light_program_update_from_binary_file_in_memory(light_program, show_data, length);
    if (retval == SB_SUCCESS) {
        // Light program loaded successfully
        sb_screenplay_chapter_set_light_program(chapter, light_program);
    } else if (retval == SB_ENOENT) {
        // No light program in show file, this is okay
    } else {
        // Some other error
        goto exit;
    }

    // ---------------------------------------------------------------------------------
    // Loading yaw control data
    // ---------------------------------------------------------------------------------

    yaw_control = sb_yaw_control_new();
    if (!yaw_control) {
        /* LCOV_EXCL_START */
        retval = SB_ENOMEM;
        goto exit;
        /* LCOV_EXCL_STOP */
    }

    retval = sb_yaw_control_update_from_binary_file_in_memory(yaw_control, show_data, length);
    if (retval == SB_SUCCESS) {
        // Yaw control loaded successfully
        sb_screenplay_chapter_set_yaw_control(chapter, yaw_control);
    } else if (retval == SB_ENOENT) {
        // No yaw control in show file, this is okay
    } else {
        // Some other error
        goto exit;
    }

    // ---------------------------------------------------------------------------------
    // Loading event list
    // ---------------------------------------------------------------------------------

    event_list = sb_event_list_new(4);
    if (!event_list) {
        /* LCOV_EXCL_START */
        retval = SB_ENOMEM;
        goto exit;
        /* LCOV_EXCL_STOP */
    }

    retval = sb_event_list_update_from_binary_file_in_memory(event_list, show_data, length);
    if (retval == SB_SUCCESS) {
        // Event list loaded successfully
        sb_screenplay_chapter_set_events(chapter, event_list);
    } else if (retval == SB_ENOENT) {
        // No event list in show file, this is okay
    } else {
        // Some other error
        goto exit;
    }

    retval = SB_SUCCESS;

exit:
    SB_XDECREF(trajectory);
    SB_XDECREF(light_program);
    SB_XDECREF(yaw_control);
    SB_XDECREF(event_list);

    return retval;
}
