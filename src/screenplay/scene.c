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

#include <math.h>
#include <skybrush/error.h>
#include <skybrush/refcount.h>
#include <skybrush/screenplay.h>
#include <skybrush/time_axis.h>

static void sb_i_screenplay_scene_destroy(sb_screenplay_scene_t* scene);

/**
 * \brief Allocates a new scene on the heap and initializes it.
 *
 * \return the new scene, or \c NULL if memory allocation failed
 */
sb_screenplay_scene_t* sb_screenplay_scene_new(void)
{
    sb_screenplay_scene_t* obj = sb_calloc(sb_screenplay_scene_t, 1);

    if (obj) {
        if (sb_screenplay_scene_init(obj)) {
            sb_free(obj);
        }
    }

    return obj;
}

/**
 * @brief Initializes an already allocated screenplay scene with default values.
 *
 * You must call this function on an uninitialized screenplay scene before using it.
 * \ref sb_screenplay_scene_new() takes care of the initialization for you if you
 * allocate the scene on the heap.
 *
 * @param scene  the screenplay scene to initialize
 * @return \c SB_SUCCESS if the scene was initialized successfully,
 *         \c SB_ENOMEM if a memory allocation failed
 */
sb_error_t sb_screenplay_scene_init(sb_screenplay_scene_t* scene)
{
    SB_CHECK(sb_time_axis_init(&scene->time_axis));

    scene->duration_msec = UINT32_MAX; // Infinite duration by default

    scene->trajectory = NULL;
    scene->light_program = NULL;
    scene->yaw_control = NULL;
    scene->events = NULL;

    SB_REF_INIT(scene, sb_i_screenplay_scene_destroy);

    return SB_SUCCESS;
}

/**
 * @brief Destroys a screenplay scene and releases its resources.
 *
 * @param scene  the screenplay scene to destroy
 */
static void sb_i_screenplay_scene_destroy(sb_screenplay_scene_t* scene)
{
    sb_screenplay_scene_reset(scene);
    sb_time_axis_destroy(&scene->time_axis);
}

/**
 * @brief Returns a pointer to the time axis of the given screenplay scene.
 *
 * @param scene  the screenplay scene to query
 * @return a pointer to the time axis of the scene
 */
sb_time_axis_t* sb_screenplay_scene_get_time_axis(sb_screenplay_scene_t* scene)
{
    return &scene->time_axis;
}

/**
 * @brief Returns the trajectory for a scene.
 *
 * @param scene    the scene to query
 * @return the trajectory of the scene; may be NULL
 */
sb_trajectory_t* sb_screenplay_scene_get_trajectory(sb_screenplay_scene_t* scene)
{
    return scene->trajectory;
}

/**
 * @brief Returns the light program for a scene.
 *
 * @param scene    the scene to query
 * @return the light program of the scene; may be NULL
 */
sb_light_program_t* sb_screenplay_scene_get_light_program(sb_screenplay_scene_t* scene)
{
    return scene->light_program;
}

/**
 * @brief Returns the yaw control object for a scene.
 *
 * @param scene    the scene to query
 * @return the yaw control object of the scene; may be NULL
 */
sb_yaw_control_t* sb_screenplay_scene_get_yaw_control(sb_screenplay_scene_t* scene)
{
    return scene->yaw_control;
}

/**
 * @brief Returns the event list for a scene.
 *
 * @param scene  the scene to query
 * @return the event list of the scene; may be NULL
 */
sb_event_list_t* sb_screenplay_scene_get_events(sb_screenplay_scene_t* scene)
{
    return scene->events;
}

/**
 * @brief Returns the duration of a scene in milliseconds.
 *
 * @param scene  the scene to query
 * @return the duration of the scene in milliseconds; \c UINT32_MAX if the duration is infinite
 */
uint32_t sb_screenplay_scene_get_duration_msec(
    const sb_screenplay_scene_t* scene)
{
    return scene->duration_msec;
}

/**
 * @brief Returns the duration of a scene in seconds.
 *
 * @param scene  the scene to query
 * @return the duration of the scene in seconds; \c INFINITY if the duration is infinite
 */
float sb_screenplay_scene_get_duration_sec(
    const sb_screenplay_scene_t* scene)
{
    if (scene->duration_msec == UINT32_MAX) {
        return INFINITY;
    } else {
        return scene->duration_msec / 1000.0f;
    }
}

/**
 * @brief Sets the trajectory for a scene.
 *
 * @param scene    the scene to modify
 * @param trajectory the trajectory to set; may be NULL
 */
void sb_screenplay_scene_set_trajectory(
    sb_screenplay_scene_t* scene, sb_trajectory_t* trajectory)
{
    SB_XINCREF(trajectory);
    SB_XDECREF(scene->trajectory);
    scene->trajectory = trajectory;
}

/**
 * @brief Sets the light program for a scene.
 *
 * @param scene    the scene to modify
 * @param light_program the light program to set; may be NULL
 */
void sb_screenplay_scene_set_light_program(
    sb_screenplay_scene_t* scene, sb_light_program_t* light_program)
{
    SB_XINCREF(light_program);
    SB_XDECREF(scene->light_program);
    scene->light_program = light_program;
}

/**
 * @brief Sets the yaw control object for a scene.
 *
 * @param scene    the scene to modify
 * @param yaw_control the yaw control object to set; may be NULL
 */
void sb_screenplay_scene_set_yaw_control(
    sb_screenplay_scene_t* scene, sb_yaw_control_t* yaw_control)
{
    SB_XINCREF(yaw_control);
    SB_XDECREF(scene->yaw_control);
    scene->yaw_control = yaw_control;
}

/**
 * @brief Sets the event list for a scene.
 *
 * @param scene  the scene to modify
 * @param events   the event list to set; may be NULL
 */
void sb_screenplay_scene_set_events(
    sb_screenplay_scene_t* scene, sb_event_list_t* events)
{
    SB_XINCREF(events);
    SB_XDECREF(scene->events);
    scene->events = events;
}

/**
 * @brief Sets the duration of a scene in milliseconds.
 *
 * @param scene       the scene to modify
 * @param duration_msec the duration to set in milliseconds; use \c UINT32_MAX for infinite duration
 * @return \c SB_SUCCESS if the duration was set successfully
 *         \c SB_EINVAL if the duration is invalid
 */
sb_error_t sb_screenplay_scene_set_duration_msec(
    sb_screenplay_scene_t* scene, uint32_t duration_msec)
{
    scene->duration_msec = duration_msec;
    return SB_SUCCESS;
}

/**
 * @brief Sets the duration of a scene in seconds.
 *
 * @param scene       the scene to modify
 * @param duration_sec  the duration to set in seconds; use \c INFINITY for infinite duration
 * @return \c SB_SUCCESS if the duration was set successfully
 *         \c SB_EINVAL if the duration is invalid or too large to fit in a \c uint32_t
 */
sb_error_t sb_screenplay_scene_set_duration_sec(
    sb_screenplay_scene_t* scene, float duration_sec)
{
    if (isinf(duration_sec) && duration_sec > 0) {
        scene->duration_msec = UINT32_MAX;
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

        scene->duration_msec = duration_msec_u;
    }
    return SB_SUCCESS;
}

/**
 * @brief Resets the screenplay scene to its default state.
 *
 * All associated objects of the screenplay scene will be cleared. The time axis
 * will be reset to its initial state with no segments and origin at 0 ms. The duration
 * will be set to infinite.
 *
 * @param scene  the screenplay scene to reset
 */
void sb_screenplay_scene_reset(sb_screenplay_scene_t* scene)
{
    sb_screenplay_scene_set_trajectory(scene, NULL);
    sb_screenplay_scene_set_light_program(scene, NULL);
    sb_screenplay_scene_set_yaw_control(scene, NULL);
    sb_screenplay_scene_set_events(scene, NULL);
    sb_screenplay_scene_set_duration_msec(scene, UINT32_MAX);
    sb_time_axis_clear(&scene->time_axis);
}

/**
 * @brief Updates a screenplay scene from a binary show file in memory.
 *
 * This function updates the trajectory, light program, yaw control and event list
 * of the given screenplay scene by parsing the provided binary show file data.
 * If any of these components are not present in the show file, the corresponding
 * component in the scene will be set to NULL. The trajectory is considered mandatory;
 * if it cannot be loaded, an error will be returned.
 *
 * The duration of the screenplay scene will be set to infinity. The time axis
 * will be reset. In other words, assume that `sb_screenplay_scene_reset()` is called
 * before loading the components from the show file.
 *
 * @param scene    the screenplay scene to update
 * @param show_data  pointer to the binary show file data in memory. May be NULL.
 *        The ownership of the data is not taken; the caller is responsible for
 *        managing the memory of the data and to keep it alive while the screenplay
 *        scene is using it.
 * @param length     length of the binary show file data in bytes
 */
sb_error_t sb_screenplay_scene_update_from_binary_file_in_memory(
    sb_screenplay_scene_t* scene, uint8_t* show_data, size_t length)
{
    sb_error_t retval;
    sb_trajectory_t* trajectory = NULL;
    sb_light_program_t* light_program = NULL;
    sb_yaw_control_t* yaw_control = NULL;
    sb_event_list_t* event_list = NULL;

    sb_screenplay_scene_reset(scene);
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

    sb_screenplay_scene_set_trajectory(scene, trajectory);

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
        sb_screenplay_scene_set_light_program(scene, light_program);
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
        sb_screenplay_scene_set_yaw_control(scene, yaw_control);
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
        sb_screenplay_scene_set_events(scene, event_list);
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
