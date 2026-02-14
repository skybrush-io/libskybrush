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
#include <limits.h>
#include <math.h>
#include <skybrush/refcount.h>
#include <skybrush/screenplay.h>

static sb_error_t sb_i_screenplay_ensure_has_free_space(sb_screenplay_t* screenplay);

/**
 * @brief Initializes a screenplay structure.
 *
 * @param screenplay  the screenplay to initialize
 * @return \c SB_SUCCESS if the screenplay was initialized successfully,
 *       \c SB_ENOMEM if a memory allocation failed.
 */
sb_error_t sb_screenplay_init(sb_screenplay_t* screenplay)
{
    const int initial_capacity = 4;

    screenplay->scenes = sb_calloc(sb_screenplay_scene_t*, initial_capacity);
    if (screenplay->scenes == NULL) {
        return SB_ENOMEM; /* LCOV_EXCL_LINE */
    }

    screenplay->num_scenes = 0;
    screenplay->max_scenes = initial_capacity;

    screenplay->rth_plan = NULL;

    return SB_SUCCESS;
}

/**
 * @brief Destroys a screenplay structure, freeing all associated resources.
 *
 * @param screenplay  the screenplay to destroy
 */
void sb_screenplay_destroy(sb_screenplay_t* screenplay)
{
    sb_screenplay_clear(screenplay);

    if (screenplay->scenes != NULL) {
        sb_free(screenplay->scenes);
    }

    screenplay->num_scenes = 0;
    screenplay->max_scenes = 0;
}

/**
 * @brief Returns the number of scenes that the screenplay can hold without reallocating memory.
 *
 * @param screenplay  the screenplay to query
 * @return the capacity of the screenplay
 */
size_t sb_screenplay_capacity(const sb_screenplay_t* screenplay)
{
    return screenplay->max_scenes;
}

/**
 * @brief Returns the number of scenes in the screenplay.
 *
 * @param screenplay  the screenplay to query
 * @return the number of scenes in the screenplay
 */
size_t sb_screenplay_size(const sb_screenplay_t* screenplay)
{
    return screenplay->num_scenes;
}

/**
 * @brief Checks whether the screenplay is empty (has no scenes).
 *
 * @param screenplay  the screenplay to query
 * @return \c true if the screenplay is empty, \c false otherwise
 */
sb_bool_t sb_screenplay_is_empty(const sb_screenplay_t* screenplay)
{
    return screenplay->num_scenes == 0;
}

/**
 * @brief Checks whether the screenplay contains the given scene.
 *
 * @param screenplay  the screenplay to query
 * @param scene   the scene to check for
 * @return \c true if the screenplay contains the scene, \c false otherwise
 */
sb_bool_t sb_screenplay_contains_scene(const sb_screenplay_t* screenplay, const sb_screenplay_scene_t* scene)
{
    if (scene == NULL) {
        return 0;
    }

    for (size_t i = 0; i < screenplay->num_scenes; i++) {
        if (screenplay->scenes[i] == scene) {
            return 1;
        }
    }
    return 0;
}

/**
 * @brief Removes all scenes from the screenplay and clears the RTH plan (if any).
 *
 * @param screenplay  the screenplay to clear
 */
void sb_screenplay_clear(sb_screenplay_t* screenplay)
{
    while (!sb_screenplay_is_empty(screenplay)) {
        sb_screenplay_remove_last_scene(screenplay); /* will succeed */
    }

    sb_screenplay_set_rth_plan(screenplay, NULL);
}

/**
 * @brief Returns a pointer to the RTH plan associated with the screenplay.
 *
 * You can modify the RTH plan via the provided pointer.
 *
 * @param screenplay  the screenplay to query
 * @return a pointer to the RTH plan, or \c NULL if no RTH plan exists
 */
sb_rth_plan_t* sb_screenplay_get_rth_plan(sb_screenplay_t* screenplay)
{
    return screenplay->rth_plan;
}

/**
 * @brief Returns a pointer to the scene at the given index in the screenplay.
 *
 * You can modify the scene via the provided pointer.
 *
 * @param screenplay  the screenplay to query
 * @param index   the index of the scene to return
 * @return a pointer to the scene at the given index, or \c NULL if the index
 *         is out of bounds
 */
sb_screenplay_scene_t* sb_screenplay_get_scene_ptr(
    sb_screenplay_t* screenplay, size_t index)
{
    return (index < screenplay->num_scenes)
        ? screenplay->scenes[index]
        : NULL;
}

/**
 * @brief Returns a pointer to the scene that is active at the given time.
 *
 * The time is specified in milliseconds from the start of the screenplay.
 * If no scene is active at the given time, \c NULL is returned.
 *
 * @param screenplay  the screenplay to query
 * @param time_msec   pointer to a variable specifying the time in milliseconds from the
 *        start of the screenplay. It will be updated to contain the time offset
 *        within the returned scene. Its value will be zero upon returning when the
 *        returned scene is \c NULL.
 * @param scene_index  optional pointer to a variable that will be set to the index of the
 *        returned scene within the screenplay. Can be \c NULL if the index is not needed.
 *        Will be set to -1 if no scene is active at the given time.
 * @return a pointer to the scene that is active at the given time, or \c NULL
 *         if no scene is active at that time
 */
sb_screenplay_scene_t* sb_screenplay_get_scene_ptr_at_time_msec(
    sb_screenplay_t* screenplay, uint32_t* time_msec, ssize_t* scene_index)
{
    for (size_t i = 0; i < screenplay->num_scenes; i++) {
        sb_screenplay_scene_t* scene = screenplay->scenes[i];
        uint32_t scene_duration_msec = sb_screenplay_scene_get_duration_msec(scene);

        if (scene_duration_msec == UINT32_MAX) {
            /* Infinite duration scene -> always active */
            if (scene_index) {
                *scene_index = i;
            }
            return scene;
        }

        if (*time_msec < scene_duration_msec) {
            /* Current scene found */
            if (scene_index) {
                /* We should use SSIZE_MAX here but it is not standard and therefore
                 * it does not exist on some platforms. Use UINT16_MAX as a practical
                 * limit for the number of scenes instead; SSIZE_MAX is supposed to be
                 * larger than that.
                 */
                *scene_index = i < UINT16_MAX ? ((ssize_t)i) : -1;
            }
            return scene;
        }

        *time_msec -= scene_duration_msec;
    }

    *time_msec = 0;
    if (scene_index) {
        *scene_index = -1;
    }

    return NULL;
}

/**
 * @brief Appends a new scene to the end of the screenplay.
 *
 * The new scene is initialized with default values (infinite duration, no trajectory,
 * no light program, no yaw control, no events).
 *
 * @param screenplay  the screenplay to append the scene to
 * @param out_scene  if not \c NULL, will be set to point to the newly added scene.
 *        The returned value is a borrowed reference.
 * @return \c SB_SUCCESS if the scene was appended successfully,
 *         \c SB_ENOMEM if a memory allocation failed
 */
sb_error_t sb_screenplay_append_new_scene(sb_screenplay_t* screenplay, sb_screenplay_scene_t** out_scene)
{
    sb_screenplay_scene_t* scene;

    SB_CHECK(sb_i_screenplay_ensure_has_free_space(screenplay));

    scene = sb_screenplay_scene_new();
    if (scene == 0) {
        return SB_ENOMEM; /* LCOV_EXCL_LINE */
    }

    screenplay->scenes[screenplay->num_scenes++] = scene;

    if (out_scene != NULL) {
        *out_scene = scene;
    }

    return SB_SUCCESS;
}

/**
 * @brief Clears the contents of a scene, releasing all associated resources.
 *
 * The time axis of the scene will be left intact.
 *
 * @param scene  the scene to clear
 */
void sb_screenplay_scene_clear_contents(sb_screenplay_scene_t* scene)
{
    sb_screenplay_scene_update_contents_from(scene, NULL);
}

/**
 * @brief Removes the last scene from the screenplay.
 *
 * @param screenplay  the screenplay to remove the scene from
 * @return \c SB_SUCCESS if the scene was removed successfully,
 *         \c SB_EEMPTY if there are no scenes to remove
 */
sb_error_t sb_screenplay_remove_last_scene(sb_screenplay_t* screenplay)
{
    if (screenplay->num_scenes == 0) {
        return SB_EEMPTY;
    }

    screenplay->num_scenes--;
    SB_XDECREF(screenplay->scenes[screenplay->num_scenes]);
    screenplay->scenes[screenplay->num_scenes] = 0;

    return SB_SUCCESS;
}

/**
 * @brief Updates the \em contents of a scene from another scene.
 *
 * This function copies all contents from the source scene to the destination scene,
 * except the reference counts and the fields related to the duration or the time axis.
 *
 * @param scene  the destination scene to update
 * @param src    the source scene to copy from. May be \c NULL to indicate an empty
 *        scene with no contents.
 */
void sb_screenplay_scene_update_contents_from(
    sb_screenplay_scene_t* scene, sb_screenplay_scene_t* src)
{
    if (src) {
        sb_screenplay_scene_set_trajectory(scene, sb_screenplay_scene_get_trajectory(src));
        sb_screenplay_scene_set_light_program(scene, sb_screenplay_scene_get_light_program(src));
        sb_screenplay_scene_set_yaw_control(scene, sb_screenplay_scene_get_yaw_control(src));
        sb_screenplay_scene_set_events(scene, sb_screenplay_scene_get_events(src));
    } else {
        sb_screenplay_scene_set_trajectory(scene, NULL);
        sb_screenplay_scene_set_light_program(scene, NULL);
        sb_screenplay_scene_set_yaw_control(scene, NULL);
        sb_screenplay_scene_set_events(scene, NULL);
    }
}

/**
 * @brief Returns the number of seconds remaining from the trajectory at the end of the
 * \em time axis of the scene, in warped time.
 *
 * The function returns \em warped time so it takes into account the different rates
 * of the time axis segmnts as well as the time axis origin.
 *
 * Returns zero if no trajectory is associated to the scene. Also returns zero if the
 * time axis of the scene is already longer than the trajectory.
 *
 * Note that the \em duration of the scene is ignored; we always inspect the end of the
 * time axis.
 *
 * @param scene  the scene to query
 */
float sb_screenplay_scene_get_warped_time_remaining_from_trajectory_at_end_of_time_axis(
    sb_screenplay_scene_t* scene)
{
    sb_trajectory_t* trajectory = sb_screenplay_scene_get_trajectory(scene);
    sb_time_axis_t* time_axis = sb_screenplay_scene_get_time_axis(scene);

    if (trajectory && time_axis) {
        float warped_duration_of_axis = sb_time_axis_get_total_warped_duration_sec(time_axis);
        if (!isfinite(warped_duration_of_axis)) {
            /* Time axis has infinite warped duration -> trajectory will definitely be
             * played to the end
             */
            return 0.0f;
        }

        float result = sb_trajectory_get_total_duration_sec(trajectory) - warped_duration_of_axis;
        float origin = sb_time_axis_get_origin_sec(time_axis);
        if (origin < 0) {
            // scene starts _later_ than 00:00 on the show clock so we need to take into
            // account that we are not playing the trajectory from the very beginning
            result += origin;
        }
        return result > 0 ? result : 0.0f;
    } else {
        return 0.0f;
    }
}

/**
 * @brief Updates the screenplay from binary show file data in memory.
 *
 * This function clears the existing scenes in the screenplay and creates a new
 * scene based on the provided binary show file data. If the show data is NULL or has
 * zero length, the screenplay will be cleared and no new scene will be added.
 */
sb_error_t sb_screenplay_update_from_binary_file_in_memory(sb_screenplay_t* screenplay, uint8_t* show_data, size_t length)
{
    sb_rth_plan_t* rth_plan = NULL;
    sb_screenplay_scene_t* scene = NULL;
    sb_error_t retval = SB_SUCCESS;

    rth_plan = sb_rth_plan_new();
    if (rth_plan == NULL) {
        return SB_ENOMEM; /* LCOV_EXCL_LINE */
    }

    sb_screenplay_clear(screenplay);

    if (show_data && length > 0) {
        retval = sb_screenplay_append_new_scene(screenplay, &scene);
        if (retval != SB_SUCCESS) {
            goto exit;
        }

        retval = sb_screenplay_scene_update_from_binary_file_in_memory(scene, show_data, length);
        if (retval != SB_SUCCESS) {
            goto exit;
        }

        retval = sb_rth_plan_update_from_binary_file_in_memory(rth_plan, show_data, length);
        if (retval == SB_ENOENT) {
            /* No RTH plan in the show data */
            SB_XDECREF(rth_plan);
            retval = SB_SUCCESS;
        } else if (retval != SB_SUCCESS) {
            /* Some other error occurred */
            goto exit;
        }

        /* RTH plan successfully updated */
        sb_screenplay_set_rth_plan(screenplay, rth_plan);
    }

exit:
    if (retval != SB_SUCCESS) {
        sb_screenplay_clear(screenplay);
    }

    SB_XDECREF(rth_plan);
    return retval;
}

/* ************************************************************************** */

/**
 * @brief Ensures that the screenplay has enough free space to store a new
 * scene.
 *
 * If the screenplay does not have enough free space, it will be resized to
 * accommodate the new scene.
 *
 * @param screenplay  the screenplay to check
 * @return \c SB_SUCCESS if there is enough free space,
 *         \c SB_ENOMEM if memory allocation failed
 */
static sb_error_t sb_i_screenplay_ensure_has_free_space(sb_screenplay_t* screenplay)
{
    size_t free_space = screenplay->max_scenes - screenplay->num_scenes;

    if (free_space == 0) {
        size_t new_capacity = screenplay->max_scenes * 2;
        sb_screenplay_scene_t** new_scenes = sb_realloc(screenplay->scenes, sb_screenplay_scene_t*, new_capacity);
        if (new_scenes == 0) {
            return SB_ENOMEM; /* LCOV_EXCL_LINE */
        }

        screenplay->scenes = new_scenes;
        screenplay->max_scenes = new_capacity;
    }

    return SB_SUCCESS;
}

/**
 * @brief Sets the RTH plan associated with the screenplay.
 *
 * @param screenplay  the screenplay to update
 * @param rth_plan    the RTH plan to associate with the screenplay; may be \c NULL
 */
void sb_screenplay_set_rth_plan(sb_screenplay_t* screenplay, sb_rth_plan_t* rth_plan)
{
    SB_XINCREF(rth_plan);
    SB_XDECREF(screenplay->rth_plan);
    screenplay->rth_plan = rth_plan;
}
