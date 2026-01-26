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

#ifndef SKYBRUSH_SCREENPLAY_H
#define SKYBRUSH_SCREENPLAY_H

/**
 * @file screenplay.h
 * @brief Functions and structures to evaluate a complex list of \c libskybrush
 * trajectories, light programs and yaw controllers on a common timeline where time
 * may even pass at different speeds.
 */

#include <skybrush/decls.h>
#include <skybrush/events.h>
#include <skybrush/lights.h>
#include <skybrush/refcount.h>
#include <skybrush/time_axis.h>
#include <skybrush/trajectory.h>
#include <skybrush/yaw_control.h>

__BEGIN_DECLS

/**
 * @brief A single scene in a \c sb_screenplay_t.
 *
 * A scene owns a mandatory time axis that specifies how time flows during the
 * scene in relation to wall clock time. Furthermore, a scene may refer to a
 * trajectory, a light program, a yaw control track, and an event track.
 *
 * The duration of a scene is either infinite (by default) or finite (if a duration
 * in _wall clock time_ is specified).
 */
typedef struct sb_screenplay_scene_s {
    SB_REFCOUNTED;

    /** The duration of the scene, in milliseconds. \c UINT32_MAX means infinite. */
    uint32_t duration_msec;

    /** The time axis of the scene. */
    sb_time_axis_t time_axis;

    /**
     * Optional trajectory correspnding to the scene; \c NULL if no position or
     * velocity commands should be emitted while playing the scene.
     */
    sb_trajectory_t* trajectory;

    /**
     * Optional light program correspnding to the scene; \c NULL if no light commands
     * xshould be emitted while playing the scene.
     */
    sb_light_program_t* light_program;

    /**
     * Optional yaw control object corresponding to the scene; \c NULL if no yaw
     * commands should be emitted while playing the scene.
     */
    sb_yaw_control_t* yaw_control;

    /**
     * Optional event list corresponding to the scene; \c NULL if no events should be
     * emitted while playing the scene.
     */
    sb_event_list_t* events;
} sb_screenplay_scene_t;

sb_error_t sb_screenplay_scene_init(sb_screenplay_scene_t* scene);

uint32_t sb_screenplay_scene_get_duration_msec(
    const sb_screenplay_scene_t* scene);
float sb_screenplay_scene_get_duration_sec(
    const sb_screenplay_scene_t* scene);
sb_trajectory_t* sb_screenplay_scene_get_trajectory(
    sb_screenplay_scene_t* scene);
sb_light_program_t* sb_screenplay_scene_get_light_program(
    sb_screenplay_scene_t* scene);
sb_yaw_control_t* sb_screenplay_scene_get_yaw_control(
    sb_screenplay_scene_t* scene);
sb_event_list_t* sb_screenplay_scene_get_events(
    sb_screenplay_scene_t* scene);
sb_time_axis_t* sb_screenplay_scene_get_time_axis(sb_screenplay_scene_t* scene);

sb_error_t sb_screenplay_scene_set_duration_msec(
    sb_screenplay_scene_t* scene, uint32_t duration_msec);
sb_error_t sb_screenplay_scene_set_duration_sec(
    sb_screenplay_scene_t* scene, float duration_sec);
void sb_screenplay_scene_set_trajectory(
    sb_screenplay_scene_t* scene, sb_trajectory_t* trajectory);
void sb_screenplay_scene_set_light_program(
    sb_screenplay_scene_t* scene, sb_light_program_t* light_program);
void sb_screenplay_scene_set_yaw_control(
    sb_screenplay_scene_t* scene, sb_yaw_control_t* yaw_control);
void sb_screenplay_scene_set_events(
    sb_screenplay_scene_t* scene, sb_event_list_t* events);

void sb_screenplay_scene_reset(sb_screenplay_scene_t* scene);
sb_error_t sb_screenplay_scene_update_from_binary_file_in_memory(
    sb_screenplay_scene_t* scene, uint8_t* show_data, size_t length);

/* ************************************************************************* */

/**
 * @brief A screenplay is a sequence of scenes that define a complex performance
 * timeline for a \c libskybrush-powered device.
 *
 * Each scene may refer to a trajectory, a light program, a yaw control track,
 * and an event track. Furthermore, each scene has its own time axis that defines
 * how time flows during the scene in relation to wall clock time.
 *
 * A screenplay can be evaluated by a show controller (\ref sb_show_controller_t)
 * to obtain the control outputs at any given point in time.
 */
typedef struct sb_screenplay_s {
    sb_screenplay_scene_t* scenes; /**< The list of scenes */
    size_t num_scenes; /**< The current number of scenes in the list */
    size_t max_scenes; /**< The maximum number of scenes that the list can hold */
} sb_screenplay_t;

sb_error_t sb_screenplay_init(sb_screenplay_t* screenplay);
void sb_screenplay_destroy(sb_screenplay_t* screenplay);

size_t sb_screenplay_capacity(const sb_screenplay_t* screenplay);
sb_bool_t sb_screenplay_is_empty(const sb_screenplay_t* screenplay);
size_t sb_screenplay_size(const sb_screenplay_t* screenplay);

sb_screenplay_scene_t* sb_screenplay_get_scene_ptr(
    sb_screenplay_t* screenplay, size_t index);
sb_screenplay_scene_t* sb_screenplay_get_scene_ptr_at_time_msec(
    sb_screenplay_t* screenplay, uint32_t* time_msec, ssize_t* scene_index);

void sb_screenplay_clear(sb_screenplay_t* screenplay);
sb_error_t sb_screenplay_append_new_scene(
    sb_screenplay_t* screenplay, sb_screenplay_scene_t** out_scene);
sb_error_t sb_screenplay_remove_last_scene(sb_screenplay_t* screenplay);
sb_error_t sb_screenplay_update_from_binary_file_in_memory(
    sb_screenplay_t* screenplay, uint8_t* show_data, size_t length);

__END_DECLS

#endif
