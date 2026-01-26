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
#include <skybrush/control.h>
#include <skybrush/error.h>
#include <skybrush/memory.h>
#include <string.h>

/**
 * @brief Clears the control output structure.
 */
void sb_control_output_clear(sb_control_output_t* output)
{
    output->mask = SB_CONTROL_OUTPUT_NONE;
}

/**
 * @brief Clears the specified components from the control output.
 *
 * @param output      the control output to modify
 * @param components  the components to clear
 */
void sb_control_output_clear_components(sb_control_output_t* output,
    sb_control_component_mask_t components)
{
    output->mask &= ~components;
}

/**
 * @brief Checks whether the control output has any of the given components.
 *
 * @param output      the control output to query
 * @param components  the components to check for
 */
sb_bool_t sb_control_output_has_any_component_in(const sb_control_output_t* output,
    sb_control_component_mask_t components)
{
    return (output->mask & components) != 0;
}

/**
 * @brief Checks whether the control output has all of the given components.
 *
 * @param output      the control output to query
 * @param components  the components to check for
 */
sb_bool_t sb_control_output_has_all_components_in(const sb_control_output_t* output,
    sb_control_component_mask_t components)
{
    return (output->mask & components) == components;
}

/**
 * @brief Retrieves the position from the control output if it is set.
 *
 * @param output  the control output to query
 * @param out_position  if not \c NULL, will be set to the position if it is set
 * @return true if the position is set, false otherwise
 */
sb_bool_t sb_control_output_get_position_if_set(const sb_control_output_t* output,
    sb_vector3_t* out_position)
{
    if (sb_control_output_has_any_component_in(output, SB_CONTROL_OUTPUT_POSITION)) {
        if (out_position != NULL) {
            out_position->x = output->position.x;
            out_position->y = output->position.y;
            out_position->z = output->position.z;
        }
        return 1;
    } else {
        return 0;
    }
}

/**
 * @brief Retrieves the velocity from the control output if it is set.
 *
 * @param output  the control output to query
 * @param out_velocity  if not \c NULL, will be set to the velocity if it is set
 * @return true if the position is set, false otherwise
 */
sb_bool_t sb_control_output_get_velocity_if_set(const sb_control_output_t* output,
    sb_vector3_t* out_velocity)
{
    if (sb_control_output_has_any_component_in(output, SB_CONTROL_OUTPUT_VELOCITY)) {
        if (out_velocity != NULL) {
            out_velocity->x = output->velocity.x;
            out_velocity->y = output->velocity.y;
            out_velocity->z = output->velocity.z;
        }
        return 1;
    } else {
        return 0;
    }
}

/**
 * @brief Retrieves the color from the control output if it is set.
 *
 * @param output  the control output to query
 * @param out_color  if not \c NULL, will be set to the color if it is set
 * @return true if the color is set, false otherwise
 */
sb_bool_t sb_control_output_get_color_if_set(const sb_control_output_t* output,
    sb_rgb_color_t* out_color)
{
    if (sb_control_output_has_any_component_in(output, SB_CONTROL_OUTPUT_LIGHTS)) {
        if (out_color != NULL) {
            *out_color = output->color;
        }
        return 1;
    } else {
        return 0;
    }
}

/**
 * @brief Retrieves the yaw from the control output if it is set.
 *
 * @param output  the control output to query
 * @param out_yaw  if not \c NULL, will be set to the yaw if it is set
 * @return true if the yaw is set, false otherwise
 */
sb_bool_t sb_control_output_get_yaw_if_set(const sb_control_output_t* output,
    float* out_yaw)
{
    if (sb_control_output_has_any_component_in(output, SB_CONTROL_OUTPUT_YAW)) {
        if (out_yaw != NULL) {
            *out_yaw = output->position.yaw;
        }
        return 1;
    } else {
        return 0;
    }
}

/**
 * @brief Retrieves the yaw rate from the control output if it is set.
 *
 * @param output  the control output to query
 * @param out_yaw_rate  if not \c NULL, will be set to the yaw rate if it is set
 * @return true if the yaw rate is set, false otherwise
 */
sb_bool_t sb_control_output_get_yaw_rate_if_set(const sb_control_output_t* output,
    float* out_yaw_rate)
{
    if (sb_control_output_has_any_component_in(output, SB_CONTROL_OUTPUT_YAW_RATE)) {
        if (out_yaw_rate != NULL) {
            *out_yaw_rate = output->velocity.yaw;
        }
        return 1;
    } else {
        return 0;
    }
}

/**
 * @brief Sets the position in the control output.
 *
 * @param output    the control output to modify
 * @param position  the position to set
 */
void sb_control_output_set_position(sb_control_output_t* output, sb_vector3_t position)
{
    output->position.x = position.x;
    output->position.y = position.y;
    output->position.z = position.z;
    output->mask |= SB_CONTROL_OUTPUT_POSITION;
}

/**
 * @brief Sets the velocity in the control output.
 *
 * @param output    the control output to modify
 * @param velocity  the velocity to set
 */
void sb_control_output_set_velocity(sb_control_output_t* output, sb_vector3_t velocity)
{
    output->velocity.x = velocity.x;
    output->velocity.y = velocity.y;
    output->velocity.z = velocity.z;
    output->mask |= SB_CONTROL_OUTPUT_VELOCITY;
}

/**
 * @brief Sets the color in the control output.
 *
 * @param output  the control output to modify
 * @param color   the color to set
 */
void sb_control_output_set_color(sb_control_output_t* output, sb_rgb_color_t color)
{
    output->color = color;
    output->mask |= SB_CONTROL_OUTPUT_LIGHTS;
}

/**
 * @brief Sets the yaw in the control output.
 *
 * @param output  the control output to modify
 * @param yaw     the yaw to set
 */
void sb_control_output_set_yaw(sb_control_output_t* output, float yaw)
{
    output->position.yaw = yaw;
    output->mask |= SB_CONTROL_OUTPUT_YAW;
}

/**
 * @brief Sets the yaw rate in the control output.
 *
 * @param output    the control output to modify
 * @param yaw_rate  the yaw rate to set
 */
void sb_control_output_set_yaw_rate(sb_control_output_t* output, float yaw_rate)
{
    output->velocity.yaw = yaw_rate;
    output->mask |= SB_CONTROL_OUTPUT_YAW_RATE;
}

/* ************************************************************************* */

void sb_control_output_time_invalidate(sb_control_output_time_t* time)
{
    memset(time, 0, sizeof(sb_control_output_time_t));
    time->time_msec = UINT32_MAX;
    time->time_in_scene_msec = UINT32_MAX;
}

/* ************************************************************************* */

static sb_error_t sb_i_show_controller_set_current_scene(
    sb_show_controller_t* ctrl, sb_screenplay_scene_t* scene);

/**
 * @brief Initializes a show controller with the given screenplay.
 *
 * The default control output of the show controller is set to zero velocity and
 * zero yaw rate.
 *
 * @param ctrl        pointer to the show controller to initialize
 * @param screenplay  pointer to the screenplay to use
 */
sb_error_t sb_show_controller_init(sb_show_controller_t* ctrl, sb_screenplay_t* screenplay)
{
    sb_vector3_t zero = { 0.0f, 0.0f, 0.0f };

    memset(ctrl, 0, sizeof(sb_show_controller_t));

    ctrl->screenplay = screenplay;

    sb_control_output_clear(&ctrl->default_output);
    sb_control_output_set_velocity(&ctrl->default_output, zero);
    sb_control_output_set_yaw_rate(&ctrl->default_output, 0.0f);

    sb_show_controller_invalidate_output(ctrl);

    return SB_SUCCESS;
}

/**
 * @brief Destroys a show controller and releases all associated resources.
 *
 * @param ctrl  pointer to the show controller to destroy
 */
void sb_show_controller_destroy(sb_show_controller_t* ctrl)
{
    sb_i_show_controller_set_current_scene(ctrl, NULL);
    sb_control_output_clear(&ctrl->output);
    sb_control_output_time_invalidate(&ctrl->output_time);
}

/**
 * @brief Returns the current scene of the show controller.
 *
 * @param ctrl  pointer to the show controller to query
 * @return pointer to the current scene
 */
sb_screenplay_scene_t* sb_show_controller_get_current_scene(const sb_show_controller_t* ctrl)
{
    return ctrl->current_scene;
}

/**
 * @brief Returns the current control output of the show controller.
 *
 * @param ctrl  pointer to the show controller to query
 * @return pointer to the current control output
 */
const sb_control_output_t* sb_show_controller_get_current_output(const sb_show_controller_t* ctrl)
{
    return &ctrl->output;
}

/**
 * @brief Returns the time that the current control output of the show controller belongs to.
 *
 * @param controller  pointer to the show controller to query
 */
sb_control_output_time_t sb_show_controller_get_current_output_time(const sb_show_controller_t* controller)
{
    return controller->output_time;
}

/**
 * @brief Checks whether the current control output of the show controller is valid.
 *
 * @param controller  pointer to the show controller to query
 * @return \c SB_TRUE if the current control output is valid, \c SB_FALSE otherwise
 */
sb_bool_t sb_show_controller_is_output_valid(const sb_show_controller_t* controller)
{
    return controller->output_time.time_msec != UINT32_MAX;
}

/**
 * @brief Updates the control output to the desired output at a specific time in the screenplay.
 *
 * When the specified time is out of bounds, we return a control output that commands
 * zero velocity and zero yaw rate, with no position, yaw, or light commands.
 *
 * @param ctrl       pointer to the show controller to update
 * @param time_msec  the time in milliseconds from the start of the screenplay
 * @return \c SB_SUCCESS if the control output was updated successfully,
 *         \c SB_EINVAL if the time is out of bounds
 */
sb_error_t sb_show_controller_update_time_msec(sb_show_controller_t* ctrl, uint32_t time_msec)
{
    sb_vector3_with_yaw_t vec_with_yaw;
    sb_vector3_t vec;
    sb_rgb_color_t color;
    sb_screenplay_scene_t* scene;
    sb_control_output_t* out = &ctrl->output;
    ssize_t scene_index;
    uint32_t time_msec_orig;
    float warped_time_sec;
    float warped_rate;
    float yaw;

    if (ctrl->output_time.time_msec != UINT32_MAX && time_msec == ctrl->output_time.time_msec) {
        /* Output is already up to date */
        return SB_SUCCESS;
    }

    sb_control_output_clear(out);

    time_msec_orig = time_msec;
    scene = ctrl->screenplay ? sb_screenplay_get_scene_ptr_at_time_msec(ctrl->screenplay, &time_msec, &scene_index) : NULL;
    sb_i_show_controller_set_current_scene(ctrl, scene);

    /* time_in_scene_msec is now up-to-date */

    /* invalidate the cached output_time_msec and warped_output_time_sec fields in
     * case we bail out with an error below */
    sb_show_controller_invalidate_output(ctrl);

    if (scene == NULL) {
        /* Time is out of bounds */
        warped_time_sec = 0.0f;
        *out = ctrl->default_output;
    } else {
        /* Update control output from trajectory if available */
        warped_time_sec = sb_time_axis_map_ex(&scene->time_axis, time_msec_orig, &warped_rate);

        sb_control_output_clear(out);

        if (ctrl->trajectory_player) {
            SB_CHECK(sb_trajectory_player_get_position_at(
                ctrl->trajectory_player, warped_time_sec, &vec_with_yaw));
            vec.x = vec_with_yaw.x;
            vec.y = vec_with_yaw.y;
            vec.z = vec_with_yaw.z;
            sb_control_output_set_position(out, vec);

            SB_CHECK(sb_trajectory_player_get_velocity_at(
                ctrl->trajectory_player, warped_time_sec, &vec_with_yaw));
            vec.x = vec_with_yaw.x * warped_rate;
            vec.y = vec_with_yaw.y * warped_rate;
            vec.z = vec_with_yaw.z * warped_rate;
            sb_control_output_set_velocity(out, vec);
        }

        if (ctrl->light_player) {
            /* clang-format off */
            uint32_t warped_time_msec = (uint32_t)(
                (warped_time_sec < 0 || !isfinite(warped_time_sec) ? 0 :
                warped_time_sec > 86400 ? 86400 :
                warped_time_sec) * 1000.0f
            );
            /* clang-format on */
            color = sb_light_player_get_color_at(ctrl->light_player, warped_time_msec);
            sb_control_output_set_color(out, color);
        }

        if (ctrl->yaw_player) {
            SB_CHECK(sb_yaw_player_get_yaw_at(
                ctrl->yaw_player, warped_time_sec, &yaw));
            sb_control_output_set_yaw(out, yaw);

            SB_CHECK(sb_yaw_player_get_yaw_rate_at(
                ctrl->yaw_player, warped_time_sec, &yaw));
            sb_control_output_set_yaw_rate(out, yaw * warped_rate);
        }
    }

    /* Output calculated successfully; we can now update the timestamp */
    ctrl->output_time.time_msec = time_msec;
    ctrl->output_time.scene = scene_index;
    ctrl->output_time.time_in_scene_msec = time_msec_orig;
    ctrl->output_time.warped_time_in_scene_sec = warped_time_sec;

    return SB_SUCCESS;
}

/**
 * @brief Returns the next event from the show up to and including the current time.
 *
 * This function must be called in a loop until it returns \c NULL to retrieve all events
 * that are due at the current time.
 *
 * @param ctrl  pointer to the show controller to query
 * @return pointer to the next event, or \c NULL if there are no more events
 */
const sb_event_t* sb_show_controller_get_next_event(sb_show_controller_t* ctrl)
{
    if (ctrl->event_list_player) {
        return sb_event_list_player_get_next_event_not_later_than(
            ctrl->event_list_player, ctrl->output_time.warped_time_in_scene_sec);
    } else {
        return NULL;
    }
}

/**
 * @brief Invalidates the current control output of the show controller.
 *
 * This function must be called whenever the screenplay is modified in a way that may
 * potentially invalidate the current output, e.g., when scenes are added, removed,
 * or modified.
 *
 * @param controller  pointer to the show controller to modify
 */
void sb_show_controller_invalidate_output(sb_show_controller_t* ctrl)
{
    ctrl->output = ctrl->default_output;
    sb_control_output_time_invalidate(&ctrl->output_time);
}

static sb_error_t sb_i_show_controller_set_current_scene(
    sb_show_controller_t* ctrl, sb_screenplay_scene_t* scene)
{
    if (scene == ctrl->current_scene) {
        return SB_SUCCESS;
    }

    if (ctrl->trajectory_player) {
        sb_trajectory_player_destroy(ctrl->trajectory_player);
        sb_free(ctrl->trajectory_player);
    }

    if (ctrl->light_player) {
        sb_light_player_destroy(ctrl->light_player);
        sb_free(ctrl->light_player);
    }

    if (ctrl->yaw_player) {
        sb_yaw_player_destroy(ctrl->yaw_player);
        sb_free(ctrl->yaw_player);
    }

    if (ctrl->event_list_player) {
        sb_event_list_player_destroy(ctrl->event_list_player);
        sb_free(ctrl->event_list_player);
    }

    ctrl->current_scene = scene;

    if (scene) {
        if (scene->trajectory) {
            ctrl->trajectory_player = sb_calloc(sb_trajectory_player_t, 1);
            if (ctrl->trajectory_player == NULL) {
                return SB_ENOMEM; /* LCOV_EXCL_LINE */
            }
            SB_CHECK(sb_trajectory_player_init(ctrl->trajectory_player, scene->trajectory));
        }

        if (scene->light_program) {
            ctrl->light_player = sb_calloc(sb_light_player_t, 1);
            if (ctrl->light_player == NULL) {
                return SB_ENOMEM; /* LCOV_EXCL_LINE */
            }
            SB_CHECK(sb_light_player_init(ctrl->light_player, scene->light_program));
        }

        if (scene->yaw_control) {
            ctrl->yaw_player = sb_calloc(sb_yaw_player_t, 1);
            if (ctrl->yaw_player == NULL) {
                return SB_ENOMEM; /* LCOV_EXCL_LINE */
            }
            SB_CHECK(sb_yaw_player_init(ctrl->yaw_player, scene->yaw_control));
        }

        if (scene->events) {
            ctrl->event_list_player = sb_calloc(sb_event_list_player_t, 1);
            if (ctrl->event_list_player == NULL) {
                return SB_ENOMEM; /* LCOV_EXCL_LINE */
            }
            SB_CHECK(sb_event_list_player_init(ctrl->event_list_player, scene->events));
        }
    }

    return SB_SUCCESS;
}
