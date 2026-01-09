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

#ifndef SKYBRUSH_LIGHT_PROGRAM_H
#define SKYBRUSH_LIGHT_PROGRAM_H

#include <stdlib.h>

#include <skybrush/basic_types.h>
#include <skybrush/buffer.h>
#include <skybrush/colors.h>
#include <skybrush/decls.h>
#include <skybrush/error.h>
#include <skybrush/refcount.h>

__BEGIN_DECLS

/**
 * @file lights.h
 * @brief Functions and structures to interpret and "play" a \c libskybrush light program
 */

/**
 * Structure that represents a \c libskybrush light program.
 */
typedef struct sb_light_program_s {
    SB_REFCOUNTED;
    sb_buffer_t buffer; /**< Buffer holding the light program */
} sb_light_program_t;

sb_light_program_t* sb_light_program_new(void);
sb_error_t sb_light_program_init(sb_light_program_t* program);

void sb_light_program_clear(sb_light_program_t* program);

sb_error_t sb_light_program_update_from_binary_file(sb_light_program_t* program, int fd);
sb_error_t sb_light_program_update_from_binary_file_in_memory(
    sb_light_program_t* program, uint8_t* buf, size_t length);
sb_error_t sb_light_program_update_from_buffer(
    sb_light_program_t* program, uint8_t* buf, size_t length);
sb_error_t sb_light_program_update_from_bytes(
    sb_light_program_t* program, uint8_t* buf, size_t size);

/**
 * Structure that represents a \c libskybrush light program player that the
 * calling code can "ask" what color the light program dictates at any given
 * timestamp.
 */
typedef struct sb_light_player_s {
    /**
     * Pointer to the light program that the player plays.
     */
    sb_light_program_t* program;

    /**
     * Pointer to an ArrayBytecodeStore C++ template class that points to the
     * buffer holding the light program being played. The pointer is untyped
     * because we don't want to expose a C++ class in the API.
     */
    void* store;

    /**
     * Pointer to a BytecodePlayer C++ class that the player struct wraps.
     * The pointer is untyped because we don't want to expose a C++ class
     * in the API.
     */
    void* player;

    /**
     * Variable to hold the next timestamp where something interesting might
     * happen in the bytecode.
     */
    unsigned long next_timestamp;
} sb_light_player_t;

sb_error_t sb_light_player_init(sb_light_player_t* player, sb_light_program_t* program);
void sb_light_player_destroy(sb_light_player_t* player);

sb_rgb_color_t sb_light_player_get_color_at(
    sb_light_player_t* player, unsigned long timestamp);
sb_bool_t sb_light_player_seek(
    sb_light_player_t* player, unsigned long timestamp,
    unsigned long* next_timestamp);

__END_DECLS

#endif
