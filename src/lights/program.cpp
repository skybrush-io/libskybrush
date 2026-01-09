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
#include <math.h>
#include <string.h>

#include <skybrush/formats/binary.h>
#include <skybrush/lights.h>
#include <skybrush/memory.h>

#include "bytecode_array.hpp"
#include "bytecode_player.h"
#include "skybrush/refcount.h"

static void sb_i_light_program_destroy(sb_light_program_t* program);
static sb_error_t sb_i_light_program_update_from_bytes(sb_light_program_t* program, uint8_t* buf, size_t nbytes, sb_bool_t owned);
static sb_error_t sb_i_light_program_update_from_parser(sb_light_program_t* program, sb_binary_file_parser_t* parser);

/**
 * \brief Allocates a new light program on the heap and initializes it.
 *
 * \return the new light program, or \c NULL if memory allocation failed
 */
sb_light_program_t* sb_light_program_new(void)
{
    sb_light_program_t* obj = sb_calloc(sb_light_program_t, 1);

    if (obj) {
        if (sb_light_program_init(obj)) {
            sb_free(obj);
        }
    }

    return obj;
}

/**
 * Initializes an already allocated light program.
 *
 * You must call this function on an uninitialized light program before using it.
 * \ref sb_light_program_new() takes care of the initialization for you if you
 * allocate the light program on the heap.
 *
 * \param ctrl  the light program to initialize
 * \return \c SB_SUCCESS if the light program was initialized successfully,
 *         \c SB_ENOMEM if memory allocation failed
 */
sb_error_t sb_light_program_init(sb_light_program_t* program)
{
    SB_CHECK(sb_buffer_init(&program->buffer, 0));
    SB_REF_INIT(program, sb_i_light_program_destroy);
    return SB_SUCCESS;
}

/* ************************************************************************** */

/**
 * Clears the light program object.
 */
void sb_light_program_clear(sb_light_program_t* program)
{
    sb_buffer_clear(&program->buffer);
}

/**
 * Updates a light program object from the contents of a memory buffer.
 *
 * \param program  the light program to update
 * \param buf   the buffer holding the encoded light program
 * \param size  the length of the buffer
 *
 * \return \c SB_SUCCESS if the object was updated successfully,
 *         \c SB_ENOENT if the memory buffer did not contain a light program
 */
sb_error_t sb_light_program_update_from_buffer(sb_light_program_t* program, uint8_t* buf, size_t size)
{
    return sb_i_light_program_update_from_bytes(program, buf, size, /* owned = */ 0);
}

/**
 * Updates a light program object from the contents of a memory buffer,
 * taking ownership.
 *
 * \param program  the light program to update
 * \param buf   the buffer holding the encoded light program
 * \param size  the length of the buffer
 *
 * \return \c SB_SUCCESS if the object was updated successfully,
 *         \c SB_ENOENT if the memory buffer did not contain a light program
 */
sb_error_t sb_light_program_update_from_bytes(sb_light_program_t* program, uint8_t* buf, size_t size)
{
    return sb_i_light_program_update_from_bytes(program, buf, size, /* owned = */ 1);
}

/**
 * Updates a light program object from the contents of a Skybrush file in
 * binary format.
 *
 * \param program  the light program to update
 * \param fd  handle to the low-level file object to update the object from
 *
 * \return \c SB_SUCCESS if the object was updated successfully,
 *         \c SB_ENOENT if the file did not contain a light program
 *         \c SB_EREAD for read errors
 */
sb_error_t sb_light_program_update_from_binary_file(sb_light_program_t* program, int fd)
{
    sb_binary_file_parser_t parser;
    sb_error_t retval;

    SB_CHECK(sb_binary_file_parser_init_from_file(&parser, fd));
    retval = sb_i_light_program_update_from_parser(program, &parser);
    sb_binary_file_parser_destroy(&parser);

    return retval;
}

/**
 * Updates a light program object from the contents of a Skybrush file in
 * binary format, already loaded into memory.
 *
 * \return \c SB_SUCCESS if the object was updated successfully,
 *         \c SB_ENOENT if the memory block did not contain a light program
 */
sb_error_t sb_light_program_update_from_binary_file_in_memory(
    sb_light_program_t* program, uint8_t* buf, size_t nbytes)
{
    sb_binary_file_parser_t parser;
    sb_error_t retval;

    SB_CHECK(sb_binary_file_parser_init_from_buffer(&parser, buf, nbytes));
    retval = sb_i_light_program_update_from_parser(program, &parser);
    sb_binary_file_parser_destroy(&parser);

    return retval;
}

/* ************************************************************************** */

static void sb_i_light_program_destroy(sb_light_program_t* program)
{
    sb_buffer_destroy(&program->buffer);
}

static sb_error_t sb_i_light_program_update_from_bytes(sb_light_program_t* program, uint8_t* buf, size_t nbytes, sb_bool_t owned)
{
    sb_buffer_t new_buffer;

    if (owned) {
        SB_CHECK(sb_buffer_init_from_bytes(&new_buffer, buf, nbytes));
    } else {
        sb_buffer_init_view(&new_buffer, buf, nbytes);
    }

    sb_buffer_destroy(&program->buffer);
    program->buffer = new_buffer;

    return SB_SUCCESS;
}

static sb_error_t sb_i_light_program_update_from_parser(
    sb_light_program_t* program, sb_binary_file_parser_t* parser)
{
    sb_error_t retval;
    uint8_t* buf;
    size_t size;
    sb_bool_t owned;

    SB_CHECK(sb_binary_file_find_first_block_by_type(parser, SB_BINARY_BLOCK_LIGHT_PROGRAM));
    SB_CHECK(sb_binary_file_read_current_block_ex(parser, &buf, &size, &owned));

    retval = sb_i_light_program_update_from_bytes(program, buf, size, owned);
    if (retval != SB_SUCCESS) {
        if (owned) {
            sb_free(buf);
        }
        return retval;
    }

    /* ownership of 'buf' taken by the light program if needed */

    return SB_SUCCESS;
}

/* ************************************************************************** */

#define PLAYER (static_cast<BytecodePlayer*>(player->player))
#define STORE (static_cast<BytecodeStore*>(player->store))

static void sb_i_light_player_set_store(sb_light_player_t* player, BytecodeStore* store);

/**
 * Initializes a \c sb_light_player_t structure.
 *
 * \param  player   the player object
 * \param  program  the light program that the player will play
 */
sb_error_t sb_light_player_init(sb_light_player_t* player, sb_light_program_t* program)
{
    BytecodePlayer* new_player;
    BytecodeStore* new_store;

    if (program == 0) {
        return SB_EINVAL;
    }

    memset(player, 0, sizeof(sb_light_player_t));

    new_player = new BytecodePlayer();
    if (new_player == 0) {
        return SB_ENOMEM; /* LCOV_EXCL_LINE */
    }

    new_store = new ArrayBytecodeStore(SB_BUFFER(program->buffer), sb_buffer_size(&program->buffer));
    if (new_store == 0) {
        delete new_player;
        return SB_ENOMEM; /* LCOV_EXCL_LINE */
    }

    player->program = program;
    SB_INCREF(player->program);

    player->player = new_player;

    sb_i_light_player_set_store(player, new_store);

    return SB_SUCCESS;
}

/**
 * Destroys a \c sb_light_player_t structure.
 *
 * \param  player  the player object
 */
void sb_light_player_destroy(sb_light_player_t* player)
{
    sb_i_light_player_set_store(player, 0);

    if (PLAYER) {
        delete PLAYER;
        player->player = 0;
    }

    SB_XDECREF(player->program);

    memset(player, 0, sizeof(sb_light_player_t));
}

static void sb_i_light_player_set_store(sb_light_player_t* player, BytecodeStore* store)
{
    BytecodeStore* oldStore = STORE;
    player->store = store;
    PLAYER->setBytecodeStore(store);

    if (oldStore) {
        delete oldStore;
    }

    player->next_timestamp = 0;
}

/**
 * Gets the color to be shown at the given timestamp.
 *
 * \param  player     the player object
 * \param  timestamp  the timestamp to seek to, in milliseconds
 */
sb_rgb_color_t sb_light_player_get_color_at(
    sb_light_player_t* player, unsigned long timestamp)
{
    sb_light_player_seek(player, timestamp, 0);
    return PLAYER->currentColor();
}

/**
 * Sets the current timestamp of the bytecode player to the given timestamp.
 *
 * \param  player  the player object
 * \param  timestamp  the timestamp to seek to, in milliseconds
 * \param  next_timestamp  pointer to an unsigned long where the next proposed
 *         timestamp will be returned from the player if not null
 * \return zero if the player has not reached the end of the bytecode, non-zero
 *         otherwise
 */
sb_bool_t sb_light_player_seek(
    sb_light_player_t* player, unsigned long timestamp,
    unsigned long* next_timestamp)
{
    sb_bool_t ended = PLAYER->seek(timestamp, &player->next_timestamp);
    if (next_timestamp) {
        *next_timestamp = player->next_timestamp;
    }
    return ended;
}

#undef PLAYER
#undef STORE
