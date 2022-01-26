/*
 * This file is part of libskybrush.
 *
 * Copyright 2020-2022 by libskybrush authors. See AUTHORS.
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
#include <float.h>
#include <math.h>
#include <string.h>

#include <skybrush/formats/binary.h>
#include <skybrush/lights.h>
#include <skybrush/memory.h>

#include "bytecode_array.hpp"
#include "bytecode_player.h"

sb_error_t sb_i_light_program_init_from_parser(sb_light_program_t* program, sb_binary_file_parser_t* parser);

/**
 * Instructs the light program object to take ownership of its inner memory buffer.
 */
static void sb_i_light_program_take_ownership(sb_light_program_t* program);

void sb_light_program_destroy(sb_light_program_t* program)
{
    sb_light_program_clear(program);
}

void sb_light_program_clear(sb_light_program_t* program)
{
    if (program->owner) {
        sb_free(program->buffer);
    }

    program->buffer = 0;
    program->buffer_length = 0;
    program->owner = 0;
}

sb_error_t sb_light_program_init_from_binary_file(sb_light_program_t* program, int fd)
{
    sb_binary_file_parser_t parser;
    sb_error_t retval;

    SB_CHECK(sb_binary_file_parser_init_from_file(&parser, fd));
    retval = sb_i_light_program_init_from_parser(program, &parser);
    sb_binary_file_parser_destroy(&parser);

    return retval;
}

sb_error_t sb_light_program_init_from_binary_file_in_memory(
    sb_light_program_t* program, uint8_t* buf, size_t nbytes)
{
    sb_binary_file_parser_t parser;
    sb_error_t retval;

    SB_CHECK(sb_binary_file_parser_init_from_buffer(&parser, buf, nbytes));
    retval = sb_i_light_program_init_from_parser(program, &parser);
    sb_binary_file_parser_destroy(&parser);

    return retval;
}

sb_error_t sb_i_light_program_init_from_parser(
    sb_light_program_t* program, sb_binary_file_parser_t* parser)
{
    sb_error_t retval;
    sb_binary_block_t block;
    uint8_t* buf;

    SB_CHECK(sb_binary_file_find_first_block_by_type(parser, SB_BINARY_BLOCK_LIGHT_PROGRAM));

    block = sb_binary_file_get_current_block(parser);

    buf = sb_calloc(uint8_t, block.length);
    if (buf == 0) {
        return SB_ENOMEM;
    }

    retval = sb_binary_file_read_current_block(parser, buf);
    if (retval != SB_SUCCESS) {
        sb_free(buf);
        return retval;
    }

    retval = sb_light_program_init_from_buffer(program, buf, block.length);
    if (retval != SB_SUCCESS) {
        sb_free(buf);
        return retval;
    }

    sb_i_light_program_take_ownership(program);

    return SB_SUCCESS;
}

sb_error_t sb_light_program_init_from_buffer(sb_light_program_t* program, uint8_t* buf, size_t size)
{
    program->buffer = buf;
    program->buffer_length = size;
    program->owner = 0;

    return SB_SUCCESS;
}

sb_error_t sb_light_program_init_empty(sb_light_program_t* program)
{
    return sb_light_program_init_from_buffer(program, 0, 0);
}

/* ************************************************************************** */

static void sb_i_light_program_take_ownership(sb_light_program_t* program)
{
    program->owner = 1;
}

/* ************************************************************************** */

#define PLAYER (static_cast<BytecodePlayer*>(player->player))
#define STORE (static_cast<BytecodeStore*>(player->store))

static void sb_i_light_player_set_store(sb_light_player_t* player, BytecodeStore* store);

sb_error_t sb_light_player_init(sb_light_player_t* player, const sb_light_program_t* program)
{
    BytecodePlayer* new_player;
    BytecodeStore* new_store;

    if (program == 0) {
        return SB_EINVAL;
    }

    memset(player, 0, sizeof(sb_light_player_t));

    new_player = new BytecodePlayer();
    if (new_player == 0) {
        return SB_ENOMEM;
    }

    new_store = new ArrayBytecodeStore(program->buffer, program->buffer_length);
    if (new_store == 0) {
        delete new_player;
        return SB_ENOMEM;
    }

    player->program = program;
    player->player = new_player;

    sb_i_light_player_set_store(player, new_store);

    return SB_SUCCESS;
}

void sb_light_player_destroy(sb_light_player_t* player)
{
    sb_i_light_player_set_store(player, 0);

    if (PLAYER) {
        delete PLAYER;
        player->player = 0;
    }

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

sb_rgb_color_t sb_light_player_get_color_at(
    sb_light_player_t* player, unsigned long timestamp)
{
    sb_light_player_seek(player, timestamp, 0);
    return PLAYER->currentColor();
}

uint8_t sb_light_player_get_pyro_channels_at(
    sb_light_player_t* player, unsigned long timestamp)
{
    sb_light_player_seek(player, timestamp, 0);
    return PLAYER->currentPyroChannels();
}

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
