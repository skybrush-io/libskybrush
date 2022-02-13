/*
 * This file is part of libskybrush.
 *
 * Copyright 2020-2022 CollMot Robotics Ltd.
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

#ifndef SKYBRUSH_BUFFER_H
#define SKYBRUSH_BUFFER_H

#include <skybrush/basic_types.h>
#include <skybrush/decls.h>
#include <skybrush/error.h>
#include <stdint.h>
#include <stdlib.h>

__BEGIN_DECLS

/**
 * @file buffer.h
 * @brief Dynamic buffer of bytes that can grow or shrink as needed.
 */

/**
 * @brief Dynamic buffer of bytes that can grow or shrink as needed.
 */
typedef struct {
    uint8_t* stor_begin; /**< Start of the buffer */
    uint8_t* end; /**< End of the \em used part of the buffer */
    uint8_t* stor_end; /**< End of the allocated storage area */
    sb_bool_t owned; /**< Whether the struct owns the internal buffer */
} sb_buffer_t;

/**
 * @def SB_BUFFER
 * @brief Macro that points to the internal storage area of the buffer.
 */
#define SB_BUFFER(buf) (buf.stor_begin)

sb_error_t sb_buffer_init(sb_buffer_t* buf, size_t initial_size);
sb_error_t sb_buffer_init_from_bytes(sb_buffer_t* buf, void* bytes, size_t num_bytes);
void sb_buffer_init_view(sb_buffer_t* buf, void* bytes, size_t num_bytes);
void sb_buffer_destroy();

size_t sb_buffer_capacity(const sb_buffer_t* buf);
sb_bool_t sb_buffer_is_view(const sb_buffer_t* buf);
size_t sb_buffer_size(const sb_buffer_t* buf);

sb_error_t sb_buffer_clear(sb_buffer_t* buf);
sb_error_t sb_buffer_resize(sb_buffer_t* buf, size_t new_size);
sb_error_t sb_buffer_prune(sb_buffer_t* buf);

void sb_buffer_fill(sb_buffer_t* buf, uint8_t value);
sb_error_t sb_buffer_append_byte(sb_buffer_t* buf, uint8_t byte);
sb_error_t sb_buffer_append_bytes(sb_buffer_t* buf, const void* bytes, size_t num_bytes);
sb_error_t sb_buffer_concat(sb_buffer_t* buf, const sb_buffer_t* other);

__END_DECLS

#endif
