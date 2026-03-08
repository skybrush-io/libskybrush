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

#ifndef SKYBRUSH_MEMORY_H
#define SKYBRUSH_MEMORY_H

/**
 * @file memory.h
 * @brief Basic macros related to memory management
 */

#include <stdlib.h>

/**
 * Friendlier form of malloc.
 */
#define sb_malloc(type, count) ((type*)malloc(count * sizeof(type)))

/**
 * Friendlier form of calloc.
 */
#define sb_calloc(type, count) ((type*)calloc(count, sizeof(type)))

/**
 * Macro that frees a pointer and resets it to null.
 */
#define sb_free(ptr) \
    {                \
        free(ptr);   \
        ptr = 0;     \
    }

/**
 * Macro that frees a pointer unless it is to null.
 */
#define sb_free_unless_null(ptr) \
    {                            \
        if (ptr != 0)            \
            sb_free(ptr);        \
    }

/**
 * Somewhat friendlier form of realloc, with an extra argument that specifies the
 * old size of the buffer allocated at \p ptr. This allows us to use a simple
 * malloc-followed-by-free approach if the platform does not provide \c realloc().
 */
#define sb_realloc(ptr, type, old_count, new_count) ((type*)sb_i_simple_realloc(ptr, sizeof(type) * (old_count), sizeof(type) * (new_count)))

void* sb_i_simple_realloc(void* ptr, size_t old_size, size_t new_size);

#endif
