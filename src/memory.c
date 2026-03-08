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

#include <skybrush/memory.h>
#include <string.h>

void* sb_i_simple_realloc(void* ptr, size_t old_size, size_t new_size)
{
    if (new_size == 0) {
        sb_free(ptr);
        return 0;
    }

    if (ptr == 0) {
        return sb_malloc(uint8_t, new_size);
    }

    void* new_mem = sb_malloc(uint8_t, new_size);
    if (new_mem != 0) {
        memcpy(new_mem, ptr, old_size < new_size ? old_size : new_size);
    }

    return new_mem;
}
