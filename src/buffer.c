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

#include <assert.h>
#include <string.h> /* memset */

#include <skybrush/buffer.h>
#include <skybrush/memory.h>

static sb_error_t sb_i_buffer_ensure_free_space(sb_buffer_t* buf, size_t min_space);
static sb_error_t sb_i_buffer_realloc(sb_buffer_t* buf, size_t new_capacity);

/**
 * @brief Initializes a new buffer.
 *
 * @param buf  the buffer to initialize
 * @param initial_size  the initial size of the buffer
 * @return Error code.
 */
sb_error_t sb_buffer_init(sb_buffer_t* buf, size_t initial_size)
{
    size_t alloc_size = initial_size > 1 ? initial_size : 1;

    buf->stor_begin = sb_calloc(uint8_t, alloc_size);
    if (buf->stor_begin == 0) {
        return SB_ENOMEM; /* LCOV_EXCL_LINE */
    }

    buf->stor_end = buf->stor_begin + alloc_size;
    buf->end = buf->stor_begin + initial_size;

    buf->owned = 1;

    return SB_SUCCESS;
}

/**
 * @brief Initializes a new buffer, taking ownership of the given array.
 *
 * Ownership of the array will be transferred to the buffer; the buffer will be
 * responsible for destroying it and the caller may not free it any more.
 *
 * @param buf  the buffer to initialize
 * @param bytes  the bytes to take ownership of
 * @param num_bytes  the number of bytes in the array; must be positive
 * @return Error code.
 */
sb_error_t sb_buffer_init_from_bytes(sb_buffer_t* buf, void* bytes, size_t num_bytes)
{
    if (num_bytes == 0) {
        return SB_EINVAL;
    }

    buf->stor_begin = bytes;
    buf->stor_end = buf->stor_begin + num_bytes;
    buf->end = buf->stor_begin + num_bytes;
    buf->owned = 1;

    return SB_SUCCESS;
}

/**
 * @brief Destroys a buffer, freeing all associated memory.
 */
void sb_buffer_destroy(sb_buffer_t* buf)
{
    if (buf->owned && buf->stor_begin != 0) {
        sb_free(buf->stor_begin);
        buf->stor_begin = 0;
    }

    buf->stor_end = 0;
    buf->end = 0;
}

/**
 * @brief Creates a buffer view into an existing array of bytes.
 *
 * The created buffer is not allowed to grow or shrink; attempts to do so
 * will yield an error code.
 *
 * @param buf the buffer to initialize as a view
 * @param bytes the array of bytes
 * @param num_bytes the number of bytes in the array
 */
void sb_buffer_init_view(sb_buffer_t* buf, void* bytes, size_t num_bytes)
{
    assert(bytes != 0);
    buf->stor_begin = bytes;
    buf->stor_end = buf->end = bytes + num_bytes;
    buf->owned = 0;
}

/**
 * @brief Returns the current size of the buffer.
 *
 * @param buf  the buffer
 * @return the current size of the buffer
 */
size_t sb_buffer_size(const sb_buffer_t* buf)
{
    assert(buf && buf->stor_begin != 0);
    return buf->end - buf->stor_begin;
}

/**
 * @brief Returns the allocated size of the buffer.
 *
 * This is the size that the buffer can grow to without having to re-allocate
 * the underlying storage.
 *
 * @param buf  the buffer
 * @return the current size of the buffer
 */
size_t sb_buffer_capacity(const sb_buffer_t* buf)
{
    assert(buf && buf->stor_begin != 0);
    return buf->stor_end - buf->stor_begin;
}

/**
 * @brief Returns whether the buffer is a view into an array.
 *
 * @param buf the buffer
 * @return whether the buffer is a view into an array
 */
sb_bool_t sb_buffer_is_view(const sb_buffer_t* buf)
{
    return !buf->owned;
}

/**
 * @brief Clears the buffer, setting its size to zero.
 *
 * Note that this function does not deallocate any memory in case the buffer
 * needs to grow again later.
 *
 * @param buf  the buffer
 * @return error code.
 */
sb_error_t sb_buffer_clear(sb_buffer_t* buf)
{
    return sb_buffer_resize(buf, 0);
}

/**
 * @brief Sets the size of the buffer, allocating more memory if needed.
 *
 * Note that this function does not deallocate any memory if the size of the
 * buffer decreases in case the buffer needs to grow again later.
 *
 * @param buf  the buffer
 * @return error code.
 */
sb_error_t sb_buffer_resize(sb_buffer_t* buf, size_t new_size)
{
    size_t current_size = sb_buffer_size(buf);

    if (!buf->owned) {
        return SB_FAILURE;
    }

    if (current_size < new_size) {
        SB_CHECK(sb_i_buffer_realloc(buf, new_size));
        memset(buf->end, 0, new_size - current_size);
    }

    buf->end = buf->stor_begin + new_size;
    return SB_SUCCESS;
}

/**
 * @brief Resizes the buffer such that its capacity becomes equal to its
 * current size.
 *
 * The goal of this function is to make the amount of memory allocated to the
 * buffer as small as possible. If there is excess memory allocated to the
 * buffer (i.e. its capacity is larger than its size), the excess memory will
 * be freed.
 *
 * @param buf  the buffer
 * @return error code
 */
sb_error_t sb_buffer_prune(sb_buffer_t* buf)
{
    return sb_i_buffer_realloc(buf, sb_buffer_size(buf));
}

/**
 * @brief Fills the buffer with the given value.
 *
 * @param buf  the buffer
 * @param value  the value to write into each byte in the buffer
 */
void sb_buffer_fill(sb_buffer_t* buf, uint8_t value)
{
    memset(buf->stor_begin, value, sb_buffer_size(buf));
}

/**
 * @brief Appends a single byte to the end of the buffer, growing it as needed.
 *
 * @param buf  the buffer
 * @param byte the byte to append
 * @return error code
 */
sb_error_t sb_buffer_append_byte(sb_buffer_t* buf, uint8_t byte)
{
    return sb_buffer_append_bytes(buf, &byte, 1);
}

/**
 * @brief Appends multiple bytes to the end of the buffer, growing it as needed.
 *
 * The byte array given in the argument will be copied.
 *
 * @param buf  the buffer
 * @param bytes pointer to the byte array to append
 * @param num_bytes  the number of bytes to append
 * @return error code
 */
sb_error_t sb_buffer_append_bytes(sb_buffer_t* buf, const void* bytes, size_t num_bytes)
{
    SB_CHECK(sb_i_buffer_ensure_free_space(buf, num_bytes));
    memcpy(buf->end, bytes, num_bytes);
    buf->end += num_bytes;
    return SB_SUCCESS;
}

/**
 * @brief Extends a buffer with another one, growing the buffer as needed.
 *
 * The contents of the other buffer will be copied.
 *
 * @param buf  the buffer to extend
 * @param other  the other buffer to extend the buffer with
 * @return error code
 */
sb_error_t sb_buffer_concat(sb_buffer_t* buf, const sb_buffer_t* other)
{
    return sb_buffer_append_bytes(buf, other->stor_begin, sb_buffer_size(other));
}

/* ************************************************************************* */

/**
 * @brief Ensures that there is free space at the end of the buffer.
 *
 * This function may resize the internal storage if needed, and the new storage
 * area may be larger than what would strictly be needed to accommodate the new
 * size.
 *
 * @param buf  the buffer
 * @param min_space  minimum number of bytes that must be free at the end of the
 *        buffer
 */
static sb_error_t sb_i_buffer_ensure_free_space(sb_buffer_t* buf, size_t min_space)
{
    size_t old_size, new_size, desired_capacity;

    if (min_space == 0) {
        return SB_SUCCESS;
    }

    desired_capacity = sb_buffer_capacity(buf);
    old_size = sb_buffer_size(buf);
    new_size = old_size + min_space;
    if (new_size < old_size) {
        /* overflow */
        return SB_ENOMEM;
    }

    while (new_size > desired_capacity) {
        if (desired_capacity >= SIZE_MAX - 1) {
            return SB_ENOMEM;
        } else if (desired_capacity <= (SIZE_MAX >> 1)) {
            desired_capacity <<= 1;
        } else {
            desired_capacity = SIZE_MAX - 1;
        }
    }

    return sb_i_buffer_realloc(buf, desired_capacity);
}

sb_error_t sb_i_buffer_realloc(sb_buffer_t* buf, size_t new_capacity)
{
    size_t capacity = sb_buffer_capacity(buf);
    size_t size;

    if (new_capacity < 1) {
        new_capacity = 1;
    }

    if (capacity != new_capacity) {
        if (!buf->owned) {
            return SB_FAILURE;
        }

        size = sb_buffer_size(buf);

        buf->stor_begin = sb_realloc(buf->stor_begin, uint8_t, new_capacity);
        if (buf->stor_begin == 0) {
            buf->stor_end = buf->end = 0;
            return SB_ENOMEM; /* LCOV_EXCL_LINE */
        }

        buf->stor_end = buf->stor_begin + new_capacity;
        buf->end = buf->stor_begin + size;

        if (buf->end > buf->stor_end) {
            /* this branch happens only if reallocating to a smaller size and
             * the memory allocator decided to move the allocated chunk anyway.
             * This is hard to test so we ignore it */
            buf->end = buf->stor_end; /* LCOV_EXCL_LINE */
        }
    }

    return SB_SUCCESS;
}
