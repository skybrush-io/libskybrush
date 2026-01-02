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

#ifndef SKYBRUSH_REFCOUNT_H
#define SKYBRUSH_REFCOUNT_H

/**
 * @file refcount.h
 * @brief Reference counting for data structures whose ownership may be shared.
 */

#include <skybrush/decls.h>
#include <skybrush/memory.h>
#include <stdint.h>

__BEGIN_DECLS

/**
 * @brief Type for reference counts.
 *
 * Do not use an unsigned type here; -1 may be used in the future for special purposes.
 */
typedef int16_t sb_ref_count_t;

/**
 * @brief Function type for freeing a reference-counted object.
 *
 * This function is called when the reference count reaches zero.
 *
 * @param obj Pointer to the object to be freed.
 */
typedef void (*sb_destructor_t)(void*);

/**
 * @brief Base structure for reference-counted objects.
 *
 * This structure should be included as the first member of any
 * reference-counted object using the macro \c SB_REFCOUNTED.
 */
typedef struct {
    sb_destructor_t destructor;
    sb_ref_count_t ref_count;
} sb_ref_counted_t;

/**
 * @def SB_REFCOUNTED
 *
 * Marks a data structure as reference-counted by including the necessary
 * bookkeeping structure (\c sb_ref_counted_t) as a member named
 * \c ref_counted.
 */
#define SB_REFCOUNTED \
    sb_ref_counted_t ref_counted

/**
 * @def SB_REF_INIT(obj, destructor_func)
 *
 * Initializes the data structure responsible for reference counting in a
 * reference-counted object. The reference count is set to 0, and the given destructor
 * function (\c destructor_func) is associated to the object.
 */
#define SB_REF_INIT(obj, destructor_func)                                   \
    do {                                                                    \
        (obj)->ref_counted.destructor = ((sb_destructor_t)destructor_func); \
        (obj)->ref_counted.ref_count = 1;                                   \
    } while (0)

/**
 * @def SB_REFCNT(obj)
 *
 * Returns the current reference count of a reference-counted object.
 */
#define SB_REFCNT(obj) ((obj)->ref_counted.ref_count)

/**
 * @def SB_INCREF(obj)
 *
 * Increments the reference count of a reference-counted object.
 */
#define SB_INCREF(obj) \
    do {               \
        SB_REFCNT(obj) \
        ++;            \
    } while (0)

/**
 * @def SB_DECREF(obj)
 *
 * Decrements the reference count of a reference-counted object allocated on the heap.
 * If the reference count reaches zero, the associated destructor function is called to
 * destroy the object and free its memory.
 */
#define SB_DECREF(obj)                                       \
    do {                                                     \
        if (--(SB_REFCNT(obj)) == 0) {                       \
            if ((obj)->ref_counted.destructor) {             \
                (obj)->ref_counted.destructor((void*)(obj)); \
                sb_free(obj);                                \
            }                                                \
        }                                                    \
    } while (0)

/**
 * @def SB_DECREF_LOCAL(obj)
 *
 * Decrements the reference count of a reference-counted object on the stack. If the
 * reference count reaches zero, the associated destructor function is called to destroy
 * the object. No attempts are made to free the object, as it is assumed to be on the
 * stack.
 */
#define SB_DECREF_LOCAL(obj)                                 \
    do {                                                     \
        if (--(SB_REFCNT(obj)) == 0) {                       \
            if ((obj)->ref_counted.destructor) {             \
                (obj)->ref_counted.destructor((void*)(obj)); \
            }                                                \
        }                                                    \
    } while (0)

/**
 * @def SB_XDECREF(obj)
 *
 * Decrements the reference count of a reference-counted object, with null checking.
 */
#define SB_XDECREF(obj)     \
    do {                    \
        if ((obj) != 0) {   \
            SB_DECREF(obj); \
        }                   \
    } while (0)

__END_DECLS

#endif
