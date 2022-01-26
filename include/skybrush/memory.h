/* vim:set ts=4 sw=4 sts=4 et: */

#ifndef SKYBRUSH_MEMORY_H
#define SKYBRUSH_MEMORY_H

#include <memory.h>
#include <stdlib.h>

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
            nc_free(ptr);        \
    }

/**
 * Friendlier form of realloc.
 */
#define sb_realloc(ptr, type, count) ((type*)realloc(ptr, sizeof(type) * (count)))

#endif
