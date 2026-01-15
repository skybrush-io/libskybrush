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
#include <skybrush/refcount.h>
#include <skybrush/screenplay.h>

static sb_error_t sb_i_screenplay_ensure_has_free_space(sb_screenplay_t* screenplay);

/**
 * @brief Initializes a screenplay structure.
 *
 * @param screenplay  the screenplay to initialize
 * @return \c SB_SUCCESS if the screenplay was initialized successfully,
 *       \c SB_ENOMEM if a memory allocation failed.
 */
sb_error_t sb_screenplay_init(sb_screenplay_t* screenplay)
{
    const int initial_capacity = 4;

    screenplay->chapters = sb_calloc(sb_screenplay_chapter_t, initial_capacity);
    if (screenplay->chapters == NULL) {
        return SB_ENOMEM; /* LCOV_EXCL_LINE */
    }

    screenplay->num_chapters = 0;
    screenplay->max_chapters = initial_capacity;

    return SB_SUCCESS;
}

/**
 * @brief Destroys a screenplay structure, freeing all associated resources.
 *
 * @param screenplay  the screenplay to destroy
 */
void sb_screenplay_destroy(sb_screenplay_t* screenplay)
{
    sb_screenplay_clear(screenplay);

    if (screenplay->chapters != NULL) {
        sb_free(screenplay->chapters);
    }

    screenplay->num_chapters = 0;
    screenplay->max_chapters = 0;
}

/**
 * @brief Returns the number of chapters that the screenplay can hold without reallocating memory.
 *
 * @param screenplay  the screenplay to query
 * @return the capacity of the screenplay
 */
size_t sb_screenplay_capacity(const sb_screenplay_t* screenplay)
{
    return screenplay->max_chapters;
}

/**
 * @brief Returns the number of chapters in the screenplay.
 *
 * @param screenplay  the screenplay to query
 * @return the number of chapters in the screenplay
 */
size_t sb_screenplay_size(const sb_screenplay_t* screenplay)
{
    return screenplay->num_chapters;
}

/**
 * @brief Checks whether the screenplay is empty (has no chapters).
 *
 * @param screenplay  the screenplay to query
 * @return \c true if the screenplay is empty, \c false otherwise
 */
sb_bool_t sb_screenplay_is_empty(const sb_screenplay_t* screenplay)
{
    return screenplay->num_chapters == 0;
}

/**
 * @brief Removes all chapters from the screenplay.
 *
 * @param screenplay  the screenplay to clear
 */
void sb_screenplay_clear(sb_screenplay_t* screenplay)
{
    while (!sb_screenplay_is_empty(screenplay)) {
        sb_screenplay_remove_last_chapter(screenplay); /* will succeed */
    }
}

/**
 * @brief Returns a pointer to the chapter at the given index in the screenplay.
 *
 * You can modify the chapter via the provided pointer.
 *
 * @param screenplay  the screenplay to query
 * @param index   the index of the chapter to return
 * @return a pointer to the chapter at the given index, or \c NULL if the index
 *         is out of bounds
 */
sb_screenplay_chapter_t* sb_screenplay_get_chapter_ptr(
    sb_screenplay_t* screenplay, size_t index)
{
    return (index < screenplay->num_chapters)
        ? &screenplay->chapters[index]
        : NULL;
}

/**
 * @brief Returns a pointer to the chapter that is active at the given time.
 *
 * The time is specified in milliseconds from the start of the screenplay.
 * If no chapter is active at the given time, \c NULL is returned.
 *
 * @param screenplay  the screenplay to query
 * @param time_msec   pointer to a variable specifying the time in milliseconds from the
 *        start of the screenplay. It will be updated to contain the time offset
 *        within the returned chapter. Its value will be zero upon returning when the
 *        returned chapter is \c NULL.
 * @return a pointer to the chapter that is active at the given time, or \c NULL
 *         if no chapter is active at that time
 */
sb_screenplay_chapter_t* sb_screenplay_get_current_chapter_ptr(
    sb_screenplay_t* screenplay, uint32_t* time_msec)
{
    for (size_t i = 0; i < screenplay->num_chapters; i++) {
        sb_screenplay_chapter_t* chapter = &screenplay->chapters[i];
        uint32_t chapter_duration_msec = sb_screenplay_chapter_get_duration_msec(chapter);

        if (chapter_duration_msec == UINT32_MAX) {
            /* Infinite duration chapter -> always active */
            return chapter;
        }

        if (*time_msec < chapter_duration_msec) {
            /* Current chapter found */
            return chapter;
        }

        *time_msec -= chapter_duration_msec;
    }

    *time_msec = 0;
    return NULL;
}

/**
 * @brief Appends a new chapter to the end of the screenplay.
 *
 * The new chapter is initialized with default values.
 *
 * @param screenplay  the screenplay to append the chapter to
 * @param out_chapter  if not \c NULL, will be set to point to the newly added chapter.
 *        The returned value is a borrowed reference.
 * @return \c SB_SUCCESS if the chapter was appended successfully,
 *         \c SB_ENOMEM if a memory allocation failed
 */
sb_error_t sb_screenplay_append_new_chapter(sb_screenplay_t* screenplay, sb_screenplay_chapter_t** out_chapter)
{
    sb_screenplay_chapter_t* chapter;

    SB_CHECK(sb_i_screenplay_ensure_has_free_space(screenplay));

    chapter = &screenplay->chapters[screenplay->num_chapters];
    SB_CHECK(sb_screenplay_chapter_init(chapter));
    screenplay->num_chapters++;

    if (out_chapter != NULL) {
        *out_chapter = chapter;
    }

    return SB_SUCCESS;
}

/**
 * @brief Removes the last chapter from the screenplay.
 *
 * @param screenplay  the screenplay to remove the chapter from
 * @return \c SB_SUCCESS if the chapter was removed successfully,
 *         \c SB_EEMPTY if there are no chapters to remove
 */
sb_error_t sb_screenplay_remove_last_chapter(sb_screenplay_t* screenplay)
{
    if (screenplay->num_chapters == 0) {
        return SB_EEMPTY;
    }

    screenplay->num_chapters--;
    SB_DECREF_STATIC(&screenplay->chapters[screenplay->num_chapters]);

    return SB_SUCCESS;
}

/**
 * @brief Updates the screenplay from binary show file data in memory.
 *
 * This function clears the existing chapters in the screenplay and creates a new
 * chapter based on the provided binary show file data. If the show data is NULL or has
 * zero length, the screenplay will be cleared and no new chapter will be added.
 */
sb_error_t sb_screenplay_update_from_binary_file_in_memory(sb_screenplay_t* screenplay, uint8_t* show_data, size_t length)
{
    sb_screenplay_chapter_t* chapter = NULL;
    sb_error_t retval = SB_SUCCESS;

    sb_screenplay_clear(screenplay);
    if (show_data && length > 0) {
        SB_CHECK(sb_screenplay_append_new_chapter(screenplay, &chapter));

        retval = sb_screenplay_chapter_update_from_binary_file_in_memory(chapter, show_data, length);
        if (retval != SB_SUCCESS) {
            sb_screenplay_clear(screenplay);
        }
    }

    return retval;
}

/* ************************************************************************** */

/**
 * @brief Ensures that the screenplay has enough free space to store a new
 * chapter.
 *
 * If the screenplay does not have enough free space, it will be resized to
 * accommodate the new chapter.
 *
 * @param screenplay  the screenplay to check
 * @return \c SB_SUCCESS if there is enough free space,
 *         \c SB_ENOMEM if memory allocation failed
 */
static sb_error_t sb_i_screenplay_ensure_has_free_space(sb_screenplay_t* screenplay)
{
    size_t free_space = screenplay->max_chapters - screenplay->num_chapters;

    if (free_space == 0) {
        size_t new_capacity = screenplay->max_chapters * 2;
        sb_screenplay_chapter_t* new_chapters = sb_realloc(screenplay->chapters, sb_screenplay_chapter_t, new_capacity);
        if (new_chapters == 0) {
            return SB_ENOMEM; /* LCOV_EXCL_LINE */
        }

        screenplay->chapters = new_chapters;
        screenplay->max_chapters = new_capacity;
    }

    return SB_SUCCESS;
}
