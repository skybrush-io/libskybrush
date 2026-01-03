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
 * @brief Removes all chapters from the screenplay.
 *
 * @param screenplay  the screenplay to clear
 */
void sb_screenplay_clear(sb_screenplay_t* screenplay)
{
    sb_screenplay_chapter_t* end = screenplay->chapters + screenplay->num_chapters;
    sb_screenplay_chapter_t* chapter;
    for (chapter = screenplay->chapters; chapter < end; chapter++) {
        sb_screenplay_chapter_destroy(chapter);
    }

    screenplay->num_chapters = 0;
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
 * @brief Returns a pointer to the chapter at the given index in the screenplay (const variant).
 *
 * The chapter returned by the provided pointer should not be modified.
 *
 * @param screenplay  the screenplay to query
 * @param index   the index of the chapter to return
 * @return a pointer to the chapter at the given index, or \c NULL if the index
 *         is out of bounds
 */
const sb_screenplay_chapter_t* sb_screenplay_get_chapter_ptr_const(
    const sb_screenplay_t* screenplay, size_t index)
{
    return (index < screenplay->num_chapters)
        ? &screenplay->chapters[index]
        : NULL;
}

/**
 * @brief Appends a new chapter to the end of the screenplay.
 *
 * The new chapter is initialized with default values.
 *
 * @param screenplay  the screenplay to append the chapter to
 * @param out_chapter  if not \c NULL, will be set to point to the newly added chapter
 * @return \c SB_SUCCESS if the chapter was appended successfully,
 *         \c SB_ENOMEM if a memory allocation failed
 */
sb_error_t sb_screenplay_append_chapter(sb_screenplay_t* screenplay, sb_screenplay_chapter_t* out_chapter)
{
    sb_screenplay_chapter_t* chapter;

    SB_CHECK(sb_i_screenplay_ensure_has_free_space(screenplay));

    chapter = &screenplay->chapters[screenplay->num_chapters];
    SB_CHECK(sb_screenplay_chapter_init(chapter));
    screenplay->num_chapters++;

    if (out_chapter != NULL) {
        *out_chapter = *chapter;
    }

    return SB_SUCCESS;
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
