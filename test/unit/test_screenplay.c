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

#include "unity.h"

#include <skybrush/screenplay.h>

void setUp(void)
{
    /* nothing to set up */
}

void tearDown(void)
{
    /* nothing to tear down */
}

void test_screenplay_init_sets_defaults_and_allocates(void)
{
    sb_screenplay_t screenplay;
    sb_error_t err;

    /* Initialize the screenplay */
    err = sb_screenplay_init(&screenplay);
    TEST_ASSERT_EQUAL(SB_SUCCESS, err);

    /* chapters storage must be allocated */
    TEST_ASSERT_NOT_NULL(screenplay.chapters);

    /* size must be zero, capacity at least 1 */
    TEST_ASSERT_EQUAL(0u, sb_screenplay_size(&screenplay));
    TEST_ASSERT_TRUE(sb_screenplay_capacity(&screenplay) >= 1u);

    /* getting chapter pointer must return NULL for empty screenplay */
    TEST_ASSERT_NULL(sb_screenplay_get_chapter_ptr(&screenplay, 0));
    TEST_ASSERT_NULL(sb_screenplay_get_chapter_ptr_const(&screenplay, 0));

    /* Clean up */
    sb_screenplay_destroy(&screenplay);

    /* After destroy, size and capacity must be zero */
    TEST_ASSERT_EQUAL(0u, sb_screenplay_size(&screenplay));
    TEST_ASSERT_EQUAL(0u, sb_screenplay_capacity(&screenplay));

    /* get chapter should still return NULL (num_chapters is zero) */
    TEST_ASSERT_NULL(sb_screenplay_get_chapter_ptr(&screenplay, 0));
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_screenplay_init_sets_defaults_and_allocates);

    return UNITY_END();
}
