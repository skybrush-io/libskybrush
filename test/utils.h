/*
 * This file is part of libskybrush.
 *
 * Copyright 2020-2023 CollMot Robotics Ltd.
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

#ifndef UNITY_TEST_UTILS_H
#define UNITY_TEST_UTILS_H

#define TEST_ASSERT_EQUAL_COLOR(expected, actual) TEST_ASSERT_TRUE(sb_rgb_color_equals(expected, actual))

#endif
