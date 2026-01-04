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

#ifndef UNITY_TEST_UTILS_H
#define UNITY_TEST_UTILS_H

/* Helper macros for unit tests */

/* Color equality: uses the library helper */
#define TEST_ASSERT_EQUAL_COLOR(expected, actual) TEST_ASSERT_TRUE(sb_rgb_color_equals((expected), (actual)))

/* Convenience macro to assert color equality by specifying RGB components directly.
 * Usage: TEST_ASSERT_EQUAL_COLOR_RGB(255, 127, 127, actual_color_var);
 * This constructs a local sb_rgb_color_t expected variable and delegates to
 * TEST_ASSERT_EQUAL_COLOR.
 */
#define TEST_ASSERT_EQUAL_COLOR_RGB(r, g, b, actual)                            \
    do {                                                                        \
        sb_rgb_color_t expected = { (uint8_t)(r), (uint8_t)(g), (uint8_t)(b) }; \
        TEST_ASSERT_EQUAL_COLOR(expected, (actual));                            \
    } while (0)

/* Vector equality: compare components with a small tolerance suitable for tests.
 * Use a do/while block so the macro behaves like a statement.
 */
#define TEST_ASSERT_EQUAL_VECTOR3(expected, actual)                \
    do {                                                           \
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, (expected).x, (actual).x); \
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, (expected).y, (actual).y); \
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, (expected).z, (actual).z); \
    } while (0)

/* Convenience macro: specify expected vector components directly (x, y, z) and the actual vector variable.
 * Usage: TEST_ASSERT_EQUAL_VECTOR3_XYZ(0.0f, 0.0f, 5000.0f, actual_vec);
 * This constructs a local sb_vector3_t expected and delegates to TEST_ASSERT_EQUAL_VECTOR3.
 */
#define TEST_ASSERT_EQUAL_VECTOR3_XYZ(x, y, z, actual) \
    do {                                               \
        sb_vector3_t expected = { (x), (y), (z) };     \
        TEST_ASSERT_EQUAL_VECTOR3(expected, (actual)); \
    } while (0)

#endif
