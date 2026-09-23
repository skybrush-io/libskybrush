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

#include <stddef.h>
#include <stdint.h>

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

/* Vector equality: compare components with a specified tolerance suitable for tests.
 * Use a do/while block so the macro behaves like a statement.
 */
#define TEST_ASSERT_EQUAL_VECTOR3_EPS(expected, actual, eps)     \
    do {                                                         \
        TEST_ASSERT_FLOAT_WITHIN(eps, (expected).x, (actual).x); \
        TEST_ASSERT_FLOAT_WITHIN(eps, (expected).y, (actual).y); \
        TEST_ASSERT_FLOAT_WITHIN(eps, (expected).z, (actual).z); \
    } while (0)

/* Vector equality: compare components with a small tolerance suitable for tests.
 * Use a do/while block so the macro behaves like a statement.
 */
#define TEST_ASSERT_EQUAL_VECTOR3(expected, actual)               \
    do {                                                          \
        TEST_ASSERT_EQUAL_VECTOR3_EPS((expected), (actual), eps); \
    } while (0)

/* Convenience macro: specify expected vector components directly (x, y, z) and the actual vector variable.
 * Usage: TEST_ASSERT_EQUAL_VECTOR3_XYZ(0.0f, 0.0f, 5000.0f, actual_vec);
 * Delegates to TEST_ASSERT_EQUAL_VECTOR3_XYZ_EPS.
 */
#define TEST_ASSERT_EQUAL_VECTOR3_XYZ(x, y, z, actual)                    \
    do {                                                                  \
        TEST_ASSERT_EQUAL_VECTOR3_XYZ_EPS((x), (y), (z), (actual), 1e-6); \
    } while (0)

/* Convenience macro: specify expected vector components directly (x, y, z) and the actual vector variable,
 * plus a tolerance.
 * Usage: TEST_ASSERT_EQUAL_VECTOR3_XYZ(0.0f, 0.0f, 5000.0f, actual_vec, 1e-1);
 * This constructs a local sb_vector3_t expected and delegates to TEST_ASSERT_EQUAL_VECTOR3_EPS.
 */
#define TEST_ASSERT_EQUAL_VECTOR3_XYZ_EPS(x, y, z, actual, eps)   \
    do {                                                          \
        sb_vector3_t expected = { (x), (y), (z) };                \
        TEST_ASSERT_EQUAL_VECTOR3_EPS(expected, (actual), (eps)); \
    } while (0)

/* Writes the header of a Skybrush binary show file (the magic word "skyb"
 * followed by the version number 1) into the given buffer at the given
 * offset and returns the offset right after the header. Useful when a unit
 * test needs to prepare an in-memory Skybrush binary show file.
 */
static inline size_t make_skyb_header(uint8_t* buf, size_t offset)
{
    buf[offset++] = 0x73; /* "s" */
    buf[offset++] = 0x6b; /* "k" */
    buf[offset++] = 0x79; /* "y" */
    buf[offset++] = 0x62; /* "b" */
    buf[offset++] = 0x01; /* version */

    return offset;
}

#endif
