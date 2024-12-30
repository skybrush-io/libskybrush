/*
 * This file is part of libskybrush.
 *
 * Copyright 2020-2024 CollMot Robotics Ltd.
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

#include <float.h>
#include <skybrush/poly.h>

#include "unity.h"

static int compare_floats(const void* a, const void* b)
{
    float diff = *(float*)a - *(float*)b;
    if (diff < 0) {
        return -1;
    } else if (diff > 0) {
        return 1;
    } else {
        return 0;
    }
}

void setUp(void)
{
}

void tearDown(void)
{
}

void check_if_poly_is_all_zero(sb_poly_t* poly)
{
    TEST_ASSERT_EQUAL(0, sb_poly_get_degree(poly));

    TEST_ASSERT_EQUAL(0, sb_poly_eval(poly, 0));
    TEST_ASSERT_EQUAL(0, sb_poly_eval(poly, -2));
    TEST_ASSERT_EQUAL(0, sb_poly_eval(poly, 1));

    TEST_ASSERT_EQUAL(0, sb_poly_eval_double(poly, 0));
    TEST_ASSERT_EQUAL(0, sb_poly_eval_double(poly, -2));
    TEST_ASSERT_EQUAL(0, sb_poly_eval_double(poly, 1));
}

void test_zero(void)
{
    sb_poly_t poly;

    sb_poly_make_zero(&poly);
    check_if_poly_is_all_zero(&poly);
}

void test_zero_4d(void)
{
    sb_poly_4d_t poly;

    sb_poly_4d_make_zero(&poly);
    check_if_poly_is_all_zero(&poly.x);
    check_if_poly_is_all_zero(&poly.y);
    check_if_poly_is_all_zero(&poly.z);
    check_if_poly_is_all_zero(&poly.yaw);
}

void test_constant(void)
{
    sb_poly_t poly;

    sb_poly_make_constant(&poly, 3);

    TEST_ASSERT_EQUAL(0, sb_poly_get_degree(&poly));

    TEST_ASSERT_EQUAL(3, sb_poly_eval(&poly, 0));
    TEST_ASSERT_EQUAL(3, sb_poly_eval(&poly, -2));
    TEST_ASSERT_EQUAL(3, sb_poly_eval(&poly, 1));

    TEST_ASSERT_EQUAL(3, sb_poly_eval_double(&poly, 0));
    TEST_ASSERT_EQUAL(3, sb_poly_eval_double(&poly, -2));
    TEST_ASSERT_EQUAL(3, sb_poly_eval_double(&poly, 1));
}

void test_linear(void)
{
    sb_poly_t poly;

    sb_poly_make_linear(&poly, 5, 10, 20);

    TEST_ASSERT_EQUAL(1, sb_poly_get_degree(&poly));

    TEST_ASSERT_EQUAL(8, sb_poly_eval(&poly, -1));
    TEST_ASSERT_EQUAL(10, sb_poly_eval(&poly, 0));
    TEST_ASSERT_EQUAL(12, sb_poly_eval(&poly, 1));
    TEST_ASSERT_EQUAL(14, sb_poly_eval(&poly, 2));
    TEST_ASSERT_EQUAL(16, sb_poly_eval(&poly, 3));
    TEST_ASSERT_EQUAL(18, sb_poly_eval(&poly, 4));
    TEST_ASSERT_EQUAL(20, sb_poly_eval(&poly, 5));
    TEST_ASSERT_EQUAL(22, sb_poly_eval(&poly, 6));

    TEST_ASSERT_EQUAL(8, sb_poly_eval_double(&poly, -1));
    TEST_ASSERT_EQUAL(10, sb_poly_eval_double(&poly, 0));
    TEST_ASSERT_EQUAL(12, sb_poly_eval_double(&poly, 1));
    TEST_ASSERT_EQUAL(14, sb_poly_eval_double(&poly, 2));
    TEST_ASSERT_EQUAL(16, sb_poly_eval_double(&poly, 3));
    TEST_ASSERT_EQUAL(18, sb_poly_eval_double(&poly, 4));
    TEST_ASSERT_EQUAL(20, sb_poly_eval_double(&poly, 5));
    TEST_ASSERT_EQUAL(22, sb_poly_eval_double(&poly, 6));
}

void test_linear_small_durations(void)
{
    sb_poly_t poly;

    sb_poly_make_linear(&poly, FLT_EPSILON, 10, 20);
    TEST_ASSERT_EQUAL(10, sb_poly_eval(&poly, 0));
    TEST_ASSERT_EQUAL(20, sb_poly_eval(&poly, FLT_EPSILON));

    sb_poly_make_linear(&poly, FLT_EPSILON, 1, 1 + 2 * FLT_EPSILON);
    TEST_ASSERT_EQUAL(1, sb_poly_eval(&poly, 0));
    TEST_ASSERT_EQUAL(1 + FLT_EPSILON, sb_poly_eval(&poly, FLT_EPSILON / 2));
    TEST_ASSERT_EQUAL(1 + 2 * FLT_EPSILON, sb_poly_eval(&poly, FLT_EPSILON));

    sb_poly_make_linear(&poly, FLT_MIN, 1, 1 + 2 * FLT_EPSILON);
    TEST_ASSERT_EQUAL(1 + FLT_EPSILON, sb_poly_eval(&poly, 0));
    TEST_ASSERT_EQUAL(1 + FLT_EPSILON, sb_poly_eval(&poly, FLT_MIN));

    sb_poly_make_linear(&poly, FLT_EPSILON, 1, 1 + FLT_EPSILON);
    TEST_ASSERT_EQUAL(1, sb_poly_eval(&poly, 0));
    TEST_ASSERT_EQUAL(1, sb_poly_eval(&poly, FLT_EPSILON));
    TEST_ASSERT_EQUAL(2, sb_poly_eval(&poly, 1));

    sb_poly_make_linear(&poly, FLT_EPSILON / 2, 1, 1 + FLT_EPSILON);
    TEST_ASSERT_EQUAL(1, sb_poly_eval(&poly, 0));
    TEST_ASSERT_EQUAL(1, sb_poly_eval(&poly, FLT_EPSILON / 2));
}

void test_bezier(void)
{
    sb_poly_t poly;
    float xs;

    sb_poly_make_bezier(&poly, 10, &xs, 0);
    TEST_ASSERT_EQUAL(0, sb_poly_eval(&poly, 0));
    TEST_ASSERT_EQUAL(0, sb_poly_eval(&poly, 5));
    TEST_ASSERT_EQUAL(0, sb_poly_eval(&poly, -3));

    sb_poly_make_cubic_bezier(&poly, 4, 0, 0, 5, 5);

    TEST_ASSERT_EQUAL(3, sb_poly_get_degree(&poly));
    TEST_ASSERT_EQUAL(0, sb_poly_eval(&poly, 0));
    TEST_ASSERT_EQUAL(0.78125, sb_poly_eval(&poly, 1));
    TEST_ASSERT_EQUAL(2.5, sb_poly_eval(&poly, 2));
    TEST_ASSERT_EQUAL(4.21875, sb_poly_eval(&poly, 3));
    TEST_ASSERT_EQUAL(5, sb_poly_eval(&poly, 4));

    sb_poly_make_cubic_bezier(&poly, 4, 0, 5, 5, 0);

    TEST_ASSERT_EQUAL(3, sb_poly_get_degree(&poly));
    TEST_ASSERT_EQUAL(0, sb_poly_eval(&poly, 0));
    TEST_ASSERT_EQUAL(2.8125, sb_poly_eval(&poly, 1));
    TEST_ASSERT_EQUAL(3.75, sb_poly_eval(&poly, 2));
    TEST_ASSERT_EQUAL(2.8125, sb_poly_eval(&poly, 3));
    TEST_ASSERT_EQUAL(0, sb_poly_eval(&poly, 4));

    TEST_ASSERT_EQUAL(3, sb_poly_get_degree(&poly));
    TEST_ASSERT_EQUAL(0, sb_poly_eval_double(&poly, 0));
    TEST_ASSERT_EQUAL(2.8125, sb_poly_eval_double(&poly, 1));
    TEST_ASSERT_EQUAL(3.75, sb_poly_eval_double(&poly, 2));
    TEST_ASSERT_EQUAL(2.8125, sb_poly_eval_double(&poly, 3));
    TEST_ASSERT_EQUAL(0, sb_poly_eval_double(&poly, 4));
}

void test_add_constant(void)
{
    sb_poly_t poly;

    sb_poly_make_zero(&poly);
    poly.num_coeffs = 0;
    sb_poly_add_constant(&poly, 7);

    TEST_ASSERT_EQUAL(7, sb_poly_eval(&poly, -1));
    TEST_ASSERT_EQUAL(7, sb_poly_eval(&poly, 0));
    TEST_ASSERT_EQUAL(7, sb_poly_eval(&poly, 3));
    TEST_ASSERT_EQUAL(7, sb_poly_eval(&poly, 5));

    sb_poly_make_constant(&poly, 50);
    sb_poly_add_constant(&poly, 7);

    TEST_ASSERT_EQUAL(57, sb_poly_eval(&poly, -1));
    TEST_ASSERT_EQUAL(57, sb_poly_eval(&poly, 0));
    TEST_ASSERT_EQUAL(57, sb_poly_eval(&poly, 3));
    TEST_ASSERT_EQUAL(57, sb_poly_eval(&poly, 5));

    sb_poly_make_linear(&poly, 5, 10, 20);
    sb_poly_add_constant(&poly, 3);

    TEST_ASSERT_EQUAL(11, sb_poly_eval(&poly, -1));
    TEST_ASSERT_EQUAL(13, sb_poly_eval(&poly, 0));
    TEST_ASSERT_EQUAL(19, sb_poly_eval(&poly, 3));
    TEST_ASSERT_EQUAL(23, sb_poly_eval(&poly, 5));
}

void test_scale(void)
{
    sb_poly_t poly;

    sb_poly_make_linear(&poly, 5, 10, 20);
    sb_poly_scale(&poly, 3);

    TEST_ASSERT_EQUAL(24, sb_poly_eval(&poly, -1));
    TEST_ASSERT_EQUAL(30, sb_poly_eval(&poly, 0));
    TEST_ASSERT_EQUAL(48, sb_poly_eval(&poly, 3));
    TEST_ASSERT_EQUAL(60, sb_poly_eval(&poly, 5));
}

void test_get_degree(void)
{
    sb_poly_t poly;
    float xs[4] = { 0, 7, 13, 61 };

    sb_poly_make_bezier(&poly, 10, xs, sizeof(xs) / sizeof(xs[0]));

    TEST_ASSERT_EQUAL(3, sb_poly_get_degree(&poly));

    sb_poly_deriv(&poly);
    TEST_ASSERT_EQUAL(2, sb_poly_get_degree(&poly));

    sb_poly_deriv(&poly);
    TEST_ASSERT_EQUAL(1, sb_poly_get_degree(&poly));

    sb_poly_deriv(&poly);
    TEST_ASSERT_EQUAL(0, sb_poly_get_degree(&poly));

    sb_poly_deriv(&poly);
    TEST_ASSERT_EQUAL(0, sb_poly_get_degree(&poly));
}

void test_get_extrema(void)
{
    sb_poly_t poly;
    sb_interval_t result;
    float xs[4] = { 0, 7, 13, 61 };
    float quadratic_convex[3] = { 7, -4, 1 }; /* x^2 - 4x + 7 */
    float quadratic_convex_2[3] = { 41 / 16.0f, -3 / 2.0f, 1 }; /* x^2 - 3/2*x + 41/16 */
    float quadratic_concave[3] = { 63 / 16.0f, 1 / 2.0f, -1 }; /* -x^2 + x/2 + 63/16 */
    float linear[5] = { 8, 2, 0, 0, 0 };

    /* ignored result */

    sb_poly_make_bezier(&poly, 10, xs, sizeof(xs) / sizeof(xs[0]));
    TEST_ASSERT_EQUAL(0, sb_poly_get_extrema(&poly, 0));

    /* pathologic case */

    sb_poly_make(&poly, xs, 0);
    TEST_ASSERT_EQUAL(0, sb_poly_get_extrema(&poly, &result));
    TEST_ASSERT_EQUAL(0, result.min);
    TEST_ASSERT_EQUAL(0, result.max);

    /* constant */

    sb_poly_make_constant(&poly, 2);
    TEST_ASSERT_EQUAL(0, sb_poly_get_extrema(&poly, &result));
    TEST_ASSERT_EQUAL(2, result.min);
    TEST_ASSERT_EQUAL(2, result.max);

    /* linear */

    sb_poly_make_linear(&poly, 5, 10, 20);
    TEST_ASSERT_EQUAL(0, sb_poly_get_extrema(&poly, &result));
    TEST_ASSERT_EQUAL(10, result.min);
    TEST_ASSERT_EQUAL(12, result.max);

    sb_poly_make_linear(&poly, 5, 20, 10);
    TEST_ASSERT_EQUAL(0, sb_poly_get_extrema(&poly, &result));
    TEST_ASSERT_EQUAL(18, result.min);
    TEST_ASSERT_EQUAL(20, result.max);

    sb_poly_make_linear(&poly, 5, 15, 15);
    TEST_ASSERT_EQUAL(0, sb_poly_get_extrema(&poly, &result));
    TEST_ASSERT_EQUAL(15, result.min);
    TEST_ASSERT_EQUAL(15, result.max);

    sb_poly_make(&poly, linear, sizeof(linear) / sizeof(linear[0]));
    TEST_ASSERT_EQUAL(0, sb_poly_get_extrema(&poly, &result));
    TEST_ASSERT_EQUAL(8, result.min);
    TEST_ASSERT_EQUAL(10, result.max);

    /* quadratic */

    sb_poly_make(&poly, quadratic_convex, sizeof(quadratic_convex) / sizeof(quadratic_convex[0]));
    TEST_ASSERT_EQUAL(0, sb_poly_get_extrema(&poly, &result));
    TEST_ASSERT_EQUAL(4, result.min);
    TEST_ASSERT_EQUAL(7, result.max);

    sb_poly_make(&poly, quadratic_convex_2, sizeof(quadratic_convex_2) / sizeof(quadratic_convex_2[0]));
    TEST_ASSERT_EQUAL(0, sb_poly_get_extrema(&poly, &result));
    TEST_ASSERT_EQUAL(2, result.min);
    TEST_ASSERT_EQUAL(41 / 16.0f, result.max);

    sb_poly_make(&poly, quadratic_concave, sizeof(quadratic_concave) / sizeof(quadratic_concave[0]));
    TEST_ASSERT_EQUAL(0, sb_poly_get_extrema(&poly, &result));
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 55 / 16.0f, result.min);
    TEST_ASSERT_EQUAL(4, result.max);
}

void test_stretch(void)
{
    sb_poly_t poly;
    sb_poly_t poly2;
    float xs[4] = { 0, 7, 13, 61 };

    sb_poly_make_bezier(&poly, 10, xs, sizeof(xs) / sizeof(xs[0]));

    poly2 = poly;
    sb_poly_stretch(&poly2, 5);

    for (int i = 0; i <= 10; i++) {
        TEST_ASSERT_FLOAT_WITHIN(1e-3, sb_poly_eval(&poly, i), sb_poly_eval(&poly2, i * 5));
    }
}

void test_deriv(void)
{
    sb_poly_t poly;
    float xs[4] = { 0, 7, 13, 61 };
    float xs1[3] = { 7, 26, 183 };
    float xs2[2] = { 26, 366 };
    float xs3[1] = { 366 };
    float xs4[1] = { 0 };

    sb_poly_make(&poly, xs, 4);

    TEST_ASSERT_EQUAL(3, sb_poly_get_degree(&poly));

    sb_poly_deriv(&poly);
    TEST_ASSERT_EQUAL(2, sb_poly_get_degree(&poly));
    TEST_ASSERT_EQUAL_FLOAT_ARRAY(xs1, poly.coeffs, 3);

    sb_poly_deriv(&poly);
    TEST_ASSERT_EQUAL(1, sb_poly_get_degree(&poly));
    TEST_ASSERT_EQUAL_FLOAT_ARRAY(xs2, poly.coeffs, 2);

    sb_poly_deriv(&poly);
    TEST_ASSERT_EQUAL(0, sb_poly_get_degree(&poly));
    TEST_ASSERT_EQUAL_FLOAT_ARRAY(xs3, poly.coeffs, 1);

    sb_poly_deriv(&poly);
    TEST_ASSERT_EQUAL(0, sb_poly_get_degree(&poly));
    TEST_ASSERT_EQUAL_FLOAT_ARRAY(xs4, poly.coeffs, 1);
}

void test_solve_simple(void)
{
    sb_poly_t poly;
    float xs[8];
    float roots[8];
    uint8_t num_roots;

    /* pathologic case */

    sb_poly_make(&poly, xs, 0);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_poly_solve(&poly, 0, roots, &num_roots));
    TEST_ASSERT_EQUAL(0, num_roots);

    sb_poly_make(&poly, xs, 0);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_poly_solve(&poly, 2, roots, &num_roots));
    TEST_ASSERT_EQUAL(0, num_roots);

    /* constants */

    sb_poly_make_zero(&poly);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_poly_solve(&poly, 0, roots, &num_roots));
    TEST_ASSERT_EQUAL(1, num_roots);
    TEST_ASSERT_EQUAL(0, roots[0]);

    sb_poly_make_constant(&poly, 2);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_poly_solve(&poly, 0, roots, &num_roots));
    TEST_ASSERT_EQUAL(0, num_roots);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_poly_solve(&poly, 2, roots, &num_roots));
    TEST_ASSERT_EQUAL(1, num_roots);
    TEST_ASSERT_EQUAL(0, roots[0]);

    /* linear */

    sb_poly_make_linear(&poly, 5, 10, 20);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_poly_solve(&poly, 0, roots, &num_roots));
    TEST_ASSERT_EQUAL(1, num_roots);
    TEST_ASSERT_FLOAT_WITHIN(1e-7, -5, roots[0]);

    sb_poly_make_linear(&poly, 5, 10, 20);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_poly_solve(&poly, 15, roots, &num_roots));
    TEST_ASSERT_EQUAL(1, num_roots);
    TEST_ASSERT_FLOAT_WITHIN(1e-7, 2.5, roots[0]);

    sb_poly_make_linear(&poly, 5, 10, 10);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_poly_solve(&poly, 0, roots, &num_roots));
    TEST_ASSERT_EQUAL(0, num_roots);

    sb_poly_make_linear(&poly, 5, 10, 10);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_poly_solve(&poly, 10, roots, &num_roots));
    TEST_ASSERT_EQUAL(1, num_roots);
    TEST_ASSERT_EQUAL(0, roots[0]);

    /* quadratic */

    xs[0] = 10;
    xs[1] = 2;
    xs[2] = 0;
    sb_poly_make(&poly, xs, 3);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_poly_solve(&poly, 0, roots, &num_roots));
    TEST_ASSERT_EQUAL(1, num_roots);
    TEST_ASSERT_FLOAT_WITHIN(1e-7, -5, roots[0]);

    xs[0] = 0;
    xs[1] = 2;
    xs[2] = 0;
    sb_poly_make(&poly, xs, 3);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_poly_solve(&poly, -10, roots, &num_roots));
    TEST_ASSERT_EQUAL(1, num_roots);
    TEST_ASSERT_FLOAT_WITHIN(1e-7, -5, roots[0]);

    xs[0] = 9;
    xs[1] = -6;
    xs[2] = 1;
    sb_poly_make(&poly, xs, 3);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_poly_solve(&poly, 0, roots, &num_roots));
    TEST_ASSERT_EQUAL(1, num_roots);
    TEST_ASSERT_FLOAT_WITHIN(1e-7, 3, roots[0]);

    xs[0] = 9;
    xs[1] = -6;
    xs[2] = 1;
    sb_poly_make(&poly, xs, 3);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_poly_solve(&poly, 4, roots, &num_roots));
    TEST_ASSERT_EQUAL(2, num_roots);
    qsort(roots, num_roots, sizeof(float), compare_floats);
    TEST_ASSERT_FLOAT_WITHIN(1e-7, 1, roots[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-7, 5, roots[1]);

    xs[0] = 5;
    xs[1] = -6;
    xs[2] = 1;
    sb_poly_make(&poly, xs, 3);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_poly_solve(&poly, 0, roots, &num_roots));
    TEST_ASSERT_EQUAL(2, num_roots);
    qsort(roots, num_roots, sizeof(float), compare_floats);
    TEST_ASSERT_FLOAT_WITHIN(1e-7, 1, roots[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-7, 5, roots[1]);

    xs[0] = 5;
    xs[1] = -6;
    xs[2] = 1;
    sb_poly_make(&poly, xs, 3);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_poly_solve(&poly, -4, roots, &num_roots));
    TEST_ASSERT_EQUAL(1, num_roots);
    TEST_ASSERT_FLOAT_WITHIN(1e-7, 3, roots[0]);

    xs[0] = 6;
    xs[1] = -3;
    xs[2] = 0;
    sb_poly_make(&poly, xs, 3);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_poly_solve(&poly, 0, roots, &num_roots));
    TEST_ASSERT_EQUAL(1, num_roots);
    TEST_ASSERT_FLOAT_WITHIN(1e-7, 2, roots[0]);

    xs[0] = 2;
    xs[1] = -1;
    xs[2] = -3;
    sb_poly_make(&poly, xs, 3);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_poly_solve(&poly, 0, roots, &num_roots));
    TEST_ASSERT_EQUAL(2, num_roots);
    qsort(roots, num_roots, sizeof(float), compare_floats);
    TEST_ASSERT_FLOAT_WITHIN(1e-7, -1, roots[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-7, 2 / 3.0f, roots[1]);
}

void test_solve_roots_not_needed(void)
{
    sb_poly_t poly;
    float xs[8];
    uint8_t num_roots;

    /* quadratic */

    xs[0] = 9;
    xs[1] = -6;
    xs[2] = 1;
    sb_poly_make(&poly, xs, 3);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_poly_solve(&poly, 0, /* roots = */ 0, &num_roots));
    TEST_ASSERT_EQUAL(1, num_roots);
}

void test_solve_root_count_not_needed(void)
{
    sb_poly_t poly;
    float xs[8];
    float roots[8];

    /* quadratic */

    xs[0] = 9;
    xs[1] = -6;
    xs[2] = 1;
    sb_poly_make(&poly, xs, 3);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_poly_solve(&poly, 0, roots, /* num_roots = */ 0));
    TEST_ASSERT_FLOAT_WITHIN(1e-7, 3, roots[0]);
}

void test_solve_generic(void)
{
    sb_poly_t poly;
    float xs[8];
    float roots[8];
    uint8_t num_roots;

    /* cubic */

    xs[0] = -5;
    xs[1] = 3;
    xs[2] = -3;
    xs[3] = 1;
    sb_poly_make(&poly, xs, 4);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_poly_solve(&poly, 0, roots, &num_roots));
    TEST_ASSERT_EQUAL(1, num_roots);
    TEST_ASSERT_FLOAT_WITHIN(2.5874, 1, roots[0]);

    xs[0] = 5;
    xs[1] = -6;
    xs[2] = 1;
    xs[3] = 0;
    sb_poly_make(&poly, xs, 4);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_poly_solve(&poly, 0, roots, &num_roots));
    TEST_ASSERT_EQUAL(2, num_roots);
    TEST_ASSERT_FLOAT_WITHIN(1e-7, 1, roots[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-7, 5, roots[1]);

    xs[0] = -27;
    xs[1] = 27;
    xs[2] = -9;
    xs[3] = 1;
    sb_poly_make(&poly, xs, 4);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_poly_solve(&poly, 0, roots, &num_roots));
    TEST_ASSERT_EQUAL(1, num_roots);
    TEST_ASSERT_FLOAT_WITHIN(1e-7, 3, roots[0]);

    xs[0] = -45;
    xs[1] = 39;
    xs[2] = -11;
    xs[3] = 1;
    sb_poly_make(&poly, xs, 4);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_poly_solve(&poly, 0, roots, &num_roots));
    TEST_ASSERT_EQUAL(2, num_roots);
    qsort(roots, num_roots, sizeof(float), compare_floats);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 3, roots[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 5, roots[1]);

    xs[0] = -15;
    xs[1] = 23;
    xs[2] = -9;
    xs[3] = 1;
    sb_poly_make(&poly, xs, 4);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_poly_solve(&poly, 0, roots, &num_roots));
    TEST_ASSERT_EQUAL(3, num_roots);
    qsort(roots, num_roots, sizeof(float), compare_floats);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 1, roots[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 3, roots[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 5, roots[2]);

    xs[0] = 0;
    xs[1] = 23;
    xs[2] = -9;
    xs[3] = 1;
    sb_poly_make(&poly, xs, 4);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_poly_solve(&poly, 15, roots, &num_roots));
    TEST_ASSERT_EQUAL(3, num_roots);
    qsort(roots, num_roots, sizeof(float), compare_floats);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 1, roots[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 3, roots[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 5, roots[2]);

    /* Casus irreducibilis example of the Cardano formula from Wikipedia */
    xs[0] = 3;
    xs[1] = -6;
    xs[2] = -9;
    xs[3] = 2;
    sb_poly_make(&poly, xs, 4);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_poly_solve(&poly, 0, roots, &num_roots));
    TEST_ASSERT_EQUAL(3, num_roots);
    qsort(roots, num_roots, sizeof(float), compare_floats);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, -0.876360f, roots[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.339843f, roots[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 5.036517f, roots[2]);

    /* Complicated cubic segment going through the following points:
     * (0, 5), (0.25, 6), (0.75, 3), (1, 7). */
    xs[0] = 5;
    xs[1] = 46 / 3.0f;
    xs[2] = -56;
    xs[3] = 128 / 3.0f;
    sb_poly_make(&poly, xs, 4);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_poly_solve(&poly, 2, roots, &num_roots));
    TEST_ASSERT_EQUAL(1, num_roots);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, -0.128953f, roots[0]);
}

void test_touches_simple(void)
{
    sb_poly_t poly;
    float xs[8];
    float result;

    /* Degenerate case: constant zero */
    sb_poly_make(&poly, xs, 0);

    TEST_ASSERT_TRUE(sb_poly_touches(&poly, 0, &result));
    TEST_ASSERT_EQUAL_FLOAT(0, result);

    TEST_ASSERT_TRUE(sb_poly_touches(&poly, 0, 0));

    result = 42;
    TEST_ASSERT_FALSE(sb_poly_touches(&poly, 1, &result));
    TEST_ASSERT_EQUAL_FLOAT(42, result); /* should be untouched */

    /* Constant nonzero */
    xs[0] = 12;
    sb_poly_make(&poly, xs, 1);

    TEST_ASSERT_FALSE(sb_poly_touches(&poly, 0, &result));
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, 12, &result));
    TEST_ASSERT_EQUAL_FLOAT(0, result);
    TEST_ASSERT_FALSE(sb_poly_touches(&poly, 24, &result));

    /* Linear segment from 1 to 3 */
    xs[0] = 1;
    xs[1] = 2;
    sb_poly_make(&poly, xs, 2);

    TEST_ASSERT_FALSE(sb_poly_touches(&poly, 0, &result));
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, 1, &result));
    TEST_ASSERT_EQUAL_FLOAT(0, result);
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, 2, &result));
    TEST_ASSERT_EQUAL_FLOAT(0.5f, result);
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, 3, &result));
    TEST_ASSERT_EQUAL_FLOAT(1, result);
    TEST_ASSERT_FALSE(sb_poly_touches(&poly, 4, &result));
    TEST_ASSERT_EQUAL_FLOAT(1, result); /* should be untouched */

    /* Linear segment from 3 to 1 */
    xs[0] = 3;
    xs[1] = -2;
    sb_poly_make(&poly, xs, 2);

    TEST_ASSERT_FALSE(sb_poly_touches(&poly, 0, &result));
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, 1, &result));
    TEST_ASSERT_EQUAL_FLOAT(1, result);
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, 2, &result));
    TEST_ASSERT_EQUAL_FLOAT(0.5f, result);
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, 3, &result));
    TEST_ASSERT_EQUAL_FLOAT(0, result);
    TEST_ASSERT_FALSE(sb_poly_touches(&poly, 4, &result));
    TEST_ASSERT_EQUAL_FLOAT(0, result); /* should be untouched */

    /* Convex quadratic segment raising from 1 to 6 */
    xs[0] = 1;
    xs[1] = 2;
    xs[2] = 3;
    sb_poly_make(&poly, xs, 3);
    TEST_ASSERT_FALSE(sb_poly_touches(&poly, 0, &result));
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, 1, &result));
    TEST_ASSERT_EQUAL_FLOAT(0, result);
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, 2, &result));
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 1 / 3.0f, result);
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, 3, &result));
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.548583f, result);
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, 4, &result));
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.720759f, result);
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, 5, &result));
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.868517f, result);
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, 6, &result));
    TEST_ASSERT_EQUAL_FLOAT(1, result);
    TEST_ASSERT_FALSE(sb_poly_touches(&poly, 7, &result));
    TEST_ASSERT_EQUAL_FLOAT(1, result); /* should be untouched */

    /* Concave quadratic segment falling from 2 to -2 */
    xs[0] = 2;
    xs[1] = -1;
    xs[2] = -3;
    sb_poly_make(&poly, xs, 3);
    TEST_ASSERT_FALSE(sb_poly_touches(&poly, 4, &result));
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, 2, &result));
    TEST_ASSERT_EQUAL_FLOAT(0, result);
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, 0, &result));
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 2 / 3.0f, result);
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, -2, &result));
    TEST_ASSERT_EQUAL_FLOAT(1, result);
    TEST_ASSERT_FALSE(sb_poly_touches(&poly, -4, &result));
    TEST_ASSERT_EQUAL_FLOAT(1, result); /* should be untouched */

    /* Concave quadratic segment raising from 1 to 5 and back to 1 */
    xs[0] = 1;
    xs[1] = 16;
    xs[2] = -16;
    sb_poly_make(&poly, xs, 3);
    TEST_ASSERT_FALSE(sb_poly_touches(&poly, -1, &result));
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, 1, &result));
    TEST_ASSERT_EQUAL_FLOAT(0, result);
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, 4, &result));
    TEST_ASSERT_EQUAL_FLOAT(0.25f, result);
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, 5, &result));
    TEST_ASSERT_EQUAL_FLOAT(0.5f, result);
    TEST_ASSERT_FALSE(sb_poly_touches(&poly, 7, &result));
    TEST_ASSERT_EQUAL_FLOAT(0.5f, result); /* should be untouched */

    /* Friendly cubic segment with no roots or critical points in the
     * [0; 1] interval */
    xs[0] = 1;
    xs[1] = 3;
    xs[2] = 3;
    xs[3] = 1;
    sb_poly_make(&poly, xs, 4);
    TEST_ASSERT_FALSE(sb_poly_touches(&poly, -1, &result));
    TEST_ASSERT_FALSE(sb_poly_touches(&poly, 0, &result));
    TEST_ASSERT_FALSE(sb_poly_touches(&poly, 0.9999f, &result));
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, 1, &result));
    TEST_ASSERT_EQUAL_FLOAT(0, result);
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, 3.375f, &result));
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.5f, result);
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, 8, &result));
    TEST_ASSERT_EQUAL_FLOAT(1, result);
    TEST_ASSERT_FALSE(sb_poly_touches(&poly, 8.00001, &result));
    TEST_ASSERT_EQUAL_FLOAT(1, result); /* should be untouched */

    /* Friendly cubic segment with no roots or critical points in the
     * [0; 1] interval where a < 0 */
    xs[0] = 1;
    xs[1] = -3;
    xs[2] = -3;
    xs[3] = -1;
    sb_poly_make(&poly, xs, 4);
    TEST_ASSERT_FALSE(sb_poly_touches(&poly, 3, &result));
    TEST_ASSERT_FALSE(sb_poly_touches(&poly, 2, &result));
    TEST_ASSERT_FALSE(sb_poly_touches(&poly, 1.00001f, &result));
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, 1, &result));
    TEST_ASSERT_EQUAL_FLOAT(0, result);
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, -1.375f, &result));
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.5f, result);
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, -6, &result));
    TEST_ASSERT_EQUAL_FLOAT(1, result);
    TEST_ASSERT_FALSE(sb_poly_touches(&poly, -6.00001, &result));
    TEST_ASSERT_EQUAL_FLOAT(1, result); /* should be untouched */

    /* Cubic segment where the first derivative is convex and has a critical
     * point in [0; 1] */
    xs[0] = 1;
    xs[1] = 3;
    xs[2] = -4;
    xs[3] = 2;
    sb_poly_make(&poly, xs, 4);
    TEST_ASSERT_FALSE(sb_poly_touches(&poly, -1, &result));
    TEST_ASSERT_FALSE(sb_poly_touches(&poly, 0, &result));
    TEST_ASSERT_FALSE(sb_poly_touches(&poly, 0.9999f, &result));
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, 1, &result));
    TEST_ASSERT_EQUAL_FLOAT(0, result);
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, 1.75f, &result));
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.5f, result);
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, 2, &result));
    TEST_ASSERT_EQUAL_FLOAT(1, result);
    TEST_ASSERT_FALSE(sb_poly_touches(&poly, 2.00001f, &result));
    TEST_ASSERT_EQUAL_FLOAT(1, result); /* should be untouched */

    /* Same as above but the first derivative is negated */
    xs[0] = 1;
    xs[1] = -3;
    xs[2] = 4;
    xs[3] = -2;
    sb_poly_make(&poly, xs, 4);
    TEST_ASSERT_FALSE(sb_poly_touches(&poly, -2, &result));
    TEST_ASSERT_FALSE(sb_poly_touches(&poly, -1, &result));
    TEST_ASSERT_FALSE(sb_poly_touches(&poly, -0.0001f, &result));
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, 0, &result));
    TEST_ASSERT_EQUAL_FLOAT(1, result);
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, 0.25f, &result));
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.5f, result);
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, 1, &result));
    TEST_ASSERT_EQUAL_FLOAT(0, result);
    TEST_ASSERT_FALSE(sb_poly_touches(&poly, 1.00001f, &result));
    TEST_ASSERT_EQUAL_FLOAT(0, result); /* should be untouched */

    /* Complicated cubic segment going through the following points:
     * (0, 5), (0.25, 6), (0.75, 3), (1, 7).
     * Some points may occur multiple times; for instance, the leftmost
     * intersection for y=3 is 0.657549858, not 0.75 */
    xs[0] = 5;
    xs[1] = 46 / 3.0f;
    xs[2] = -56;
    xs[3] = 128 / 3.0f;
    sb_poly_make(&poly, xs, 4);
    TEST_ASSERT_FALSE(sb_poly_touches(&poly, 2, &result));
    TEST_ASSERT_FALSE(sb_poly_touches(&poly, 2.9f, &result));
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, 2.93f, &result));
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.695873f, result);
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, 3, &result));
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.657549f, result);
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, 4, &result));
    TEST_ASSERT_EQUAL_FLOAT(0.5f, result);
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, 5, &result));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, result);
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, 6, &result));
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.097111f, result);
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, 7, &result));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, result);
    TEST_ASSERT_FALSE(sb_poly_touches(&poly, 7.1, &result));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, result); /* should be untouched */

    /* Another complicated cubic segment to cover the case of a < 0:
     * (0, 5), (0.25, 2), (0.75, 8), (1, 4) */
    xs[0] = 5;
    xs[1] = -33;
    xs[2] = 304 / 3.0f;
    xs[3] = -208 / 3.0f;
    sb_poly_make(&poly, xs, 4);
    TEST_ASSERT_FALSE(sb_poly_touches(&poly, 1, &result));
    TEST_ASSERT_FALSE(sb_poly_touches(&poly, 1.8f, &result));
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, 1.9f, &result));
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.198482f, result);
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, 2, &result));
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.165452f, result);
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, 3, &result));
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.078522f, result);
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, 4, &result));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, result);
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, 5, &result));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, result);
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, 6, &result));
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.552061f, result);
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, 7, &result));
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.622497f, result);
    TEST_ASSERT_TRUE(sb_poly_touches(&poly, 8, &result));
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.75f, result);
    TEST_ASSERT_FALSE(sb_poly_touches(&poly, 8.1, &result));
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.75f, result); /* should be untouched */
}

int main(int argc, char* argv[])
{
    UNITY_BEGIN();

    RUN_TEST(test_zero);
    RUN_TEST(test_zero_4d);
    RUN_TEST(test_constant);
    RUN_TEST(test_linear);
    RUN_TEST(test_linear_small_durations);
    RUN_TEST(test_bezier);

    RUN_TEST(test_add_constant);
    RUN_TEST(test_scale);
    RUN_TEST(test_get_degree);
    RUN_TEST(test_get_extrema);
    RUN_TEST(test_stretch);
    RUN_TEST(test_deriv);

    RUN_TEST(test_touches_simple);

    RUN_TEST(test_solve_simple);
    RUN_TEST(test_solve_roots_not_needed);
    RUN_TEST(test_solve_root_count_not_needed);
    RUN_TEST(test_solve_generic);

    return UNITY_END();
}
