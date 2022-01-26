/*
 * This file is part of libskybrush.
 *
 * Copyright 2020-2022 by libskybrush authors. See AUTHORS.
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

void setUp()
{
}

void tearDown()
{
}

void test_zero()
{
    sb_poly_t poly;

    sb_poly_make_zero(&poly);

    TEST_ASSERT_EQUAL(0, sb_poly_get_degree(&poly));

    TEST_ASSERT_EQUAL(0, sb_poly_eval(&poly, 0));
    TEST_ASSERT_EQUAL(0, sb_poly_eval(&poly, -2));
    TEST_ASSERT_EQUAL(0, sb_poly_eval(&poly, 1));

    TEST_ASSERT_EQUAL(0, sb_poly_eval_double(&poly, 0));
    TEST_ASSERT_EQUAL(0, sb_poly_eval_double(&poly, -2));
    TEST_ASSERT_EQUAL(0, sb_poly_eval_double(&poly, 1));
}

void test_constant()
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

void test_linear()
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

void test_linear_small_durations()
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

void test_bezier()
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

void test_add_constant()
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

void test_scale()
{
    sb_poly_t poly;

    sb_poly_make_linear(&poly, 5, 10, 20);
    sb_poly_scale(&poly, 3);

    TEST_ASSERT_EQUAL(24, sb_poly_eval(&poly, -1));
    TEST_ASSERT_EQUAL(30, sb_poly_eval(&poly, 0));
    TEST_ASSERT_EQUAL(48, sb_poly_eval(&poly, 3));
    TEST_ASSERT_EQUAL(60, sb_poly_eval(&poly, 5));
}

void test_get_degree()
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

void test_get_extrema()
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

void test_stretch()
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

void test_deriv()
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

void test_solve_simple()
{
    sb_poly_t poly;
    float xs[8];
    float roots[8];
    uint8_t num_roots;

    /* pathologic case */

    sb_poly_make(&poly, xs, 0);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_poly_solve(&poly, roots, &num_roots));
    TEST_ASSERT_EQUAL(0, num_roots);

    /* constants */

    sb_poly_make_zero(&poly);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_poly_solve(&poly, roots, &num_roots));
    TEST_ASSERT_EQUAL(1, num_roots);
    TEST_ASSERT_EQUAL(0, roots[0]);

    sb_poly_make_constant(&poly, 2);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_poly_solve(&poly, roots, &num_roots));
    TEST_ASSERT_EQUAL(0, num_roots);

    /* linear */

    sb_poly_make_linear(&poly, 5, 10, 20);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_poly_solve(&poly, roots, &num_roots));
    TEST_ASSERT_EQUAL(1, num_roots);
    TEST_ASSERT_FLOAT_WITHIN(1e-7, -5, roots[0]);

    sb_poly_make_linear(&poly, 5, 10, 10);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_poly_solve(&poly, roots, &num_roots));
    TEST_ASSERT_EQUAL(0, num_roots);

    /* quadratic */

    xs[0] = 10;
    xs[1] = 2;
    xs[2] = 0;
    sb_poly_make(&poly, xs, 3);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_poly_solve(&poly, roots, &num_roots));
    TEST_ASSERT_EQUAL(1, num_roots);
    TEST_ASSERT_FLOAT_WITHIN(1e-7, -5, roots[0]);

    xs[0] = 9;
    xs[1] = -6;
    xs[2] = 1;
    sb_poly_make(&poly, xs, 3);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_poly_solve(&poly, roots, &num_roots));
    TEST_ASSERT_EQUAL(1, num_roots);
    TEST_ASSERT_FLOAT_WITHIN(1e-7, 3, roots[0]);

    xs[0] = 5;
    xs[1] = -6;
    xs[2] = 1;
    sb_poly_make(&poly, xs, 3);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_poly_solve(&poly, roots, &num_roots));
    TEST_ASSERT_EQUAL(2, num_roots);
    TEST_ASSERT_FLOAT_WITHIN(1e-7, 1, roots[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-7, 5, roots[1]);

    xs[0] = 6;
    xs[1] = -3;
    xs[2] = 0;
    sb_poly_make(&poly, xs, 3);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_poly_solve(&poly, roots, &num_roots));
    TEST_ASSERT_EQUAL(1, num_roots);
    TEST_ASSERT_FLOAT_WITHIN(1e-7, 2, roots[0]);
}

void test_solve_roots_not_needed()
{
    sb_poly_t poly;
    float xs[8];
    uint8_t num_roots;

    /* quadratic */

    xs[0] = 9;
    xs[1] = -6;
    xs[2] = 1;
    sb_poly_make(&poly, xs, 3);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_poly_solve(&poly, /* roots = */ 0, &num_roots));
    TEST_ASSERT_EQUAL(1, num_roots);
}

void test_solve_root_count_not_needed()
{
    sb_poly_t poly;
    float xs[8];
    float roots[8];

    /* quadratic */

    xs[0] = 9;
    xs[1] = -6;
    xs[2] = 1;
    sb_poly_make(&poly, xs, 3);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_poly_solve(&poly, roots, /* num_roots = */ 0));
    TEST_ASSERT_FLOAT_WITHIN(1e-7, 3, roots[0]);
}

void test_solve_generic()
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
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_poly_solve(&poly, roots, &num_roots));
    TEST_ASSERT_EQUAL(1, num_roots);
    TEST_ASSERT_FLOAT_WITHIN(2.5874, 1, roots[0]);

    xs[0] = 5;
    xs[1] = -6;
    xs[2] = 1;
    xs[3] = 0;
    sb_poly_make(&poly, xs, 4);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_poly_solve(&poly, roots, &num_roots));
    TEST_ASSERT_EQUAL(2, num_roots);
    TEST_ASSERT_FLOAT_WITHIN(1e-7, 1, roots[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-7, 5, roots[1]);

    xs[0] = -27;
    xs[1] = 27;
    xs[2] = -9;
    xs[3] = 1;
    sb_poly_make(&poly, xs, 4);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_poly_solve(&poly, roots, &num_roots));
    TEST_ASSERT_EQUAL(1, num_roots);
    TEST_ASSERT_FLOAT_WITHIN(1e-7, 3, roots[0]);

    xs[0] = -45;
    xs[1] = 39;
    xs[2] = -11;
    xs[3] = 1;
    sb_poly_make(&poly, xs, 4);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_poly_solve(&poly, roots, &num_roots));
    TEST_ASSERT_EQUAL(2, num_roots);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 3, roots[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 5, roots[1]);

    xs[0] = -15;
    xs[1] = 23;
    xs[2] = -9;
    xs[3] = 1;
    sb_poly_make(&poly, xs, 4);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_poly_solve(&poly, roots, &num_roots));
    TEST_ASSERT_EQUAL(3, num_roots);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 1, roots[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 3, roots[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 5, roots[2]);
}

int main(int argc, char* argv[])
{
    UNITY_BEGIN();

    RUN_TEST(test_zero);
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

    RUN_TEST(test_solve_simple);
    RUN_TEST(test_solve_roots_not_needed);
    RUN_TEST(test_solve_root_count_not_needed);
    // RUN_TEST(test_solve_generic);

    return UNITY_END();
}
