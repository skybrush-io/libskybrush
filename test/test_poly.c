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
}

void test_constant()
{
    sb_poly_t poly;

    sb_poly_make_constant(&poly, 3);

    TEST_ASSERT_EQUAL(0, sb_poly_get_degree(&poly));
    TEST_ASSERT_EQUAL(3, sb_poly_eval(&poly, 0));
    TEST_ASSERT_EQUAL(3, sb_poly_eval(&poly, -2));
    TEST_ASSERT_EQUAL(3, sb_poly_eval(&poly, 1));
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
}

void test_bezier()
{
    sb_poly_t poly;

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
    float xs[4] = {0, 7, 13, 61};

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

void test_stretch()
{
    sb_poly_t poly;
    sb_poly_t poly2;
    float xs[4] = {0, 7, 13, 61};

    sb_poly_make_bezier(&poly, 10, xs, sizeof(xs) / sizeof(xs[0]));

    poly2 = poly;
    sb_poly_stretch(&poly2, 5);

    for (int i = 0; i <= 10; i++)
    {
        TEST_ASSERT_FLOAT_WITHIN(1e-3, sb_poly_eval(&poly, i), sb_poly_eval(&poly2, i * 5));
    }
}

void test_deriv()
{
    sb_poly_t poly;
    float xs[4] = {0, 7, 13, 61};
    float xs1[3] = {7, 26, 183};
    float xs2[2] = {26, 366};
    float xs3[1] = {366};
    float xs4[1] = {0};

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

int main(int argc, char *argv[])
{
    UNITY_BEGIN();

    RUN_TEST(test_zero);
    RUN_TEST(test_constant);
    RUN_TEST(test_linear);
    RUN_TEST(test_bezier);

    RUN_TEST(test_add_constant);
    RUN_TEST(test_scale);
    RUN_TEST(test_get_degree);
    RUN_TEST(test_stretch);
    RUN_TEST(test_deriv);

    RUN_TEST(test_solve_simple);
    // RUN_TEST(test_solve_generic);

    return UNITY_END();
}