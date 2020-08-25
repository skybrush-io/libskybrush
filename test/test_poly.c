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

int main(int argc, char *argv[])
{
    UNITY_BEGIN();

    RUN_TEST(test_zero);
    RUN_TEST(test_constant);
    RUN_TEST(test_linear);
    RUN_TEST(test_bezier);

    RUN_TEST(test_scale);
    RUN_TEST(test_get_degree);
    RUN_TEST(test_stretch);
    RUN_TEST(test_deriv);

    return UNITY_END();
}