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

#include <complex.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include <skybrush/memory.h>
#include <skybrush/poly.h>

static uint8_t sb_i_poly_count_significant_coeffs(const sb_poly_t* poly);
static sb_error_t sb_i_poly_solve_1d(const sb_poly_t* poly, float rhs, float* roots, uint8_t* num_roots);
static sb_error_t sb_i_poly_solve_2d(const sb_poly_t* poly, float rhs, float* roots, uint8_t* num_roots);
static sb_error_t sb_i_poly_solve_3d(const sb_poly_t* poly, float rhs, float* roots, uint8_t* num_roots);
static sb_error_t sb_i_poly_solve_4d(const sb_poly_t* poly, float rhs, float* roots, uint8_t* num_roots);
static sb_error_t sb_i_poly_solve_generic(const sb_poly_t* poly, float rhs, float* roots, uint8_t* num_roots);
static sb_bool_t sb_i_poly_touches_1d(const sb_poly_t* poly, float value, float* result);
static sb_bool_t sb_i_poly_touches_2d(const sb_poly_t* poly, float value, float* result);
static sb_bool_t sb_i_poly_touches_3d(const sb_poly_t* poly, float value, float* result);
static sb_bool_t sb_i_poly_touches_4d(const sb_poly_t* poly, float value, float* result);
static sb_bool_t sb_i_poly_touches_generic(const sb_poly_t* poly, float value, float* result);

#define IS_ZERO(x) fabsf(x) < FLT_MIN

void sb_poly_make(sb_poly_t* poly, float* xs, uint8_t num_coeffs)
{
    memset(poly, 0, sizeof(sb_poly_t));
    poly->num_coeffs = (num_coeffs <= SB_MAX_POLY_COEFFS) ? num_coeffs : SB_MAX_POLY_COEFFS;
    memcpy(poly->coeffs, xs, poly->num_coeffs * sizeof(float));
}

void sb_poly_make_zero(sb_poly_t* poly)
{
    sb_poly_make_constant(poly, 0);
}

void sb_poly_make_constant(sb_poly_t* poly, float x)
{
    memset(poly, 0, sizeof(sb_poly_t));
    poly->coeffs[0] = x;
    poly->num_coeffs = 1;
}

void sb_poly_make_linear(sb_poly_t* poly, float duration, float x0, float x1)
{
    memset(poly, 0, sizeof(sb_poly_t));
    poly->num_coeffs = 2;

    if (fabsf(duration) >= FLT_EPSILON) {
        poly->coeffs[0] = x0;
        poly->coeffs[1] = (x1 - x0) / duration;
    } else {
        poly->coeffs[0] = (x0 + x1) / 2.0f;
    }
}

static const int facs[SB_MAX_POLY_COEFFS] = { 1, 1, 2, 6, 24, 120, 720, 5040 };

void sb_poly_make_bezier(sb_poly_t* poly, float duration, float* xs, uint8_t num_points)
{
    int i, j, n, sign;
    float coeff;

    if (num_points == 0) {
        sb_poly_make_zero(poly);
    } else if (num_points == 1) {
        sb_poly_make_constant(poly, xs[0]);
    } else if (num_points == 2) {
        sb_poly_make_linear(poly, duration, xs[0], xs[1]);
    } else {
        n = ((num_points < SB_MAX_POLY_COEFFS) ? num_points : SB_MAX_POLY_COEFFS) - 1;
        poly->num_coeffs = n + 1;

        for (j = 0; j <= n; j++) {
            coeff = 0;
            sign = (j % 2) ? -1 : 1;
            for (i = 0; i <= j; i++, sign *= -1) {
                coeff += sign * xs[i] / facs[i] / facs[j - i];
            }
            poly->coeffs[j] = coeff * facs[n] / facs[n - j];
        }

        sb_poly_stretch(poly, duration);
    }
}

void sb_poly_make_quadratic_bezier(sb_poly_t* poly, float duration, float u, float v, float w)
{
    float xs[] = { u, v, w };
    sb_poly_make_bezier(poly, duration, xs, sizeof(xs) / sizeof(float));
}

void sb_poly_make_cubic_bezier(sb_poly_t* poly, float duration, float u, float v, float w, float x)
{
    float xs[] = { u, v, w, x };
    sb_poly_make_bezier(poly, duration, xs, sizeof(xs) / sizeof(float));
}

float sb_poly_eval(const sb_poly_t* poly, float t)
{
    float result = 0.0f;
    const float* ptr = poly->coeffs + poly->num_coeffs;

    while (ptr > poly->coeffs) {
        result = result * t + (*(--ptr));
    }

    return result;
}

double sb_poly_eval_double(const sb_poly_t* poly, double t)
{
    double result = 0.0;
    const float* ptr = poly->coeffs + poly->num_coeffs;

    while (ptr > poly->coeffs) {
        result = result * t + ((double)*(--ptr));
    }

    return result;
}

uint8_t sb_poly_get_degree(const sb_poly_t* poly)
{
    return poly->num_coeffs >= 1 ? poly->num_coeffs - 1 : 0;
}

sb_error_t sb_poly_get_extrema(const sb_poly_t* poly, sb_interval_t* result)
{
    uint8_t coeffs;

    if (!result) {
        return SB_SUCCESS;
    }

    coeffs = poly->num_coeffs;
    while (coeffs > 2 && IS_ZERO(poly->coeffs[coeffs - 1])) {
        coeffs--;
    }

    switch (coeffs) {
    case 0:
        result->min = result->max = 0;
        break;

    case 1:
        result->min = result->max = poly->coeffs[0];
        break;

    case 2:
        if (poly->coeffs[1] > 0) {
            result->min = poly->coeffs[0];
            result->max = result->min + poly->coeffs[1];
        } else {
            result->max = poly->coeffs[0];
            result->min = result->max + poly->coeffs[1];
        }
        break;

    default: {
        float roots[4];

        if (coeffs <= sizeof(roots) / sizeof(roots[0])) {
            sb_poly_t deriv = *poly;
            uint8_t num_roots;

            sb_poly_deriv(&deriv);
            SB_CHECK(sb_poly_solve(&deriv, 0, roots, &num_roots));

            result->min = sb_poly_eval(poly, 0);
            result->max = sb_poly_eval(poly, 1);
            if (result->min > result->max) {
                float y = result->min;
                result->min = result->max;
                result->max = y;
            }

            while (num_roots > 0) {
                float y = roots[--num_roots];
                if (y >= 0 && y <= 1) {
                    y = sb_poly_eval(poly, y);
                    if (y < result->min) {
                        result->min = y;
                    }
                    if (y > result->max) {
                        result->max = y;
                    }
                }
            }
        }
    }
    }

    return SB_SUCCESS;
}

void sb_poly_deriv(sb_poly_t* poly)
{
    if (poly->num_coeffs > 1) {
        for (uint8_t i = 1; i < poly->num_coeffs; i++) {
            poly->coeffs[i - 1] = i * poly->coeffs[i];
        }

        poly->num_coeffs--;
        poly->coeffs[poly->num_coeffs] = 0;
    } else {
        sb_poly_make_zero(poly);
    }
}

void sb_poly_add_constant(sb_poly_t* poly, float constant)
{
    if (poly->num_coeffs == 0) {
        poly->num_coeffs = 1;
        poly->coeffs[0] = constant;
    } else {
        poly->coeffs[0] += constant;
    }
}

void sb_poly_scale(sb_poly_t* poly, float factor)
{
    for (uint8_t i = 0; i < poly->num_coeffs; i++) {
        poly->coeffs[i] *= factor;
    }
}

sb_error_t sb_poly_solve(const sb_poly_t* poly, float rhs, float* roots, uint8_t* num_roots)
{
    uint8_t dummy, num_significant_coeffs;
    sb_bool_t roots_allocated = 0;
    sb_error_t retval;

    if (!num_roots) {
        num_roots = &dummy;
    }

    num_significant_coeffs = sb_i_poly_count_significant_coeffs(poly);

    if (num_significant_coeffs == 0) {
        *num_roots = 0;
        return SB_SUCCESS;
    }

    if (!roots) {
        roots = sb_calloc(float, num_significant_coeffs);
        roots_allocated = 1;
    }

    switch (num_significant_coeffs) {
    case 1:
        retval = sb_i_poly_solve_1d(poly, rhs, roots, num_roots);
        break;

    case 2:
        retval = sb_i_poly_solve_2d(poly, rhs, roots, num_roots);
        break;

    case 3:
        retval = sb_i_poly_solve_3d(poly, rhs, roots, num_roots);
        break;

    case 4:
        retval = sb_i_poly_solve_4d(poly, rhs, roots, num_roots);
        break;

    default:
        retval = sb_i_poly_solve_generic(poly, rhs, roots, num_roots);
        break;
    }

    if (roots_allocated) {
        sb_free(roots);
    }

    return retval;
}

void sb_poly_stretch(sb_poly_t* poly, float factor)
{
    float scale;

    factor = 1.0f / factor;
    scale = factor;

    for (uint8_t i = 1; i < poly->num_coeffs; i++) {
        poly->coeffs[i] *= scale;
        scale *= factor;
    }
}

sb_bool_t sb_poly_touches(const sb_poly_t* poly, float value, float* result)
{
    uint8_t num_significant_coeffs = sb_i_poly_count_significant_coeffs(poly);
    float result_tmp;

    if (result == 0) {
        result = &result_tmp;
    }

    if (num_significant_coeffs < 2) {
        return sb_i_poly_touches_1d(poly, value, result);
    } else if (num_significant_coeffs == 2) {
        return sb_i_poly_touches_2d(poly, value, result);
    } else if (num_significant_coeffs == 3) {
        return sb_i_poly_touches_3d(poly, value, result);
    } else if (num_significant_coeffs == 4) {
        return sb_i_poly_touches_4d(poly, value, result);
    } else {
        return sb_i_poly_touches_generic(poly, value, result);
    }
}

/* ************************************************************************* */

static uint8_t sb_i_poly_count_significant_coeffs(const sb_poly_t* poly)
{
    uint8_t i;

    if (poly->num_coeffs > 0) {
        for (i = poly->num_coeffs; i > 1 && IS_ZERO(poly->coeffs[i - 1]); i--)
            ;
        return i;
    } else {
        return 0;
    }
}

static sb_bool_t sb_i_poly_touches_1d(const sb_poly_t* poly, float value, float* result)
{
    /* Polynomial is constant */
    if (value == poly->coeffs[0]) {
        *result = 0;
        return 1;
    } else {
        return 0;
    }
}

static sb_bool_t sb_i_poly_touches_2d(const sb_poly_t* poly, float value, float* result)
{
    /* Polynomial is linear */
    float a = poly->coeffs[1];
    float b = poly->coeffs[0];
    if (IS_ZERO(a)) {
        return sb_i_poly_touches_1d(poly, value, result); /* LCOV_EXCL_LINE */
    } else if (a > 0 && value >= b && value <= a + b) {
        *result = (value - b) / a;
        return 1;
    } else if (a < 0 && value >= a + b && value <= b) {
        *result = (value - b) / a;
        return 1;
    } else {
        return 0;
    }
}

static sb_bool_t sb_i_poly_touches_3d(const sb_poly_t* poly, float value, float* result)
{
    /* Polynomial is quadratic */
    float a = poly->coeffs[2];
    float b = poly->coeffs[1];
    float c = poly->coeffs[0];
    float value_at_zero = c;
    float value_at_one = a + b + c;
    float roots[2];
    float leftmost_root;
    uint8_t num_roots;

    /* Handle the easy cases first */
    if (value_at_zero == value) {
        *result = 0;
        return 1;
    }
    if (value_at_one == value) {
        *result = 1;
        return 1;
    }

    /* If the value at zero is already above the value we are looking for
     * _and_ the derivative is always non-negative in the [0; 1] interval,
     * there cannot be a touching point */
    if (value_at_zero > value && b >= 0 && a >= -b / 2) {
        return 0;
    }

    /* If the value at zero is already below the value we are looking for
     * _and_ the derivative is always non-positive in the [0; 1] interval,
     * there cannot be a touching point */
    if (value_at_zero < value && b <= 0 && a <= -b / 2) {
        return 0;
    }

    /* At this point we should probably just solve for value on the RHS */
    if (sb_poly_solve(poly, value, roots, &num_roots) == SB_SUCCESS) {
        leftmost_root = 2.0f;

        for (uint8_t i = 0; i < num_roots; i++) {
            if (roots[i] >= 0 && roots[i] <= 1 && roots[i] < leftmost_root) {
                leftmost_root = roots[i];
            }
        }

        if (leftmost_root < 2.0f) {
            *result = leftmost_root;
            return 1;
        }
    }

    return 0;
}

static sb_bool_t sb_i_poly_touches_4d(const sb_poly_t* poly, float value, float* result)
{
    /* Polynomial is cubic */
    float a = poly->coeffs[3];
    float b = poly->coeffs[2];
    float c = poly->coeffs[1];
    float d = poly->coeffs[0];
    float value_at_zero = d;
    float value_at_one = a + b + c + d;
    float roots[3];
    float leftmost_root;
    uint8_t num_roots;

    /* Handle the easy cases first */
    if (value_at_zero == value) {
        *result = 0;
        return 1;
    }
    if (value_at_one == value) {
        *result = 1;
        return 1;
    }

    /* If the value at zero is already above the value we are looking for
     * _and_ the derivative is always non-negative in the [0; 1] interval,
     * there cannot be a touching point */
    if (value_at_zero > value) {
        /* Minimum of 3*a*x*x + 2*b*x + c has to be >= 0 in the range [0;1] */
        /* The minimum is achieved at 0, 1 or where 6*a*x + 2*b = 3*a*x + b = 0,
         * i.e. x = -b / 3a.
         *
         * We know that f(0) = c, f(1) = 3*a + 2*b + c and
         * f(-b/3a) = c - (b*b)/(3*a). The latter matters only if -b/3a is
         * between 0 and 1, inclusive, and only if the function is convex
         * (i.e. a > 0 -- otherwise it is a maximum).
         *
         * -b / 3a >= 0 ==> b <= 0 so we are not interested in positive b values.
         * -b / 3a <= 1 ==> b >= -3a
         */
        if (c >= 0 && (3 * a + 2 * b + c) >= 0) {
            if (a <= 0 || b > 0 || b < -3 * a) {
                /* Function not convex or critical point is not in (0; 1) */
                return 0;
            }

            /* Critical point is in (0; 1), we need to check the minimum */
            if (c - (b * b) / (3 * a) >= 0) {
                return 0;
            }
        }
    }

    /* If the value at zero is already below the value we are looking for
     * _and_ the derivative is always non-positive in the [0; 1] interval,
     * there cannot be a touching point */
    if (value_at_zero < value) {
        /* Maximum of 3*a*x*x + 2*b*x + c has to be <= 0 in the range [0;1] */
        /* Same as above but with opposite signs */
        if (c <= 0 && (3 * a + 2 * b + c) <= 0) {
            if (a >= 0 || b < 0 || b > -3 * a) {
                /* Function is not concave or critical point is not in [0; 1] */
                return 0;
            }

            /* Critical point is in [0; 1], we need to check the minimum */
            if (c - (b * b) / (3 * a) <= 0) {
                return 0;
            }
        }
    }

    /* At this point we should probably just solve for value on the RHS */
    if (sb_poly_solve(poly, value, roots, &num_roots) == SB_SUCCESS) {
        leftmost_root = 2.0f;

        for (uint8_t i = 0; i < num_roots; i++) {
            if (roots[i] >= 0 && roots[i] <= 1 && roots[i] < leftmost_root) {
                leftmost_root = roots[i];
            }
        }

        if (leftmost_root < 2.0f) {
            *result = leftmost_root;
            return 1;
        }
    }

    return 0;
}

static sb_bool_t sb_i_poly_touches_generic(const sb_poly_t* poly, float value, float* result)
{
    /* TODO(ntamas) */
    return 0;
}

static sb_error_t sb_i_poly_solve_1d(const sb_poly_t* poly, float rhs, float* roots, uint8_t* num_roots)
{
    /* Polynomial is constant but not necessarily zero */
    if (IS_ZERO(poly->coeffs[0] - rhs)) {
        roots[0] = 0;
        *num_roots = 1;
    } else {
        *num_roots = 0;
    }
    return SB_SUCCESS;
}

static sb_error_t sb_i_poly_solve_2d(const sb_poly_t* poly, float rhs, float* roots, uint8_t* num_roots)
{
    float a = poly->coeffs[1];
    float b = poly->coeffs[0] - rhs;

    if (IS_ZERO(a)) {
        *num_roots = 0; /* LCOV_EXCL_LINE */
    } else {
        roots[0] = -b / a;
        *num_roots = 1;
    }

    return SB_SUCCESS;
}

static sb_error_t sb_i_poly_solve_3d(const sb_poly_t* poly, float rhs, float* roots, uint8_t* num_roots)
{
    /* Polynomial is quadratic */
    float a = poly->coeffs[2];
    float b = poly->coeffs[1];
    float c = poly->coeffs[0] - rhs;
    float d;

    if (IS_ZERO(a)) {
        return sb_i_poly_solve_2d(poly, rhs, roots, num_roots); /* LCOV_EXCL_LINE */
    }

    d = (b * b - 4 * a * c);

    if (IS_ZERO(d)) {
        *num_roots = 1;
        roots[0] = -b / (2 * a);
    } else if (d > 0) {
        *num_roots = 2;
        d = sqrtf(d);
        roots[0] = (-b - d) / (2 * a);
        roots[1] = (-b + d) / (2 * a);
    } else {
        *num_roots = 0;
    }

    return SB_SUCCESS;
}

static sb_error_t sb_i_poly_solve_4d(const sb_poly_t* poly, float rhs, float* roots, uint8_t* num_roots)
{
    /* Polynomial is cubic */
    float a = poly->coeffs[3];
    float b = poly->coeffs[2];
    float c = poly->coeffs[1];
    float d = poly->coeffs[0] - rhs;

    if (IS_ZERO(a)) {
        return sb_i_poly_solve_3d(poly, rhs, roots, num_roots); /* LCOV_EXCL_LINE */
    }

    /* We are solving the equation ax^3 + bx^2 + cx + d = 0 */
    float p = (3 * a * c - b * b) / (3 * a * a);
    float q = (2 * b * b * b - 9 * a * b * c + 27 * a * a * d) / (27 * a * a * a);
    float delta = (q * q) / 4 + (p * p * p) / 27;
    float offset = -b / (3 * a);

    if (fabsf(delta) < 1e-8f) {
        float u = cbrtf(-q / 2);

        /* one or two real roots */
        if (u == 0) {
            *num_roots = 1;
            roots[0] = 0;
        } else {
            *num_roots = 2;
            roots[0] = 2 * u;
            roots[1] = -u;
        }
    } else {
        float complex u, v;
        float complex sqrt_delta;

        if (delta > 0) {
            /* one real and two complex roots */
            *num_roots = 1;
            roots[0] = cbrtf(-q / 2 + sqrtf(delta)) + cbrtf(-q / 2 - sqrtf(delta));
        } else {
            /* three distinct real roots */
            sqrt_delta = csqrtf(delta);
            u = cpowf(-q / 2 + sqrt_delta, 1 / 3.0f);
            v = cpowf(-q / 2 - sqrt_delta, 1 / 3.0f);
            *num_roots = 3;
            roots[0] = crealf(u + v);
            roots[1] = crealf(-(u + v) / 2.0f + I * 0.5f * sqrtf(3) * (u - v));
            roots[2] = crealf(-(u + v) / 2.0f - I * 0.5f * sqrtf(3) * (u - v));
        }
    }

    roots[0] += offset;
    roots[1] += offset;
    roots[2] += offset;

    return SB_SUCCESS;
}

/* LCOV_EXCL_START */
static sb_error_t sb_i_poly_solve_generic(const sb_poly_t* poly, float rhs, float* roots, uint8_t* num_roots)
{
    return SB_EUNIMPLEMENTED;
}
/* LCOV_EXCL_STOP */

#undef ZERO

/* ************************************************************************* */

sb_vector3_with_yaw_t sb_poly_4d_eval(const sb_poly_4d_t* poly, float t)
{
    sb_vector3_with_yaw_t result;

    result.x = sb_poly_eval(&poly->x, t);
    result.y = sb_poly_eval(&poly->y, t);
    result.z = sb_poly_eval(&poly->z, t);
    result.yaw = sb_poly_eval(&poly->yaw, t);

    return result;
}

void sb_poly_4d_make_constant(sb_poly_4d_t* poly, sb_vector3_with_yaw_t vec)
{
    sb_poly_make_constant(&poly->x, vec.x);
    sb_poly_make_constant(&poly->y, vec.y);
    sb_poly_make_constant(&poly->z, vec.z);
    sb_poly_make_constant(&poly->yaw, vec.yaw);
}

void sb_poly_4d_make_zero(sb_poly_4d_t* poly)
{
    sb_poly_make_zero(&poly->x);
    sb_poly_make_zero(&poly->y);
    sb_poly_make_zero(&poly->z);
    sb_poly_make_zero(&poly->yaw);
}

void sb_poly_4d_deriv(sb_poly_4d_t* poly)
{
    sb_poly_deriv(&poly->x);
    sb_poly_deriv(&poly->y);
    sb_poly_deriv(&poly->z);
    sb_poly_deriv(&poly->yaw);
}

void sb_poly_4d_scale(sb_poly_4d_t* poly, float factor)
{
    sb_poly_scale(&poly->x, factor);
    sb_poly_scale(&poly->y, factor);
    sb_poly_scale(&poly->z, factor);
    sb_poly_scale(&poly->yaw, factor);
}
