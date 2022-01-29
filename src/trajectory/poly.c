/*
 * This file is part of libskybrush.
 *
 * Copyright 2020-2022 CollMot Robotics Ltd.
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
#include <math.h>
#include <stdio.h>
#include <string.h>

#include <skybrush/memory.h>
#include <skybrush/poly.h>

static sb_error_t sb_i_poly_solve_1d(const sb_poly_t* poly, float* roots, uint8_t* num_roots);
static sb_error_t sb_i_poly_solve_2d(const sb_poly_t* poly, float* roots, uint8_t* num_roots);
static sb_error_t sb_i_poly_solve_3d(const sb_poly_t* poly, float* roots, uint8_t* num_roots);
static sb_error_t sb_i_poly_solve_generic(const sb_poly_t* poly, float* roots, uint8_t* num_roots);

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
        result = result * t + (*(--ptr));
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
            SB_CHECK(sb_poly_solve(&deriv, roots, &num_roots));

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

sb_error_t sb_poly_solve(const sb_poly_t* poly, float* roots, uint8_t* num_roots)
{
    uint8_t dummy, num_significant_coeffs;
    sb_bool_t roots_allocated = 0;
    sb_error_t retval;

    if (!num_roots) {
        num_roots = &dummy;
    }

    if (poly->num_coeffs > 0) {
        for (
            num_significant_coeffs = poly->num_coeffs;
            num_significant_coeffs > 1 && IS_ZERO(poly->coeffs[num_significant_coeffs - 1]);
            num_significant_coeffs--)
            ;
    } else {
        num_significant_coeffs = 0;
    }

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
        retval = sb_i_poly_solve_1d(poly, roots, num_roots);
        break;

    case 2:
        retval = sb_i_poly_solve_2d(poly, roots, num_roots);
        break;

    case 3:
        retval = sb_i_poly_solve_3d(poly, roots, num_roots);
        break;

    default:
        retval = sb_i_poly_solve_generic(poly, roots, num_roots);
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

/* ************************************************************************* */

static sb_error_t sb_i_poly_solve_1d(const sb_poly_t* poly, float* roots, uint8_t* num_roots)
{
    if (IS_ZERO(poly->coeffs[0])) {
        roots[0] = 0;
        *num_roots = 1;
    } else {
        *num_roots = 0;
    }
    return SB_SUCCESS;
}

static sb_error_t sb_i_poly_solve_2d(const sb_poly_t* poly, float* roots, uint8_t* num_roots)
{
    float a = poly->coeffs[1];
    float b = poly->coeffs[0];

    if (IS_ZERO(a)) {
        *num_roots = 0; /* LCOV_EXCL_LINE */
    } else {
        roots[0] = -b / a;
        *num_roots = 1;
    }

    return SB_SUCCESS;
}

static sb_error_t sb_i_poly_solve_3d(const sb_poly_t* poly, float* roots, uint8_t* num_roots)
{
    float a = poly->coeffs[2];
    float b = poly->coeffs[1];
    float c = poly->coeffs[0];
    float d;

    if (IS_ZERO(a)) {
        return sb_i_poly_solve_2d(poly, roots, num_roots); /* LCOV_EXCL_LINE */
    }

    d = (b * b - 4 * a * c);

    if (IS_ZERO(d)) {
        *num_roots = 1;
        roots[0] = -b / (2 * a);
    } else {
        *num_roots = 2;
        d = sqrtf(d);
        roots[0] = (-b - d) / (2 * a);
        roots[1] = (-b + d) / (2 * a);
    }

    return SB_SUCCESS;
}

static sb_error_t sb_i_poly_solve_generic(const sb_poly_t* poly, float* roots, uint8_t* num_roots)
{
    return SB_EUNIMPLEMENTED;
}

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
