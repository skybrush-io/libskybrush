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

/**
 * @file poly.h
 * @brief Handling of real-valued polynomials of finite order.
 *
 * @def SB_MAX_POLY_DEGREE
 * @brief The maximum degree of polynomials handled by this module.
 *
 * @def SB_MAX_POLY_COEFFS
 * @brief The maximum number of coefficients of a polynomial handled by this module.
 */
#ifndef SKYBRUSH_POLY_H
#define SKYBRUSH_POLY_H

#include <stdlib.h>

#include <skybrush/basic_types.h>
#include <skybrush/decls.h>
#include <skybrush/error.h>

__BEGIN_DECLS

#define SB_MAX_POLY_DEGREE 7
#define SB_MAX_POLY_COEFFS 8

/**
 * Struct representing a one-dimensional polynomial of degree at most 7.
 */
typedef struct
{
    /** The coefficients of the polynomial */
    float coeffs[SB_MAX_POLY_COEFFS];

    /** \brief The number of coefficients to consider from the \c coeffs array.
     *
     * Zero means an all-zero polynomial, 1 means a constant term, 2
     * means a linear segment, 3 means a quadratic polynomial and so on.
     */
    uint8_t num_coeffs;
} sb_poly_t;

/**
 * Creates a polynomial from its coefficients.
 */
void sb_poly_make(sb_poly_t* poly, float* xs, uint8_t num_coeffs);

/**
 * Creates a constant zero polynomial.
 */
void sb_poly_make_zero(sb_poly_t* poly);

/**
 * Creates a constant polynomial.
 */
void sb_poly_make_constant(sb_poly_t* poly, float x);

/**
 * Creates a linear polynomial from p(0) = x0 to p(duration) = x1.
 */
void sb_poly_make_linear(sb_poly_t* poly, float duration, float x0, float x1);

/**
 * Creates a polynomial from Bezier control points.
 */
void sb_poly_make_bezier(sb_poly_t* poly, float duration, float* xs, uint8_t num_points);

/**
 * Creates a cubic polynomial from Bezier control points.
 */
void sb_poly_make_cubic_bezier(sb_poly_t* poly, float duration, float u, float v, float w, float x);

/**
 * Creates a quadratic polynomial from Bezier control points.
 */
void sb_poly_make_quadratic_bezier(sb_poly_t* poly, float duration, float u, float v, float w);

/**
 * Evaluates a polynomial using Horner's rule.
 */
float sb_poly_eval(const sb_poly_t* poly, float t);

/**
 * Evaluates a polynomial using Horner's rule with double precision.
 */
double sb_poly_eval_double(const sb_poly_t* poly, double t);

/**
 * Finds the real roots of a polynomial.
 *
 * \param  poly  the polynomial to solve
 * \param  roots the real roots of the polynomial will be stored here; it must
 *         point to a memory area that is large enough to hold all roots
 * \param  num_roots  the actual number of roots found will be returned here
 * \return error code; \c SB_SUCCESS if the call was successful,
 *         \c SB_EUNIMPLEMENTED if polynomial solving is not implemented for the
 *         given degree
 */
sb_error_t sb_poly_solve(const sb_poly_t* poly, float* roots, uint8_t* num_roots);

/**
 * Returns the degree of a polynomial.
 */
uint8_t sb_poly_get_degree(const sb_poly_t* poly);

/**
 * Computes the minimum and maximum of a polynomial on the [0; 1] interval.
 *
 * Works if and only if the degree of the polynomial is at most 3; returns
 * \c SB_EUNIMPLEMENTED for higher-degree polynomials.
 */
sb_error_t sb_poly_get_extrema(const sb_poly_t* poly, sb_interval_t* result);

/**
 * Adds a constant to the polynomial in-place.
 */
void sb_poly_add_constant(sb_poly_t* poly, float constant);

/**
 * Scales a polynomial in-place.
 */
void sb_poly_scale(sb_poly_t* poly, float factor);

/**
 * Computes the derivative of a polynomial in-place.
 */
void sb_poly_deriv(sb_poly_t* poly);

/**
 * Stretches the time dimension of a polynomial in-place.
 */
void sb_poly_stretch(sb_poly_t* poly, float factor);

/* ************************************************************************* */

/**
 * Struct representing a 4D polynomial along the X-Y-Z-yaw coordinates.
 */
typedef struct
{
    /** The polynomial along the X coordinate */
    sb_poly_t x;

    /** The polynomial along the Y coordinate */
    sb_poly_t y;

    /** The polynomial along the Z coordinate */
    sb_poly_t z;

    /** The polynomial along the yaw coordinate */
    sb_poly_t yaw;
} sb_poly_4d_t;

/**
 * Creates a constant 4D polynomial.
 */
void sb_poly_4d_make_constant(sb_poly_4d_t* poly, sb_vector3_with_yaw_t vec);

/**
 * Creates a constant zero 4D polynomial.
 */
void sb_poly_4d_make_zero(sb_poly_4d_t* poly);

/**
 * Evaluates a 4D polynomial and returns an x-y-z-yaw vector.
 */
sb_vector3_with_yaw_t sb_poly_4d_eval(const sb_poly_4d_t* poly, float t);

/**
 * Calculates the derivative of a 4D polynomial in-place.
 */
void sb_poly_4d_deriv(sb_poly_4d_t* poly);

/**
 * Scales a 4D polynomial in-place.
 */
void sb_poly_4d_scale(sb_poly_4d_t* poly, float factor);

__END_DECLS

#endif
