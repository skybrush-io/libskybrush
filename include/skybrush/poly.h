#ifndef SKYBRUSH_POLY_H
#define SKYBRUSH_POLY_H

#include <stdlib.h>

#include <skybrush/basic_types.h>
#include <skybrush/decls.h>

__BEGIN_DECLS

#define SB_MAX_POLY_DEGREE 7
#define SB_MAX_POLY_COEFFS 8

/**
 * Struct representing a one-dimensional polynomial of degree at most 7.
 */
typedef struct
{
    float coeffs[SB_MAX_POLY_COEFFS];
    uint8_t num_coeffs;
} sb_poly_t;

/**
 * Creates a polynomial from its coefficients.
 */
void sb_poly_make(sb_poly_t *poly, float *xs, uint8_t num_coeffs);

/**
 * Creates a constant zero polynomial.
 */
void sb_poly_make_zero(sb_poly_t *poly);

/**
 * Creates a constant polynomial.
 */
void sb_poly_make_constant(sb_poly_t *poly, float x);

/**
 * Creates a linear polynomial from p(0) = x0 to p(duration) = x1.
 */
void sb_poly_make_linear(sb_poly_t *poly, float duration, float x0, float x1);

/**
 * Creates a polynomial from Bezier control points.
 */
void sb_poly_make_bezier(sb_poly_t *poly, float duration, float *xs, uint8_t num_points);

/**
 * Creates a cubic polynomial from Bezier control points.
 */
void sb_poly_make_cubic_bezier(sb_poly_t *poly, float duration, float u, float v, float w, float x);

/**
 * Creates a quadratic polynomial from Bezier control points.
 */
void sb_poly_make_quadratic_bezier(sb_poly_t *poly, float duration, float u, float v, float w);

/**
 * Evaluates a polynomial using Horner's rule.
 */
float sb_poly_eval(const sb_poly_t *poly, float t);

/**
 * Returns the degree of a polynomial.
 */
uint8_t sb_poly_get_degree(const sb_poly_t *poly);

/**
 * Scales a polynomial in-place.
 */
void sb_poly_scale(sb_poly_t *poly, float factor);

/**
 * Computes the derivative of a polynomial in-place.
 */
void sb_poly_deriv(sb_poly_t *poly);

/**
 * Stretches the time dimension of a polynomial in-place.
 */
void sb_poly_stretch(sb_poly_t *poly, float factor);

/* ************************************************************************* */

/**
 * Struct representing a 4D polynomial along the X-Y-Z-yaw coordinates.
 */
typedef struct
{
    sb_poly_t x;
    sb_poly_t y;
    sb_poly_t z;
    sb_poly_t yaw;
} sb_poly_4d_t;

/**
 * Creates a constant 4D polynomial.
 */
void sb_poly_4d_make_constant(sb_poly_4d_t *poly, sb_vector3_with_yaw_t vec);

/**
 * Creates a constant zero 4D polynomial.
 */
void sb_poly_4d_make_zero(sb_poly_4d_t *poly);

/**
 * Evaluates a 4D polynomial and returns an x-y-z-yaw vector.
 */
sb_vector3_with_yaw_t sb_poly_4d_eval(const sb_poly_4d_t *poly, float t);

/**
 * Calculates the derivative of a 4D polynomial in-place.
 */
void sb_poly_4d_deriv(sb_poly_4d_t *poly);

/**
 * Scales a 4D polynomial in-place.
 */
void sb_poly_4d_scale(sb_poly_4d_t *poly, float factor);

__END_DECLS

#endif
