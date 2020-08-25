#include <strings.h>
#include <skybrush/poly.h>

void sb_poly_make(sb_poly_t *poly, float *xs, uint8_t num_coeffs)
{
    bzero(poly, sizeof(sb_poly_t));
    poly->num_coeffs = (num_coeffs <= SB_MAX_POLY_COEFFS) ? num_coeffs : SB_MAX_POLY_COEFFS;
    memcpy(poly->coeffs, xs, poly->num_coeffs * sizeof(float));
}

void sb_poly_make_zero(sb_poly_t *poly)
{
    sb_poly_make_constant(poly, 0);
}

void sb_poly_make_constant(sb_poly_t *poly, float x)
{
    bzero(poly, sizeof(sb_poly_t));
    poly->coeffs[0] = x;
    poly->num_coeffs = 1;
}

void sb_poly_make_linear(sb_poly_t *poly, float duration, float x0, float x1)
{
    bzero(poly, sizeof(sb_poly_t));
    poly->num_coeffs = 2;

    if (duration != 0)
    {
        poly->coeffs[0] = x0;
        poly->coeffs[1] = (x1 - x0) / duration;
    }
    else
    {
        poly->coeffs[0] = (x0 + x1) / 2.0f;
    }
}

static const int facs[SB_MAX_POLY_COEFFS] = {1, 1, 2, 6, 24, 120, 720, 5040};

void sb_poly_make_bezier(sb_poly_t *poly, float duration, float *xs, uint8_t num_points)
{
    int i, j, n, sign;
    float coeff;

    if (num_points == 0)
    {
        sb_poly_make_zero(poly);
    }
    else if (num_points == 1)
    {
        sb_poly_make_constant(poly, xs[0]);
    }
    else if (num_points == 2)
    {
        sb_poly_make_linear(poly, duration, xs[0], xs[1]);
    }
    else
    {
        n = ((num_points < SB_MAX_POLY_COEFFS) ? num_points : SB_MAX_POLY_COEFFS) - 1;
        poly->num_coeffs = n + 1;

        sign = 1;
        for (j = 0; j <= n; j++)
        {
            coeff = 0;
            sign = (j % 2) ? -1 : 1;
            for (i = 0; i <= j; i++, sign *= -1)
            {
                coeff += sign * xs[i] / facs[i] / facs[j - i];
            }
            poly->coeffs[j] = coeff * facs[n] / facs[n - j];
        }

        sb_poly_stretch(poly, duration);
    }
}

void sb_poly_make_quadratic_bezier(sb_poly_t *poly, float duration, float u, float v, float w)
{
    float xs[] = {u, v, w};
    sb_poly_make_bezier(poly, duration, xs, sizeof(xs) / sizeof(float));
}

void sb_poly_make_cubic_bezier(sb_poly_t *poly, float duration, float u, float v, float w, float x)
{
    float xs[] = {u, v, w, x};
    sb_poly_make_bezier(poly, duration, xs, sizeof(xs) / sizeof(float));
}

float sb_poly_eval(const sb_poly_t *poly, float t)
{
    float result = 0.0f;
    const float *ptr = poly->coeffs + poly->num_coeffs;

    while (ptr > poly->coeffs)
    {
        result = result * t + (*(--ptr));
    }

    return result;
}

uint8_t sb_poly_get_degree(const sb_poly_t *poly)
{
    return poly->num_coeffs >= 1 ? poly->num_coeffs - 1 : 0;
}

void sb_poly_deriv(sb_poly_t *poly)
{
    if (poly->num_coeffs > 1)
    {
        for (uint8_t i = 1; i < poly->num_coeffs; i++)
        {
            poly->coeffs[i - 1] = i * poly->coeffs[i];
        }

        poly->num_coeffs--;
        poly->coeffs[poly->num_coeffs] = 0;
    }
    else
    {
        sb_poly_make_zero(poly);
    }
}

void sb_poly_scale(sb_poly_t *poly, float factor)
{
    for (uint8_t i = 0; i < poly->num_coeffs; i++)
    {
        poly->coeffs[i] *= factor;
    }
}

void sb_poly_stretch(sb_poly_t *poly, float factor)
{
    float scale;

    factor = 1.0f / factor;
    scale = factor;

    for (uint8_t i = 1; i < poly->num_coeffs; i++)
    {
        poly->coeffs[i] *= scale;
        scale *= factor;
    }
}

/* ************************************************************************* */

sb_vector3_with_yaw_t sb_poly_4d_eval(const sb_poly_4d_t *poly, float t)
{
    sb_vector3_with_yaw_t result;

    result.x = sb_poly_eval(&poly->x, t);
    result.y = sb_poly_eval(&poly->y, t);
    result.z = sb_poly_eval(&poly->z, t);
    result.yaw = sb_poly_eval(&poly->yaw, t);

    return result;
}

void sb_poly_4d_make_constant(sb_poly_4d_t *poly, sb_vector3_with_yaw_t vec)
{
    sb_poly_make_constant(&poly->x, vec.x);
    sb_poly_make_constant(&poly->y, vec.y);
    sb_poly_make_constant(&poly->z, vec.z);
    sb_poly_make_constant(&poly->yaw, vec.yaw);
}

void sb_poly_4d_make_zero(sb_poly_4d_t *poly)
{
    sb_poly_make_zero(&poly->x);
    sb_poly_make_zero(&poly->y);
    sb_poly_make_zero(&poly->z);
    sb_poly_make_zero(&poly->yaw);
}

void sb_poly_4d_deriv(sb_poly_4d_t *poly)
{
    sb_poly_deriv(&poly->x);
    sb_poly_deriv(&poly->y);
    sb_poly_deriv(&poly->z);
    sb_poly_deriv(&poly->yaw);
}

void sb_poly_4d_scale(sb_poly_4d_t *poly, float factor)
{
    sb_poly_scale(&poly->x, factor);
    sb_poly_scale(&poly->y, factor);
    sb_poly_scale(&poly->z, factor);
    sb_poly_scale(&poly->yaw, factor);
}