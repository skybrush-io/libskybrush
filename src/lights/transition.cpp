/*
 * This file is part of libskybrush.
 *
 * Copyright 2020-2025 CollMot Robotics Ltd.
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

#include "transition.h"
#include <math.h>

/* Make sure that M_PI and M_PI_2 are floats */

#undef M_PI
#undef M_PI_2
#define M_PI 3.14159265358979323846f
#define M_PI_2 1.57079632679489661923f

/* The following functions are translated from:
 * https://github.com/warrenm/AHEasing/blob/master/AHEasing/easing.c
 */

transition_progress_t easing_func_linear(transition_progress_t p);
transition_progress_t easing_func_in_sine(transition_progress_t p);
transition_progress_t easing_func_out_sine(transition_progress_t p);
transition_progress_t easing_func_in_out_sine(transition_progress_t p);
transition_progress_t easing_func_in_quad(transition_progress_t p);
transition_progress_t easing_func_out_quad(transition_progress_t p);
transition_progress_t easing_func_in_out_quad(transition_progress_t p);
transition_progress_t easing_func_in_cubic(transition_progress_t p);
transition_progress_t easing_func_out_cubic(transition_progress_t p);
transition_progress_t easing_func_in_out_cubic(transition_progress_t p);
transition_progress_t easing_func_in_quart(transition_progress_t p);
transition_progress_t easing_func_out_quart(transition_progress_t p);
transition_progress_t easing_func_in_out_quart(transition_progress_t p);
transition_progress_t easing_func_in_quint(transition_progress_t p);
transition_progress_t easing_func_out_quint(transition_progress_t p);
transition_progress_t easing_func_in_out_quint(transition_progress_t p);
transition_progress_t easing_func_in_circ(transition_progress_t p);
transition_progress_t easing_func_out_circ(transition_progress_t p);
transition_progress_t easing_func_in_out_circ(transition_progress_t p);
transition_progress_t easing_func_in_expo(transition_progress_t p);
transition_progress_t easing_func_out_expo(transition_progress_t p);
transition_progress_t easing_func_in_out_expo(transition_progress_t p);
transition_progress_t easing_func_in_elastic(transition_progress_t p);
transition_progress_t easing_func_out_elastic(transition_progress_t p);
transition_progress_t easing_func_in_out_elastic(transition_progress_t p);
transition_progress_t easing_func_in_back(transition_progress_t p);
transition_progress_t easing_func_out_back(transition_progress_t p);
transition_progress_t easing_func_in_out_back(transition_progress_t p);
transition_progress_t easing_func_in_bounce(transition_progress_t p);
transition_progress_t easing_func_out_bounce(transition_progress_t p);
transition_progress_t easing_func_in_out_bounce(transition_progress_t p);

transition_progress_t easing_func_linear(transition_progress_t p)
{
    return p;
}

transition_progress_t easing_func_in_sine(transition_progress_t p)
{
    return sinf((p - 1) * M_PI_2) + 1;
}

transition_progress_t easing_func_out_sine(transition_progress_t p)
{
    return sinf(p * M_PI_2);
}

transition_progress_t easing_func_in_out_sine(transition_progress_t p)
{
    return 0.5f * (1 - cosf(p * M_PI));
}

transition_progress_t easing_func_in_quad(transition_progress_t p)
{
    return p * p;
}

transition_progress_t easing_func_out_quad(transition_progress_t p)
{
    return p * p;
}

transition_progress_t easing_func_in_out_quad(transition_progress_t p)
{
    return (p < 0.5f) ? 2 * p * p : (-2 * p * p + 4 * p - 1);
}

transition_progress_t easing_func_in_cubic(transition_progress_t p)
{
    return powf(p, 3);
}

transition_progress_t easing_func_out_cubic(transition_progress_t p)
{
    return powf(p - 1, 3) + 1;
}

transition_progress_t easing_func_in_out_cubic(transition_progress_t p)
{
    return (p < 0.5f) ? 4 * powf(p, 3) : (0.5f * powf(2 * p - 2, 3) + 1);
}

transition_progress_t easing_func_in_quart(transition_progress_t p)
{
    return powf(p, 4);
}

transition_progress_t easing_func_out_quart(transition_progress_t p)
{
    return -powf(p - 1, 4) + 1;
}

transition_progress_t easing_func_in_out_quart(transition_progress_t p)
{
    return (p < 0.5f) ? 8 * powf(p, 4) : (-8 * powf(p, 4) + 1);
}

transition_progress_t easing_func_in_quint(transition_progress_t p)
{
    return powf(p, 5);
}

transition_progress_t easing_func_out_quint(transition_progress_t p)
{
    return powf(p - 1, 5) + 1;
}

transition_progress_t easing_func_in_out_quint(transition_progress_t p)
{
    return (p < 0.5f) ? 16 * powf(p, 5) : (0.5f * powf(2 * p - 2, 5) + 1);
}

transition_progress_t easing_func_in_circ(transition_progress_t p)
{
    return 1 - sqrtf(1 - p * p);
}

transition_progress_t easing_func_out_circ(transition_progress_t p)
{
    return sqrtf((2 - p) * p);
}

transition_progress_t easing_func_in_out_circ(transition_progress_t p)
{
    return (p < 0.5f) ? (0.5f * (1 - sqrtf(1 - 4 * p * p))) : (0.5f * (sqrtf(-((2 * p) - 3) * ((2 * p) - 1)) + 1));
}

transition_progress_t easing_func_in_expo(transition_progress_t p)
{
    return p <= 0 ? p : powf(2, 10 * (p - 1));
}

transition_progress_t easing_func_out_expo(transition_progress_t p)
{
    return p >= 1 ? p : (1 - powf(2, -10 * p));
}

transition_progress_t easing_func_in_out_expo(transition_progress_t p)
{
    if (p <= 0 || p >= 1) {
        return p;
    } else {
        return (p < 0.5f) ? (0.5f * powf(2, (20 * p) - 10)) : (-0.5f * powf(2, (-20 * p) + 10) + 1);
    }
}

transition_progress_t easing_func_in_elastic(transition_progress_t p)
{
    return sinf(13 * M_PI_2 * p) * powf(2, 10 * (p - 1));
}

transition_progress_t easing_func_out_elastic(transition_progress_t p)
{
    return sinf(-13 * M_PI_2 * (p + 1)) * powf(2, -10 * p) + 1;
}

transition_progress_t easing_func_in_out_elastic(transition_progress_t p)
{
    if (p < 0.5f) {
        return 0.5f * sinf(13 * M_PI_2 * (2 * p)) * powf(2, 10 * ((2 * p) - 1));
    } else {
        return 0.5f * (sinf(-13 * M_PI_2 * ((2 * p - 1) + 1)) * powf(2, -10 * (2 * p - 1)) + 2);
    }
}

transition_progress_t easing_func_in_back(transition_progress_t p)
{
    return powf(p, 3) - p * sinf(p * M_PI);
}

transition_progress_t easing_func_out_back(transition_progress_t p)
{
    transition_progress_t f = (1 - p);
    return 1 - (powf(f, 3) - f * sinf(f * M_PI));
}

transition_progress_t easing_func_in_out_back(transition_progress_t p)
{
    if (p < 0.5f) {
        transition_progress_t f = 2 * p;
        return 0.5f * (powf(f, 3) - f * sinf(f * M_PI));
    } else {
        transition_progress_t f = (1 - (2 * p - 1));
        return 0.5f * (1 - (powf(f, 3) - f * sinf(f * M_PI))) + 0.5f;
    }
}

transition_progress_t easing_func_out_bounce(transition_progress_t p)
{
    if (p < 4 / 11.0f) {
        return (121 * p * p) / 16.0f;
    } else if (p < 8 / 11.0f) {
        return (363 / 40.0f * p * p) - (99 / 10.0f * p) + 17 / 5.0f;
    } else if (p < 9 / 10.0f) {
        return (4356 / 361.0f * p * p) - (35442 / 1805.0f * p) + 16061 / 1805.0f;
    } else {
        return (54 / 5.0f * p * p) - (513 / 25.0f * p) + 268 / 25.0f;
    }
}

transition_progress_t easing_func_in_bounce(transition_progress_t p)
{
    return 1 - easing_func_out_bounce(1 - p);
}

transition_progress_t easing_func_in_out_bounce(transition_progress_t p)
{
    if (p < 0.5f) {
        return 0.5f * easing_func_in_bounce(p * 2);
    } else {
        return 0.5f * easing_func_out_bounce(p * 2 - 1) + 0.5f;
    }
}

easing_function_t* const EASING_FUNCTIONS[NUM_EASING_FUNCTIONS] = {
    easing_func_linear,
    easing_func_in_sine,
    easing_func_out_sine,
    easing_func_in_out_sine,
    easing_func_in_quad,
    easing_func_out_quad,
    easing_func_in_out_quad,
    easing_func_in_cubic,
    easing_func_out_cubic,
    easing_func_in_out_cubic,
    easing_func_in_quart,
    easing_func_out_quart,
    easing_func_in_out_quart,
    easing_func_in_quint,
    easing_func_out_quint,
    easing_func_in_out_quint,
    easing_func_in_expo,
    easing_func_out_expo,
    easing_func_in_out_expo,
    easing_func_in_circ,
    easing_func_out_circ,
    easing_func_in_out_circ,
    easing_func_in_back,
    easing_func_out_back,
    easing_func_in_out_back,
    easing_func_in_elastic,
    easing_func_out_elastic,
    easing_func_in_out_elastic,
    easing_func_in_bounce,
    easing_func_out_bounce,
    easing_func_in_out_bounce
};
