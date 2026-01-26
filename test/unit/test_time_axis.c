/*
 * This file is part of libskybrush.
 *
 * Copyright 2020-2026 CollMot Robotics Ltd.
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
#include <skybrush/time_axis.h>

#include "unity.h"

#define EPS 1e-6f

sb_time_axis_t axis;

void setUp(void)
{
    sb_time_axis_init(&axis);
}

void tearDown(void)
{
    sb_time_axis_destroy(&axis);
}

void test_init_empty(void)
{
    TEST_ASSERT_EQUAL(0, sb_time_axis_num_segments(&axis));
}

void test_initial_capacity(void)
{
    TEST_ASSERT_TRUE(sb_time_axis_capacity(&axis) >= 4);
}

void test_append_one_segment(void)
{
    sb_time_segment_t seg = sb_time_segment_make_realtime(1500);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_append_segment(&axis, seg));
    TEST_ASSERT_EQUAL(1, sb_time_axis_num_segments(&axis));

    /* verify stored values */
    sb_time_segment_t stored = axis.stor_begin[0];
    TEST_ASSERT_FLOAT_WITHIN(EPS, seg.duration_msec, stored.duration_msec);
    TEST_ASSERT_FLOAT_WITHIN(EPS, seg.initial_rate, stored.initial_rate);
    TEST_ASSERT_FLOAT_WITHIN(EPS, seg.final_rate, stored.final_rate);
}

void test_append_grows_capacity(void)
{
    /* Append more segments than the initial capacity to force growth */
    const int N = 10;
    for (int i = 0; i < N; ++i) {
        sb_time_segment_t seg = sb_time_segment_make_constant_rate((float)(i + 0.5f), 1.0f);
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_append_segment(&axis, seg));
    }

    TEST_ASSERT_EQUAL(N, sb_time_axis_num_segments(&axis));
    TEST_ASSERT_TRUE(sb_time_axis_capacity(&axis) >= N);
}

void test_clear_removes_all_entries(void)
{
    sb_time_segment_t seg = sb_time_segment_make_realtime(200);

    /* Append some entries */
    for (int i = 0; i < 5; ++i) {
        TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_append_segment(&axis, seg));
    }

    TEST_ASSERT_EQUAL(5, sb_time_axis_num_segments(&axis));

    /* Now clear and ensure entries are gone */
    sb_time_axis_clear(&axis);
    TEST_ASSERT_EQUAL(0, sb_time_axis_num_segments(&axis));
}

void test_get_segment_returns_ptr_and_values(void)
{
    /* Append a few different types of segments */
    sb_time_segment_t s0 = sb_time_segment_make_realtime(500);
    sb_time_segment_t s1 = sb_time_segment_make_constant_rate(1000, 2.0f);
    sb_time_segment_t s2 = sb_time_segment_make_slowdown_from(250, 2.0f);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_append_segment(&axis, s0));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_append_segment(&axis, s1));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_append_segment(&axis, s2));

    /* retrieve by index and verify pointer and contents */
    const sb_time_segment_t* p0 = sb_time_axis_get_segment(&axis, 0);
    const sb_time_segment_t* p1 = sb_time_axis_get_segment(&axis, 1);
    const sb_time_segment_t* p2 = sb_time_axis_get_segment(&axis, 2);

    TEST_ASSERT_NOT_NULL(p0);
    TEST_ASSERT_NOT_NULL(p1);
    TEST_ASSERT_NOT_NULL(p2);

    TEST_ASSERT_EQUAL_PTR(&axis.stor_begin[0], p0);
    TEST_ASSERT_EQUAL_PTR(&axis.stor_begin[1], p1);
    TEST_ASSERT_EQUAL_PTR(&axis.stor_begin[2], p2);

    TEST_ASSERT_FLOAT_WITHIN(EPS, s0.duration_msec, p0->duration_msec);
    TEST_ASSERT_FLOAT_WITHIN(EPS, s1.initial_rate, p1->initial_rate);
    TEST_ASSERT_FLOAT_WITHIN(EPS, s2.final_rate, p2->final_rate);
}

void test_get_segment_out_of_bounds_returns_null(void)
{
    /* no segments yet */
    TEST_ASSERT_NULL(sb_time_axis_get_segment(&axis, 0));

    /* append one and check out-of-bounds */
    sb_time_segment_t s = sb_time_segment_make_realtime(100);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_append_segment(&axis, s));

    TEST_ASSERT_NULL(sb_time_axis_get_segment(&axis, 1));
    TEST_ASSERT_NULL(sb_time_axis_get_segment(&axis, 100));
}

/* Tests for sb_time_segment_get_duration_sec_in_warped_time() */

void test_time_segment_warped_duration_constant(void)
{
    sb_time_segment_t s = sb_time_segment_make_constant_rate(2000, 3.0f);
    float observed;

    observed = sb_time_segment_get_duration_in_warped_time_sec(&s);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 2.0f * 3.0f, observed);

    observed = sb_time_segment_get_duration_in_wall_clock_time_sec(&s);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 2.0f, observed);
}

void test_time_segment_warped_duration_spinup_and_slowdown(void)
{
    sb_time_segment_t s;
    float observed;

    /* spin up from 0 to 2 over 1 second => avg rate = 1.0 => warped = 1.0 */
    s = sb_time_segment_make_spinup_to(1000, 2.0f);
    observed = sb_time_segment_get_duration_in_warped_time_sec(&s);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 1.0f, observed);

    observed = sb_time_segment_get_duration_in_wall_clock_time_sec(&s);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 1.0f, observed);

    /* slow down from 1 to 0 over 1.5 seconds => avg rate = 0.5 => warped = 0.75 */
    s = sb_time_segment_make_slowdown_from(1500, 1.0f);
    observed = sb_time_segment_get_duration_in_warped_time_sec(&s);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.75f, observed);

    observed = sb_time_segment_get_duration_in_wall_clock_time_sec(&s);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 1.5f, observed);
}

void test_time_segment_warped_duration_realtime_cases(void)
{
    sb_time_segment_t s;
    float observed;

    /* real-time to zero: avg rate = 0.5 => warped = duration * 0.5 */
    s = sb_time_segment_make_slowdown_from_realtime(4000);
    observed = sb_time_segment_get_duration_in_warped_time_sec(&s);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 2.0f, observed);

    observed = sb_time_segment_get_duration_in_wall_clock_time_sec(&s);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 4.0f, observed);

    /* zero to real-time: avg rate = 0.5 => warped = duration * 0.5 */
    s = sb_time_segment_make_spinup_to_realtime(4000);
    observed = sb_time_segment_get_duration_in_warped_time_sec(&s);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 2.0f, observed);

    observed = sb_time_segment_get_duration_in_wall_clock_time_sec(&s);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 4.0f, observed);
}

/* Tests for sb_time_axis_map() */

/* Single constant-rate segment: mapping is linear with the rate */
void test_time_axis_map_single_constant_segment(void)
{
    sb_time_segment_t s = sb_time_segment_make_constant_rate(5000, 2.0f);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_append_segment(&axis, s));

    /* inside segment */
    float observed = sb_time_axis_map(&axis, 1500);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 3.0f, observed);

    /* at segment boundary */
    observed = sb_time_axis_map(&axis, 5000);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 10.0f, observed);

    /* beyond segment -- continues with final rate */
    observed = sb_time_axis_map(&axis, 7000);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 14.0f, observed);
}

/* Mapping across multiple segments (different constant rates) */
void test_time_axis_map_across_segments(void)
{
    /* first segment: duration 2s, rate 1 => warped 2s */
    sb_time_segment_t a = sb_time_segment_make_constant_rate(2000, 1.0f);
    /* second: duration 3s, rate 2 => warped 6s */
    sb_time_segment_t b = sb_time_segment_make_constant_rate(3000, 2.0f);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_append_segment(&axis, a));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_append_segment(&axis, b));

    /* time 2.5s -> 0.5s into second segment => warped = 2.0 + 0.5*2.0 = 3.0 */
    float observed = sb_time_axis_map(&axis, 2500);
    float expected = 2.0f + 0.5f * 2.0f;
    TEST_ASSERT_FLOAT_WITHIN(EPS, expected, observed);

    /* time 4.0s -> 2.0s into second segment => warped = 2.0 + 2.0*2.0 = 6.0 */
    observed = sb_time_axis_map(&axis, 4000);
    expected = 2.0f + 2.0f * 2.0f;
    TEST_ASSERT_FLOAT_WITHIN(EPS, expected, observed);
}

/* Mapping inside a linearly changing-rate segment */
void test_time_axis_map_linear_changing_rate(void)
{
    /* segment: duration 2s, initial rate 1, final rate 3
     * For t within [0,2], warped = initial*t + ((final-initial)/(2*d))*t^2
     */
    sb_time_segment_t s = sb_time_segment_make(2000, 1.0f, 3.0f);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_append_segment(&axis, s));

    uint32_t t_msec = 1000;
    float t = t_msec / 1000.0f;
    float observed = sb_time_axis_map(&axis, t_msec);
    float expected = 1.0f * t + 0.5f * t * t; /* = 1 + 0.5 = 1.5 */
    TEST_ASSERT_FLOAT_WITHIN(EPS, expected, observed);

    t_msec = 1500;
    t = t_msec / 1000.0f;
    observed = sb_time_axis_map(&axis, t_msec);
    expected = 1.0f * t + 0.5f * t * t; /* = 1 + 9/8 = 2.125 */
    TEST_ASSERT_FLOAT_WITHIN(EPS, expected, observed);
}

/* Tests for sb_time_axis_map_ex() derivative (rate) behavior */

/* Single constant-rate segment: mapping is linear with the rate */
void test_time_axis_map_ex_single_constant_segment(void)
{
    float rate = 0.0f;
    sb_time_segment_t s = sb_time_segment_make_constant_rate(5000, 2.0f);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_append_segment(&axis, s));

    /* inside segment */
    float observed = sb_time_axis_map_ex(&axis, 1500, &rate);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 3.0f, observed);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 2.0f, rate);

    /* at segment boundary */
    rate = 0.0f;
    observed = sb_time_axis_map_ex(&axis, 5000, &rate);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 10.0f, observed);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 2.0f, rate);

    /* beyond segment -- continues with final rate */
    rate = 0.0f;
    observed = sb_time_axis_map_ex(&axis, 7000, &rate);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 14.0f, observed);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 2.0f, rate);
}

/* Mapping across multiple segments (different constant rates) */
void test_time_axis_map_ex_across_segments(void)
{
    float rate = 0.0f;

    /* first segment: duration 2s, rate 1 => warped 2s */
    sb_time_segment_t a = sb_time_segment_make_constant_rate(2000, 1.0f);
    /* second: duration 3s, rate 2 => warped 6s */
    sb_time_segment_t b = sb_time_segment_make_constant_rate(3000, 2.0f);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_append_segment(&axis, a));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_append_segment(&axis, b));

    /* time 2.5s -> 0.5s into second segment => warped = 2.0 + 0.5*2.0 = 3.0 */
    float observed = sb_time_axis_map_ex(&axis, 2500, &rate);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 3.0f, observed);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 2.0f, rate);

    /* time 4.0s -> 2.0s into second segment => warped = 2.0 + 2.0*2.0 = 6.0 */
    observed = sb_time_axis_map_ex(&axis, 4000, &rate);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 6.0f, observed);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 2.0f, rate);
}

/* Derivative for a linearly changing-rate segment should be:
 * rate(t) = initial_rate + (final_rate - initial_rate) * (t / duration)
 */
void test_time_axis_map_ex_linear_changing_rate(void)
{
    sb_time_segment_t s = sb_time_segment_make(2000, 1.0f, 3.0f);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_append_segment(&axis, s));

    float rate = -1.0f;
    sb_time_axis_map_ex(&axis, 1000, &rate);

    /* expected rate: 1 + (3-1) * (1/2) = 2.0 */
    TEST_ASSERT_FLOAT_WITHIN(EPS, 2.0f, rate);

    /* boundary checks */
    sb_time_axis_map_ex(&axis, 0.0f, &rate);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 1.0f, rate);

    sb_time_axis_map_ex(&axis, 2000, &rate);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 3.0f, rate);

    /* ensure the map result still matches the non-_ex variant when out_rate is NULL */
    rate = sb_time_axis_map_ex(&axis, 1500, NULL);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 2.625f, rate);
}

/* Test derivative behavior across a multi-segment scenario (three segments) */
void test_time_axis_map_ex_three_segment_scenario(void)
{
    /* Segments:
     * 1) realtime for 5s (rate = 1)
     * 2) slowdown from realtime to 0 over 5s (initial=1, final=0)
     * 3) realtime for 5s (rate = 1)
     */
    sb_time_segment_t s1 = sb_time_segment_make_realtime(5000);
    sb_time_segment_t s2 = sb_time_segment_make_slowdown_from_realtime(5000);
    sb_time_segment_t s3 = sb_time_segment_make_realtime(5000);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_append_segment(&axis, s1));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_append_segment(&axis, s2));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_append_segment(&axis, s3));

    float rate = -1.0f;

    /* t = 2 (inside first segment) -> rate should be 1.0 */
    sb_time_axis_map_ex(&axis, 2000, &rate);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 1.0f, rate);

    /* t = 7 (2s into seg2). seg2 duration = 5, relative_t = 2/5 = 0.4
     * seg2 initial=1 final=0 -> rate = 1 + (0-1)*0.4 = 0.6
     */
    sb_time_axis_map_ex(&axis, 7000, &rate);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.6f, rate);

    /* t = 10 (end of seg2, start of seg3, segments are open from the right) ->
     * rate should be 1.0 (rate of seg3) */
    sb_time_axis_map_ex(&axis, 10000, &rate);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 1.0f, rate);

    /* t = 12 (inside seg3) -> rate should be 1.0 */
    sb_time_axis_map_ex(&axis, 12000, &rate);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 1.0f, rate);

    /* Infinity: out_rate should be final rate of the last segment (1.0) */
    sb_time_axis_map_ex(&axis, UINT32_MAX, &rate);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 1.0f, rate);
}

/* Additional mapping scenario test:
 * Three segments:
 *  - realtime for 5s
 *  - slowdown from realtime to zero over 5s
 *  - realtime for 5s
 *
 * We check at least one point inside each segment and all boundaries.
 */
void test_time_axis_map_three_segment_scenario(void)
{
    /* Segments:
     * 1) realtime for 5s (rate = 1)
     * 2) slowdown from realtime to 0 over 5s (initial=1, final=0)
     * 3) realtime for 5s (rate = 1)
     */
    sb_time_segment_t s1 = sb_time_segment_make_realtime(5000);
    sb_time_segment_t s2 = sb_time_segment_make_slowdown_from_realtime(5000);
    sb_time_segment_t s3 = sb_time_segment_make_realtime(5000);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_append_segment(&axis, s1));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_append_segment(&axis, s2));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_append_segment(&axis, s3));

    /* Precompute expected accumulated warped durations */
    float warped_s1 = sb_time_segment_get_duration_in_warped_time_sec(&s1); /* 5 * 1 = 5 */
    float warped_s2 = sb_time_segment_get_duration_in_warped_time_sec(&s2); /* 5 * 0.5 = 2.5 */
    float warped_s3 = sb_time_segment_get_duration_in_warped_time_sec(&s3); /* 5 * 1 = 5 */

    /* t = 0 (start) */
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, sb_time_axis_map(&axis, 0));

    /* t = 2 (within first realtime segment) => warped = 2 */
    TEST_ASSERT_FLOAT_WITHIN(EPS, 2.0f, sb_time_axis_map(&axis, 2000));

    /* t = 5 (boundary between seg1 and seg2) => warped = warped_s1 = 5 */
    TEST_ASSERT_FLOAT_WITHIN(EPS, warped_s1, sb_time_axis_map(&axis, 5000));

    /* t = 7 (2s into seg2). For seg2: initial=1 final=0, d=5
     * warped_in_seg = initial*t_in + ((final-initial)/(2*d)) * t_in^2
     * = 1*2 + (-1/(10)) * 4 = 2 - 0.4 = 1.6
     * total = warped_s1 + 1.6 = 6.6
     */
    float t_in_seg2 = 2.0f;
    float warped_in_seg2 = t_in_seg2 - 0.1f * t_in_seg2 * t_in_seg2; /* 2 - 0.4 */
    float expected7 = warped_s1 + warped_in_seg2;
    TEST_ASSERT_FLOAT_WITHIN(EPS, expected7, sb_time_axis_map(&axis, 7000));

    /* t = 10 (end of seg2) => warped = warped_s1 + warped_s2 = 7.5 */
    TEST_ASSERT_FLOAT_WITHIN(EPS, warped_s1 + warped_s2, sb_time_axis_map(&axis, 10000));

    /* t = 12 (2s into seg3) => warped = warped_s1 + warped_s2 + 2*1 = 7.5 + 2 = 9.5 */
    TEST_ASSERT_FLOAT_WITHIN(EPS, warped_s1 + warped_s2 + 2.0f, sb_time_axis_map(&axis, 12000));

    /* t = 15 (end of seg3) => warped = sum = 5 + 2.5 + 5 = 12.5 */
    TEST_ASSERT_FLOAT_WITHIN(EPS, warped_s1 + warped_s2 + warped_s3, sb_time_axis_map(&axis, 15000));
}

/* Tests for insert/remove segment functions */

void test_insert_segment_at_positions(void)
{
    sb_time_segment_t a = sb_time_segment_make_spinup_to_realtime(100);
    sb_time_segment_t b = sb_time_segment_make_constant_rate(200, 2.0f);
    sb_time_segment_t c = sb_time_segment_make_constant_rate(300, 3.0f);

    /* append a and c */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_append_segment(&axis, a));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_append_segment(&axis, c));
    TEST_ASSERT_EQUAL(2, sb_time_axis_num_segments(&axis));

    /* insert b in the middle at index 1 */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_insert_segment_at(&axis, 1, b));
    TEST_ASSERT_EQUAL(3, sb_time_axis_num_segments(&axis));

    /* verify order a, b, c */
    const sb_time_segment_t* p0 = sb_time_axis_get_segment(&axis, 0);
    const sb_time_segment_t* p1 = sb_time_axis_get_segment(&axis, 1);
    const sb_time_segment_t* p2 = sb_time_axis_get_segment(&axis, 2);

    TEST_ASSERT_NOT_NULL(p0);
    TEST_ASSERT_NOT_NULL(p1);
    TEST_ASSERT_NOT_NULL(p2);

    TEST_ASSERT_FLOAT_WITHIN(EPS, a.duration_msec, p0->duration_msec);
    TEST_ASSERT_FLOAT_WITHIN(EPS, b.duration_msec, p1->duration_msec);
    TEST_ASSERT_FLOAT_WITHIN(EPS, c.duration_msec, p2->duration_msec);

    /* insert at beginning */
    sb_time_segment_t d = sb_time_segment_make_realtime(400);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_insert_segment_at(&axis, 0, d));
    TEST_ASSERT_EQUAL(4, sb_time_axis_num_segments(&axis));
    TEST_ASSERT_FLOAT_WITHIN(EPS, d.duration_msec, sb_time_axis_get_segment(&axis, 0)->duration_msec);

    /* insert at end (equivalent to append) */
    sb_time_segment_t e = sb_time_segment_make_realtime(500);
    size_t idx_end = sb_time_axis_num_segments(&axis);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_insert_segment_at(&axis, idx_end, e));
    TEST_ASSERT_FLOAT_WITHIN(EPS, e.duration_msec, sb_time_axis_get_segment(&axis, idx_end)->duration_msec);
}

void test_insert_segment_invalid_index(void)
{
    sb_time_segment_t s = sb_time_segment_make_realtime(100);
    /* empty axis, valid index 0 only */
    TEST_ASSERT_EQUAL(SB_EINVAL, sb_time_axis_insert_segment_at(&axis, 1, s));
}

void test_remove_segment_at_positions(void)
{
    sb_time_segment_t a = sb_time_segment_make_realtime(1000);
    sb_time_segment_t b = sb_time_segment_make_slowdown_from_realtime(2000);
    sb_time_segment_t c = sb_time_segment_make_constant_rate(3000, 3.0f);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_append_segment(&axis, a));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_append_segment(&axis, b));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_append_segment(&axis, c));
    TEST_ASSERT_EQUAL(3, sb_time_axis_num_segments(&axis));

    /* remove middle (index 1) */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_remove_segment_at(&axis, 1));
    TEST_ASSERT_EQUAL(2, sb_time_axis_num_segments(&axis));
    TEST_ASSERT_FLOAT_WITHIN(EPS, a.duration_msec, sb_time_axis_get_segment(&axis, 0)->duration_msec);
    TEST_ASSERT_FLOAT_WITHIN(EPS, c.duration_msec, sb_time_axis_get_segment(&axis, 1)->duration_msec);

    /* remove first */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_remove_segment_at(&axis, 0));
    TEST_ASSERT_EQUAL(1, sb_time_axis_num_segments(&axis));
    TEST_ASSERT_FLOAT_WITHIN(EPS, c.duration_msec, sb_time_axis_get_segment(&axis, 0)->duration_msec);

    /* remove last */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_remove_segment_at(&axis, 0));
    TEST_ASSERT_EQUAL(0, sb_time_axis_num_segments(&axis));
}

void test_remove_segment_invalid_index(void)
{
    /* empty axis */
    TEST_ASSERT_EQUAL(SB_EINVAL, sb_time_axis_remove_segment_at(&axis, 0));
}

void test_time_axis_swap_exchanges_contents(void)
{
    sb_time_axis_t axis_a;
    sb_time_axis_t axis_b;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_init(&axis_a));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_init(&axis_b));

    sb_time_segment_t seg_a = sb_time_segment_make_constant_rate(1000, 1.5f);
    sb_time_segment_t seg_b = sb_time_segment_make_spinup_to_realtime(2500);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_append_segment(&axis_a, seg_a));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_append_segment(&axis_b, seg_b));

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_set_origin_sec(&axis_a, 1.5f));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_set_origin_sec(&axis_b, 3.0f));

    sb_time_segment_t* stor_a = axis_a.stor_begin;
    sb_time_segment_t* stor_b = axis_b.stor_begin;

    sb_time_axis_swap(&axis_a, &axis_b);

    TEST_ASSERT_EQUAL_PTR(stor_b, axis_a.stor_begin);
    TEST_ASSERT_EQUAL_PTR(stor_a, axis_b.stor_begin);

    TEST_ASSERT_EQUAL(1, sb_time_axis_num_segments(&axis_a));
    TEST_ASSERT_EQUAL(1, sb_time_axis_num_segments(&axis_b));

    TEST_ASSERT_FLOAT_WITHIN(EPS, 3.0f, sb_time_axis_get_origin_sec(&axis_a));
    TEST_ASSERT_FLOAT_WITHIN(EPS, 1.5f, sb_time_axis_get_origin_sec(&axis_b));

    const sb_time_segment_t* seg_a_swapped = sb_time_axis_get_segment(&axis_a, 0);
    const sb_time_segment_t* seg_b_swapped = sb_time_axis_get_segment(&axis_b, 0);

    TEST_ASSERT_NOT_NULL(seg_a_swapped);
    TEST_ASSERT_NOT_NULL(seg_b_swapped);

    TEST_ASSERT_FLOAT_WITHIN(EPS, seg_b.duration_msec, seg_a_swapped->duration_msec);
    TEST_ASSERT_FLOAT_WITHIN(EPS, seg_b.initial_rate, seg_a_swapped->initial_rate);
    TEST_ASSERT_FLOAT_WITHIN(EPS, seg_b.final_rate, seg_a_swapped->final_rate);

    TEST_ASSERT_FLOAT_WITHIN(EPS, seg_a.duration_msec, seg_b_swapped->duration_msec);
    TEST_ASSERT_FLOAT_WITHIN(EPS, seg_a.initial_rate, seg_b_swapped->initial_rate);
    TEST_ASSERT_FLOAT_WITHIN(EPS, seg_a.final_rate, seg_b_swapped->final_rate);

    sb_time_axis_destroy(&axis_a);
    sb_time_axis_destroy(&axis_b);
}

/* Tests that exercise non-zero origin handling.
 *
 * The origin shifts the mapping such that warped time zero corresponds to
 * wall-clock time = origin. Times before the origin are treated as real-time
 * and are returned unchanged (with rate==1.0 in the _ex variant).
 */
void test_time_axis_map_origin_shift_constant_segment(void)
{
    /* single constant-rate segment at 2.0x */
    sb_time_segment_t s = sb_time_segment_make_constant_rate(5000, 2.0f);
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_append_segment(&axis, s));

    /* set origin to 10.0s (warped t = 0 corresponds to wall-clock 10s) */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_set_origin_sec(&axis, 10.0f));
    TEST_ASSERT_FLOAT_WITHIN(EPS, 10.0f, sb_time_axis_get_origin_sec(&axis));

    /* Query before origin: wall=9.0 -> relative = -1.0 -> should map unchanged and rate=1.0 */
    float rate = -123.0f;
    float observed = sb_time_axis_map_ex(&axis, 9000, &rate);
    TEST_ASSERT_FLOAT_WITHIN(EPS, -1.0f, observed);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 1.0f, rate);

    /* Query inside segment: wall=11.5 -> relative 1.5 -> warped = 1.5 * 2.0 = 3.0 */
    observed = sb_time_axis_map(&axis, 11500);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 3.0f, observed);

    /* At boundary: wall=15.0 -> relative 5.0 (end of segment) -> warped = 10.0 */
    rate = 0.0f;
    observed = sb_time_axis_map_ex(&axis, 15000, &rate);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 10.0f, observed);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 2.0f, rate);

    /* After the segment end: wall=17.0 -> relative 7.0 -> warped = 5*2 + (7-5)*final_rate(=2) = 14.0 */
    observed = sb_time_axis_map(&axis, 17000);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 14.0f, observed);
}

void test_time_axis_map_origin_shift_across_segments(void)
{
    /* first segment: duration 2s, rate 1 => warped 2s
     * second: duration 3s, rate 2 => warped 6s
     */
    sb_time_segment_t a = sb_time_segment_make_constant_rate(2000, 1.0f);
    sb_time_segment_t b = sb_time_segment_make_constant_rate(3000, 2.0f);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_append_segment(&axis, a));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_append_segment(&axis, b));

    /* shift origin so that wall-clock origin is 1.0s */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_time_axis_set_origin_sec(&axis, 1.0f));
    TEST_ASSERT_FLOAT_WITHIN(EPS, 1.0f, sb_time_axis_get_origin_sec(&axis));

    /* wall 3.5 -> relative = 2.5 -> 0.5s into second segment -> warped = 2.0 + 0.5*2.0 = 3.0 */
    float observed = sb_time_axis_map(&axis, 3500);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 3.0f, observed);

    /* wall 1.5 -> relative 0.5 -> inside first segment -> warped = 0.5*1.0 = 0.5 */
    observed = sb_time_axis_map(&axis, 1500);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.5f, observed);

    /* wall 0.5 -> before origin -> returned unchanged (relative = -0.5) */
    float rate = 0.0f;
    observed = sb_time_axis_map_ex(&axis, 500, &rate);
    TEST_ASSERT_FLOAT_WITHIN(EPS, -0.5f, observed);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 1.0f, rate);
}

int main(int argc, char* argv[])
{
    UNITY_BEGIN();

    /* time axis initialization */
    RUN_TEST(test_init_empty);
    RUN_TEST(test_initial_capacity);

    /* segment insertion and removal */
    RUN_TEST(test_clear_removes_all_entries);
    RUN_TEST(test_append_one_segment);
    RUN_TEST(test_append_grows_capacity);
    RUN_TEST(test_insert_segment_at_positions);
    RUN_TEST(test_insert_segment_invalid_index);
    RUN_TEST(test_remove_segment_at_positions);
    RUN_TEST(test_remove_segment_invalid_index);
    RUN_TEST(test_time_axis_swap_exchanges_contents);

    /* segment retrieval */
    RUN_TEST(test_get_segment_returns_ptr_and_values);
    RUN_TEST(test_get_segment_out_of_bounds_returns_null);

    /* warped-duration tests */
    RUN_TEST(test_time_segment_warped_duration_constant);
    RUN_TEST(test_time_segment_warped_duration_spinup_and_slowdown);
    RUN_TEST(test_time_segment_warped_duration_realtime_cases);

    /* sb_time_axis_map tests */
    RUN_TEST(test_time_axis_map_single_constant_segment);
    RUN_TEST(test_time_axis_map_across_segments);
    RUN_TEST(test_time_axis_map_linear_changing_rate);
    RUN_TEST(test_time_axis_map_three_segment_scenario);

    /* sb_time_axis_map_ex tests */
    RUN_TEST(test_time_axis_map_ex_single_constant_segment);
    RUN_TEST(test_time_axis_map_ex_across_segments);
    RUN_TEST(test_time_axis_map_ex_linear_changing_rate);
    RUN_TEST(test_time_axis_map_ex_three_segment_scenario);

    /* origin-shift tests */
    RUN_TEST(test_time_axis_map_origin_shift_constant_segment);
    RUN_TEST(test_time_axis_map_origin_shift_across_segments);

    return UNITY_END();
}
