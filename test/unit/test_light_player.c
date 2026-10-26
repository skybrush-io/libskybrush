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

#include <skybrush/formats/binary.h>
#include <skybrush/lights.h>

#include "../src/lights/commands.h"
#include "unity.h"
#include "utils.h"

sb_light_program_t* program;
sb_light_player_t player;

sb_error_t loadFixture(const char* fname);
void loadBytecode(uint8_t* buf, size_t size);
void assertColorsAt(const unsigned long* t_ms, const sb_rgb_color_t* expected, int n);

/* Durations in the bytecode are expressed in units of 20 ms */
#define MS_500 25
#define MS_1000 50

void setUp(void)
{
    program = sb_light_program_new();
}

void tearDown(void)
{
    SB_XDECREF(program);
}

sb_error_t loadFixture(const char* fname)
{
    FILE* fp;
    int fd;
    sb_error_t retval;

    fp = fopen(fname, "rb");
    if (fp == 0) {
        abort();
    }

    fd = fileno(fp);
    if (fd < 0) {
        abort();
    }

    retval = sb_light_program_update_from_binary_file(program, fd);

    fclose(fp);

    return retval;
}

void loadBytecode(uint8_t* buf, size_t size)
{
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_light_program_update_from_buffer(program, buf, size));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_light_player_init(&player, program));
}

void assertColorsAt(const unsigned long* t_ms, const sb_rgb_color_t* expected, int n)
{
    int i;

    /* test querying forward */
    for (i = 0; i < n; i++) {
        TEST_ASSERT_EQUAL_COLOR(expected[i], sb_light_player_get_color_at(&player, t_ms[i]));
    }

    /* test querying backward; this rewinds the executor before each query */
    for (i = n - 1; i >= 0; i--) {
        TEST_ASSERT_EQUAL_COLOR(expected[i], sb_light_player_get_color_at(&player, t_ms[i]));
    }
}

void test_get_color_at(void)
{
    float t[] = { 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60 };
    sb_rgb_color_t actual;
    sb_rgb_color_t expected[] = {
        { 255, 255, 255 },
        { 255, 127, 127 },
        { 255, 0, 0 },
        { 255, 0, 0 },
        { 0, 255, 0 },
        { 0, 255, 0 },
        { 0, 0, 255 },
        { 0, 0, 255 },
        { 0, 0, 255 },
        { 127, 127, 255 },
        { 255, 255, 255 },
        { 255, 255, 255 },
        { 255, 255, 255 }
    };
    int i, j, n = sizeof(t) / sizeof(t[0]);
    const int random_order[] = { 12, 2, 5, 8, 11, 1, 4, 7, 10, 0, 3, 6, 9 };

    TEST_ASSERT_EQUAL(SB_SUCCESS, loadFixture("fixtures/test.skyb"));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_light_player_init(&player, program));

    /* test querying forward */
    for (i = 0; i < n; i++) {
        actual = sb_light_player_get_color_at(&player, t[i] * 1000);
        TEST_ASSERT_EQUAL_COLOR(expected[i], actual);
    }

    /* test querying backward */
    for (i = n - 1; i >= 0; i--) {
        actual = sb_light_player_get_color_at(&player, t[i] * 1000);
        TEST_ASSERT_EQUAL_COLOR(expected[i], actual);
    }

    /* test (pseudo)random access */
    for (j = 0; j < n; j++) {
        i = random_order[j];
        actual = sb_light_player_get_color_at(&player, t[i] * 1000);
        TEST_ASSERT_EQUAL_COLOR(expected[i], actual);
    }

    sb_light_player_destroy(&player);
}

void test_constant_color_program(void)
{
    sb_rgb_color_t expected = { 12, 34, 56 };
    sb_rgb_color_t actual;
    unsigned long t_ms[] = { 0, 500, 1000, 12345, 60000, 12345678 };
    int i, n = sizeof(t_ms) / sizeof(t_ms[0]);

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_light_program_set_constant_color(program, expected));
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_light_player_init(&player, program));

    for (i = 0; i < n; i++) {
        actual = sb_light_player_get_color_at(&player, t_ms[i]);
        TEST_ASSERT_EQUAL_COLOR(expected, actual);
    }

    sb_light_player_destroy(&player);
}

void test_empty_program(void)
{
    unsigned long next_timestamp;

    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_light_player_init(&player, program));

    TEST_ASSERT_EQUAL_COLOR(SB_COLOR_BLACK, sb_light_player_get_color_at(&player, 0));
    TEST_ASSERT_EQUAL_COLOR(SB_COLOR_BLACK, sb_light_player_get_color_at(&player, 5000));
    TEST_ASSERT_TRUE(sb_light_player_seek(&player, 1000, &next_timestamp));

    sb_light_player_destroy(&player);
}

void test_fade_commands(void)
{
    uint8_t bytecode[] = {
        CMD_SET_WHITE, MS_1000,
        CMD_NOP,
        CMD_FADE_TO_BLACK, MS_1000,
        CMD_FADE_TO_GRAY, 128, MS_1000,
        CMD_FADE_TO_WHITE, MS_1000,
        CMD_END
    };
    unsigned long t_ms[] = { 0, 500, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 5000 };
    sb_rgb_color_t expected[] = {
        { 255, 255, 255 },
        { 255, 255, 255 },
        { 255, 255, 255 },
        { 127, 127, 127 },
        { 0, 0, 0 },
        { 64, 64, 64 },
        { 128, 128, 128 },
        { 191, 191, 191 },
        { 255, 255, 255 },
        { 255, 255, 255 }
    };

    loadBytecode(bytecode, sizeof(bytecode));
    assertColorsAt(t_ms, expected, sizeof(t_ms) / sizeof(t_ms[0]));

    TEST_ASSERT_FALSE(sb_light_player_seek(&player, 3999, 0));
    TEST_ASSERT_TRUE(sb_light_player_seek(&player, 4000, 0));

    sb_light_player_destroy(&player);
}

void test_zero_duration_fade(void)
{
    uint8_t bytecode[] = {
        CMD_SET_COLOR, 255, 0, 0, MS_1000,
        CMD_FADE_TO_COLOR, 0, 255, 0, 0,
        CMD_SLEEP, MS_1000,
        CMD_END
    };
    unsigned long t_ms[] = { 0, 999, 1000, 1500 };
    sb_rgb_color_t expected[] = {
        { 255, 0, 0 },
        { 255, 0, 0 },
        { 0, 255, 0 },
        { 0, 255, 0 }
    };

    loadBytecode(bytecode, sizeof(bytecode));
    assertColorsAt(t_ms, expected, sizeof(t_ms) / sizeof(t_ms[0]));

    sb_light_player_destroy(&player);
}

void test_fade_after_zero_duration_fade(void)
{
    /* Regression test: a fade following a zero-duration fade used to start
     * from the color before the zero-duration fade */
    uint8_t bytecode[] = {
        CMD_SET_COLOR, 255, 0, 0, MS_1000,
        CMD_FADE_TO_COLOR, 0, 255, 0, 0,
        CMD_FADE_TO_BLACK, MS_1000,
        CMD_END
    };
    unsigned long t_ms[] = { 999, 1000, 1500, 2000 };
    sb_rgb_color_t expected[] = {
        { 255, 0, 0 },
        { 0, 255, 0 },
        { 0, 127, 0 },
        { 0, 0, 0 }
    };

    loadBytecode(bytecode, sizeof(bytecode));
    assertColorsAt(t_ms, expected, sizeof(t_ms) / sizeof(t_ms[0]));

    sb_light_player_destroy(&player);
}

void test_finite_loop(void)
{
    uint8_t bytecode[] = {
        CMD_LOOP_BEGIN, 3,
        CMD_SET_COLOR, 255, 0, 0, MS_500,
        CMD_SET_COLOR, 0, 0, 255, MS_500,
        CMD_LOOP_END,
        CMD_SET_COLOR, 0, 255, 0, MS_1000,
        CMD_END
    };
    unsigned long t_ms[] = { 250, 750, 1250, 1750, 2250, 2750, 3250, 3750, 5000 };
    sb_rgb_color_t expected[] = {
        { 255, 0, 0 },
        { 0, 0, 255 },
        { 255, 0, 0 },
        { 0, 0, 255 },
        { 255, 0, 0 },
        { 0, 0, 255 },
        { 0, 255, 0 },
        { 0, 255, 0 },
        { 0, 255, 0 }
    };
    unsigned long next_timestamp;

    loadBytecode(bytecode, sizeof(bytecode));
    assertColorsAt(t_ms, expected, sizeof(t_ms) / sizeof(t_ms[0]));

    TEST_ASSERT_FALSE(sb_light_player_seek(&player, 2250, &next_timestamp));
    TEST_ASSERT_EQUAL_UINT32(2500, next_timestamp);
    TEST_ASSERT_FALSE(sb_light_player_seek(&player, 3999, 0));
    TEST_ASSERT_TRUE(sb_light_player_seek(&player, 4000, 0));

    sb_light_player_destroy(&player);
}

void test_infinite_loop(void)
{
    uint8_t bytecode[] = {
        CMD_LOOP_BEGIN, 0,
        CMD_SET_COLOR, 255, 0, 0, MS_500,
        CMD_SET_COLOR, 0, 0, 255, MS_500,
        CMD_LOOP_END,
        CMD_END
    };
    unsigned long t_ms[] = { 250, 750, 100250, 100750 };
    sb_rgb_color_t expected[] = {
        { 255, 0, 0 },
        { 0, 0, 255 },
        { 255, 0, 0 },
        { 0, 0, 255 }
    };

    loadBytecode(bytecode, sizeof(bytecode));
    assertColorsAt(t_ms, expected, sizeof(t_ms) / sizeof(t_ms[0]));

    TEST_ASSERT_FALSE(sb_light_player_seek(&player, 1000000, 0));

    sb_light_player_destroy(&player);
}

void test_nested_loops(void)
{
    uint8_t bytecode[] = {
        CMD_LOOP_BEGIN, 2,
        CMD_LOOP_BEGIN, 2,
        CMD_SET_COLOR, 255, 0, 0, MS_500,
        CMD_LOOP_END,
        CMD_SET_COLOR, 0, 0, 255, MS_500,
        CMD_LOOP_END,
        CMD_END
    };
    unsigned long t_ms[] = { 250, 750, 1250, 1750, 2250, 2750, 3500 };
    sb_rgb_color_t expected[] = {
        { 255, 0, 0 },
        { 255, 0, 0 },
        { 0, 0, 255 },
        { 255, 0, 0 },
        { 255, 0, 0 },
        { 0, 0, 255 },
        { 0, 0, 255 }
    };

    loadBytecode(bytecode, sizeof(bytecode));
    assertColorsAt(t_ms, expected, sizeof(t_ms) / sizeof(t_ms[0]));

    TEST_ASSERT_FALSE(sb_light_player_seek(&player, 2999, 0));
    TEST_ASSERT_TRUE(sb_light_player_seek(&player, 3000, 0));

    sb_light_player_destroy(&player);
}

void test_zero_duration_commands_at_boundary(void)
{
    /* Regression test: when multiple zero-duration commands (loop markers,
     * NOPs) were due at the same timestamp, seeking exactly to that timestamp
     * used to execute only the first one */
    uint8_t bytecode[] = {
        CMD_LOOP_BEGIN, 2,
        CMD_NOP,
        CMD_SET_COLOR, 255, 0, 0, MS_500,
        CMD_SET_COLOR, 0, 0, 255, MS_500,
        CMD_LOOP_END,
        CMD_SET_COLOR, 0, 255, 0, MS_1000,
        CMD_NOP,
        CMD_NOP,
        CMD_END
    };

    loadBytecode(bytecode, sizeof(bytecode));

    TEST_ASSERT_EQUAL_COLOR_RGB(255, 0, 0, sb_light_player_get_color_at(&player, 0));
    TEST_ASSERT_EQUAL_COLOR_RGB(255, 0, 0, sb_light_player_get_color_at(&player, 1000));
    TEST_ASSERT_EQUAL_COLOR_RGB(0, 255, 0, sb_light_player_get_color_at(&player, 2000));
    TEST_ASSERT_TRUE(sb_light_player_seek(&player, 3000, 0));

    sb_light_player_destroy(&player);
}

void test_loop_end_without_loop_begin(void)
{
    /* A LOOP_END without a matching LOOP_BEGIN is a no-op */
    uint8_t bytecode[] = {
        CMD_LOOP_END,
        CMD_SET_COLOR, 255, 0, 0, MS_1000,
        CMD_LOOP_END,
        CMD_SET_COLOR, 0, 255, 0, MS_1000,
        CMD_END
    };

    loadBytecode(bytecode, sizeof(bytecode));

    TEST_ASSERT_EQUAL_COLOR_RGB(255, 0, 0, sb_light_player_get_color_at(&player, 500));
    TEST_ASSERT_EQUAL_COLOR_RGB(0, 255, 0, sb_light_player_get_color_at(&player, 1500));
    TEST_ASSERT_FALSE(sb_light_player_seek(&player, 1999, 0));
    TEST_ASSERT_TRUE(sb_light_player_seek(&player, 2000, 0));

    sb_light_player_destroy(&player);
}

void test_loops_nested_four_levels_deep(void)
{
    /* Four levels of nesting is the maximum that the player supports */
    uint8_t bytecode[] = {
        CMD_LOOP_BEGIN, 2,
        CMD_LOOP_BEGIN, 2,
        CMD_LOOP_BEGIN, 2,
        CMD_LOOP_BEGIN, 2,
        CMD_SET_COLOR, 255, 0, 0, MS_1000,
        CMD_LOOP_END,
        CMD_LOOP_END,
        CMD_LOOP_END,
        CMD_LOOP_END,
        CMD_SET_COLOR, 0, 255, 0, MS_1000,
        CMD_END
    };

    loadBytecode(bytecode, sizeof(bytecode));

    /* 2^4 iterations of 1 second each */
    TEST_ASSERT_EQUAL_COLOR_RGB(255, 0, 0, sb_light_player_get_color_at(&player, 15999));
    TEST_ASSERT_EQUAL_COLOR_RGB(0, 255, 0, sb_light_player_get_color_at(&player, 16000));
    TEST_ASSERT_FALSE(sb_light_player_seek(&player, 16999, 0));
    TEST_ASSERT_TRUE(sb_light_player_seek(&player, 17000, 0));

    sb_light_player_destroy(&player);
}

void test_loops_nested_too_deep(void)
{
    /* Regression test: the fifth nested loop used to be written past the end
     * of the loop stack. Exceeding the maximum nesting depth now stops the
     * execution of the program. */
    uint8_t bytecode[] = {
        CMD_SET_COLOR, 0, 0, 255, MS_1000,
        CMD_LOOP_BEGIN, 2,
        CMD_LOOP_BEGIN, 2,
        CMD_LOOP_BEGIN, 2,
        CMD_LOOP_BEGIN, 2,
        CMD_LOOP_BEGIN, 2,
        CMD_SET_COLOR, 255, 0, 0, MS_1000,
        CMD_LOOP_END,
        CMD_LOOP_END,
        CMD_LOOP_END,
        CMD_LOOP_END,
        CMD_LOOP_END,
        CMD_END
    };

    loadBytecode(bytecode, sizeof(bytecode));

    TEST_ASSERT_FALSE(sb_light_player_seek(&player, 999, 0));
    TEST_ASSERT_TRUE(sb_light_player_seek(&player, 1000, 0));
    TEST_ASSERT_EQUAL_COLOR_RGB(0, 0, 255, sb_light_player_get_color_at(&player, 1500));

    sb_light_player_destroy(&player);
}

void test_infinite_zero_duration_loops_are_rejected(void)
{
    /* Regression test: an infinite loop whose body takes no time used to
     * hang the player. Such loops now stop the execution of the program
     * when the end of the loop body is reached. */
    uint8_t infinite_empty_body[] = {
        CMD_SET_COLOR, 255, 0, 0, MS_1000,
        CMD_LOOP_BEGIN, 0,
        CMD_NOP,
        CMD_SET_COLOR, 0, 0, 255, 0,
        CMD_LOOP_END,
        CMD_SET_COLOR, 0, 255, 0, MS_1000,
        CMD_END
    };

    /* The first iteration of this loop takes time, but all the subsequent
     * ones do not as the deadline of the wait command is already in the past */
    uint8_t infinite_body_with_wait[] = {
        CMD_LOOP_BEGIN, 0,
        CMD_SET_COLOR, 255, 0, 0, 0,
        CMD_WAIT_UNTIL, MS_1000,
        CMD_LOOP_END,
        CMD_SET_COLOR, 0, 255, 0, MS_1000,
        CMD_END
    };

    loadBytecode(infinite_empty_body, sizeof(infinite_empty_body));
    TEST_ASSERT_FALSE(sb_light_player_seek(&player, 999, 0));
    TEST_ASSERT_TRUE(sb_light_player_seek(&player, 1000, 0));
    TEST_ASSERT_EQUAL_COLOR_RGB(0, 0, 255, sb_light_player_get_color_at(&player, 1500));
    sb_light_player_destroy(&player);

    loadBytecode(infinite_body_with_wait, sizeof(infinite_body_with_wait));
    TEST_ASSERT_EQUAL_COLOR_RGB(255, 0, 0, sb_light_player_get_color_at(&player, 500));
    TEST_ASSERT_FALSE(sb_light_player_seek(&player, 999, 0));
    TEST_ASSERT_TRUE(sb_light_player_seek(&player, 1000, 0));
    TEST_ASSERT_EQUAL_COLOR_RGB(255, 0, 0, sb_light_player_get_color_at(&player, 1500));
    sb_light_player_destroy(&player);
}

void test_finite_zero_duration_loops_are_allowed(void)
{
    /* Finite loops terminate even if their body takes no time so they are
     * harmless */
    uint8_t finite_empty_body[] = {
        CMD_SET_COLOR, 255, 0, 0, MS_1000,
        CMD_LOOP_BEGIN, 3,
        CMD_LOOP_END,
        CMD_SET_COLOR, 0, 255, 0, MS_1000,
        CMD_END
    };

    /* The first iteration of this loop takes time, but the subsequent ones
     * do not as the deadline of the wait command is already in the past */
    uint8_t finite_body_with_wait[] = {
        CMD_LOOP_BEGIN, 3,
        CMD_SET_COLOR, 255, 0, 0, 0,
        CMD_WAIT_UNTIL, MS_1000,
        CMD_LOOP_END,
        CMD_SET_COLOR, 0, 255, 0, MS_1000,
        CMD_END
    };

    loadBytecode(finite_empty_body, sizeof(finite_empty_body));
    TEST_ASSERT_EQUAL_COLOR_RGB(255, 0, 0, sb_light_player_get_color_at(&player, 500));
    TEST_ASSERT_EQUAL_COLOR_RGB(0, 255, 0, sb_light_player_get_color_at(&player, 1000));
    TEST_ASSERT_EQUAL_COLOR_RGB(0, 255, 0, sb_light_player_get_color_at(&player, 1500));
    TEST_ASSERT_FALSE(sb_light_player_seek(&player, 1999, 0));
    TEST_ASSERT_TRUE(sb_light_player_seek(&player, 2000, 0));
    sb_light_player_destroy(&player);

    loadBytecode(finite_body_with_wait, sizeof(finite_body_with_wait));
    TEST_ASSERT_EQUAL_COLOR_RGB(255, 0, 0, sb_light_player_get_color_at(&player, 500));
    TEST_ASSERT_EQUAL_COLOR_RGB(0, 255, 0, sb_light_player_get_color_at(&player, 1000));
    TEST_ASSERT_FALSE(sb_light_player_seek(&player, 1999, 0));
    TEST_ASSERT_TRUE(sb_light_player_seek(&player, 2000, 0));
    sb_light_player_destroy(&player);
}

void test_invalid_command_stops_execution(void)
{
    uint8_t bytecode[] = {
        CMD_SET_COLOR, 255, 0, 0, MS_1000,
        NUMBER_OF_COMMANDS,
        CMD_SET_COLOR, 0, 0, 255, MS_1000,
        CMD_END
    };

    loadBytecode(bytecode, sizeof(bytecode));

    TEST_ASSERT_FALSE(sb_light_player_seek(&player, 999, 0));
    TEST_ASSERT_TRUE(sb_light_player_seek(&player, 1000, 0));
    TEST_ASSERT_EQUAL_COLOR_RGB(255, 0, 0, sb_light_player_get_color_at(&player, 1500));
    TEST_ASSERT_EQUAL_COLOR_RGB(255, 0, 0, sb_light_player_get_color_at(&player, 2500));

    sb_light_player_destroy(&player);
}

int main(int argc, char* argv[])
{
    UNITY_BEGIN();

    RUN_TEST(test_get_color_at);
    RUN_TEST(test_constant_color_program);
    RUN_TEST(test_empty_program);
    RUN_TEST(test_fade_commands);
    RUN_TEST(test_zero_duration_fade);
    RUN_TEST(test_fade_after_zero_duration_fade);
    RUN_TEST(test_finite_loop);
    RUN_TEST(test_infinite_loop);
    RUN_TEST(test_nested_loops);
    RUN_TEST(test_zero_duration_commands_at_boundary);
    RUN_TEST(test_loop_end_without_loop_begin);
    RUN_TEST(test_loops_nested_four_levels_deep);
    RUN_TEST(test_loops_nested_too_deep);
    RUN_TEST(test_infinite_zero_duration_loops_are_rejected);
    RUN_TEST(test_finite_zero_duration_loops_are_allowed);
    RUN_TEST(test_invalid_command_stops_execution);

    return UNITY_END();
}
