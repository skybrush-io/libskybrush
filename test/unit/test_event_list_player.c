/*
 * This file is part of libskybrush.
 *
 * Copyright 2025-2026 CollMot Robotics Ltd.
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

#include <skybrush/events.h>
#include <skybrush/formats/binary.h>

#include "unity.h"

sb_event_list_t events;
sb_event_list_player_t player;

void loadFixture(const char* fname);
void closeFixture(void);

void setUp(void)
{
    loadFixture("fixtures/pyro_events.skyb");
}

void tearDown(void)
{
    closeFixture();
}

void loadFixture(const char* fname)
{
    FILE* fp;
    int fd;

    fp = fopen(fname, "rb");
    if (fp == 0) {
        abort();
    }

    fd = fileno(fp);
    if (fd < 0) {
        abort();
    }

    sb_event_list_init(&events, 0);
    sb_event_list_update_from_binary_file(&events, fd);
    sb_event_list_player_init(&player, &events);

    fclose(fp);
}

void closeFixture(void)
{
    sb_event_list_player_destroy(&player);
    SB_DECREF_STATIC(&events);
}

void test_iteration(void)
{
    const sb_event_t* event = NULL;

    event = sb_event_list_player_get_next_event(&player);
    TEST_ASSERT_NOT_NULL(event);
    TEST_ASSERT_EQUAL(10000, event->time_msec);
    TEST_ASSERT_EQUAL(SB_EVENT_TYPE_PYRO, event->type);
    TEST_ASSERT_EQUAL(1, event->subtype);
    TEST_ASSERT_EQUAL(0, event->payload.as_uint32);

    event = sb_event_list_player_get_next_event(&player);
    TEST_ASSERT_NOT_NULL(event);
    TEST_ASSERT_EQUAL(50000, event->time_msec);
    TEST_ASSERT_EQUAL(SB_EVENT_TYPE_PYRO, event->type);
    TEST_ASSERT_EQUAL(2, event->subtype);
    TEST_ASSERT_EQUAL(0, event->payload.as_uint32);

    event = sb_event_list_player_get_next_event(&player);
    TEST_ASSERT_NOT_NULL(event);
    TEST_ASSERT_EQUAL(90000, event->time_msec);
    TEST_ASSERT_EQUAL(SB_EVENT_TYPE_PYRO, event->type);
    TEST_ASSERT_EQUAL(3, event->subtype);
    TEST_ASSERT_EQUAL(0, event->payload.as_uint32);

    event = sb_event_list_player_get_next_event(&player);
    TEST_ASSERT_NOT_NULL(event);
    TEST_ASSERT_EQUAL(90000, event->time_msec);
    TEST_ASSERT_EQUAL(SB_EVENT_TYPE_PYRO, event->type);
    TEST_ASSERT_EQUAL(4, event->subtype);
    TEST_ASSERT_EQUAL(0, event->payload.as_uint32);
}

void test_iteration_by_time(void)
{
    const sb_event_t* event = NULL;

    event = sb_event_list_player_get_next_event_not_later_than(&player, 60);
    TEST_ASSERT_NOT_NULL(event);
    TEST_ASSERT_EQUAL(10000, event->time_msec);
    TEST_ASSERT_EQUAL(SB_EVENT_TYPE_PYRO, event->type);
    TEST_ASSERT_EQUAL(1, event->subtype);
    TEST_ASSERT_EQUAL(0, event->payload.as_uint32);

    event = sb_event_list_player_get_next_event_not_later_than(&player, 60);
    TEST_ASSERT_NOT_NULL(event);
    TEST_ASSERT_EQUAL(50000, event->time_msec);
    TEST_ASSERT_EQUAL(SB_EVENT_TYPE_PYRO, event->type);
    TEST_ASSERT_EQUAL(2, event->subtype);
    TEST_ASSERT_EQUAL(0, event->payload.as_uint32);

    event = sb_event_list_player_get_next_event_not_later_than(&player, 60);
    TEST_ASSERT_NULL(event);

    sb_event_list_player_seek(&player, 40);

    event = sb_event_list_player_get_next_event_not_later_than(&player, 60);
    TEST_ASSERT_NOT_NULL(event);
    TEST_ASSERT_EQUAL(50000, event->time_msec);
    TEST_ASSERT_EQUAL(SB_EVENT_TYPE_PYRO, event->type);
    TEST_ASSERT_EQUAL(2, event->subtype);
    TEST_ASSERT_EQUAL(0, event->payload.as_uint32);

    event = sb_event_list_player_get_next_event_not_later_than(&player, 90);
    TEST_ASSERT_NOT_NULL(event);
    TEST_ASSERT_EQUAL(90000, event->time_msec);
    TEST_ASSERT_EQUAL(SB_EVENT_TYPE_PYRO, event->type);
    TEST_ASSERT_EQUAL(3, event->subtype);
    TEST_ASSERT_EQUAL(0, event->payload.as_uint32);

    event = sb_event_list_player_get_next_event_not_later_than(&player, 90);
    TEST_ASSERT_NOT_NULL(event);
    TEST_ASSERT_EQUAL(90000, event->time_msec);
    TEST_ASSERT_EQUAL(SB_EVENT_TYPE_PYRO, event->type);
    TEST_ASSERT_EQUAL(4, event->subtype);
    TEST_ASSERT_EQUAL(0, event->payload.as_uint32);

    event = sb_event_list_player_get_next_event_not_later_than(&player, 90);
    TEST_ASSERT_NULL(event);

    event = sb_event_list_player_get_next_event_not_later_than(&player, 100);
    TEST_ASSERT_NULL(event);
}

void test_rewind_after_iteration(void)
{
    const sb_event_t* event = NULL;
    size_t num_events = 0;

    while ((event = sb_event_list_player_get_next_event(&player)) != NULL) {
        num_events++;
    }

    TEST_ASSERT_EQUAL(4, num_events);

    sb_event_list_player_rewind(&player);

    num_events = 0;
    while ((event = sb_event_list_player_get_next_event(&player)) != NULL) {
        num_events++;
    }

    TEST_ASSERT_EQUAL(4, num_events);
}

void test_seek_to_very_large_timestamp(void)
{
    const sb_event_t* event = NULL;

    sb_event_list_player_seek(&player, ((float)UINT32_MAX) / 500);

    event = sb_event_list_player_get_next_event(&player);
    TEST_ASSERT_NULL(event);
}

int main(int argc, char* argv[])
{
    UNITY_BEGIN();

    RUN_TEST(test_iteration);
    RUN_TEST(test_iteration_by_time);
    RUN_TEST(test_rewind_after_iteration);
    RUN_TEST(test_seek_to_very_large_timestamp);

    return UNITY_END();
}
