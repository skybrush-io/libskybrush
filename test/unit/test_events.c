/*
 * This file is part of libskybrush.
 *
 * Copyright 2025 CollMot Robotics Ltd.
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
sb_bool_t events_inited;

sb_error_t loadFixture(const char* fname);
void closeFixture(void);

void setUp(void)
{
    loadFixture("fixtures/pyro_events.skyb");
}

void tearDown(void)
{
    if (events_inited) {
        closeFixture();
    }
}

sb_error_t loadFixture(const char* fname)
{
    FILE* fp;
    int fd;
    sb_error_t retval;

    retval = sb_event_list_init(&events, 0);
    events_inited = retval == SB_SUCCESS;
    if (retval != SB_SUCCESS) {
        return retval;
    }

    fp = fopen(fname, "rb");
    if (fp == 0) {
        perror(fname);
        abort();
    }

    fd = fileno(fp);
    if (fd < 0) {
        perror(NULL);
        abort();
    }

    retval = sb_event_list_update_from_binary_file(&events, fd);

    fclose(fp);

    return retval;
}

sb_error_t loadFixtureInMemory(const char* fname)
{
    FILE* fp;
    uint8_t* buf;
    ssize_t num_bytes;
    sb_error_t retval;

    retval = sb_event_list_init(&events, 8);
    events_inited = retval == SB_SUCCESS;
    if (retval != SB_SUCCESS) {
        return retval;
    }

    fp = fopen(fname, "rb");
    if (fp == 0) {
        perror(fname);
        abort();
    }

    buf = (uint8_t*)malloc(65536);
    if (buf == 0) {
        perror(NULL);
        abort();
    }

    num_bytes = fread(buf, sizeof(uint8_t), 65536, fp);
    if (ferror(fp)) {
        perror(NULL);
        abort();
    }

    fclose(fp);

    retval = sb_event_list_update_from_binary_file_in_memory(&events, buf, num_bytes);

    free(buf);

    return retval;
}

void closeFixture(void)
{
    sb_event_list_destroy(&events);
    events_inited = 0;
}

void test_event_list_is_empty(void)
{
    TEST_ASSERT(sb_event_list_is_empty(&events));

    /*
    float t[] = { -10, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60 };
    int i, n = sizeof(t) / sizeof(t[0]);
    float value;
    sb_yaw_player_t player;

    TEST_ASSERT(sb_yaw_control_is_empty(&ctrl));

    sb_yaw_player_init(&player, &ctrl);

    for (i = 0; i < n; i++) {
        sb_yaw_player_get_yaw_at(&player, t[i], &value);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, 0, value);

        sb_yaw_player_get_yaw_rate_at(&player, t[i], &value);
        TEST_ASSERT_FLOAT_WITHIN(1e-7, 0, value);
    }

    sb_yaw_player_destroy(&player);
    */
}

void test_init_empty(void)
{
    closeFixture(); /* was created in setUp() */
    sb_event_list_init(&events, 8);
    test_event_list_is_empty();
}

void test_init_with_zero_length(void)
{
    closeFixture(); /* was created in setUp() */
    sb_event_list_init(&events, 0);
    test_event_list_is_empty();
    TEST_ASSERT_EQUAL(1, sb_event_list_capacity(&events));
}

void test_loaded_events(void)
{
    TEST_ASSERT_EQUAL(4, sb_event_list_size(&events));
    TEST_ASSERT_EQUAL(10000, sb_event_list_get_ptr(&events, 0)->time_msec);
    TEST_ASSERT_EQUAL(SB_EVENT_TYPE_PYRO, sb_event_list_get_ptr(&events, 0)->type);
    TEST_ASSERT_EQUAL(1, sb_event_list_get_ptr(&events, 0)->subtype);
    TEST_ASSERT_EQUAL(50000, sb_event_list_get_ptr(&events, 1)->time_msec);
    TEST_ASSERT_EQUAL(SB_EVENT_TYPE_PYRO, sb_event_list_get_ptr(&events, 1)->type);
    TEST_ASSERT_EQUAL(2, sb_event_list_get_ptr(&events, 1)->subtype);
    TEST_ASSERT_EQUAL(90000, sb_event_list_get_ptr(&events, 2)->time_msec);
    TEST_ASSERT_EQUAL(SB_EVENT_TYPE_PYRO, sb_event_list_get_ptr(&events, 2)->type);
    TEST_ASSERT_EQUAL(3, sb_event_list_get_ptr(&events, 2)->subtype);
    TEST_ASSERT_EQUAL(90000, sb_event_list_get_ptr(&events, 3)->time_msec);
    TEST_ASSERT_EQUAL(SB_EVENT_TYPE_PYRO, sb_event_list_get_ptr(&events, 3)->type);
    TEST_ASSERT_EQUAL(4, sb_event_list_get_ptr(&events, 3)->subtype);
    TEST_ASSERT_NULL(sb_event_list_get_ptr(&events, 4));

    TEST_ASSERT_EQUAL(4, sb_event_list_size(&events));
    TEST_ASSERT_EQUAL(10000, sb_event_list_get_ptr_const(&events, 0)->time_msec);
    TEST_ASSERT_EQUAL(SB_EVENT_TYPE_PYRO, sb_event_list_get_ptr_const(&events, 0)->type);
    TEST_ASSERT_EQUAL(1, sb_event_list_get_ptr_const(&events, 0)->subtype);
    TEST_ASSERT_EQUAL(50000, sb_event_list_get_ptr_const(&events, 1)->time_msec);
    TEST_ASSERT_EQUAL(SB_EVENT_TYPE_PYRO, sb_event_list_get_ptr_const(&events, 1)->type);
    TEST_ASSERT_EQUAL(2, sb_event_list_get_ptr_const(&events, 1)->subtype);
    TEST_ASSERT_EQUAL(90000, sb_event_list_get_ptr_const(&events, 2)->time_msec);
    TEST_ASSERT_EQUAL(SB_EVENT_TYPE_PYRO, sb_event_list_get_ptr_const(&events, 2)->type);
    TEST_ASSERT_EQUAL(3, sb_event_list_get_ptr_const(&events, 2)->subtype);
    TEST_ASSERT_EQUAL(90000, sb_event_list_get_ptr_const(&events, 3)->time_msec);
    TEST_ASSERT_EQUAL(SB_EVENT_TYPE_PYRO, sb_event_list_get_ptr_const(&events, 3)->type);
    TEST_ASSERT_EQUAL(4, sb_event_list_get_ptr_const(&events, 3)->subtype);
    TEST_ASSERT_NULL(sb_event_list_get_ptr_const(&events, 4));
}

void test_loaded_events_in_memory(void)
{
    closeFixture();
    loadFixtureInMemory("fixtures/pyro_events.skyb");
    test_loaded_events();
}

void test_append_with_earlier_timestamp(void)
{
    sb_event_t event;
    event.time_msec = 0; /* earlier than what is already in the list */
    event.type = SB_EVENT_TYPE_PYRO;
    event.subtype = 1;

    TEST_ASSERT_EQUAL(SB_EINVAL, sb_event_list_append(&events, &event));
}

void test_insertion(void)
{
    sb_event_t event;
    event.type = SB_EVENT_TYPE_PYRO;
    event.subtype = 1;

    /* 10000, 50000, 90000, 90000 -- inserting in front */
    event.time_msec = 5000;
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_event_list_insert(&events, &event));
    TEST_ASSERT_EQUAL(5, sb_event_list_size(&events));
    TEST_ASSERT_EQUAL(5000, sb_event_list_get_ptr(&events, 0)->time_msec);
    TEST_ASSERT_EQUAL(10000, sb_event_list_get_ptr(&events, 1)->time_msec);
    TEST_ASSERT_EQUAL(50000, sb_event_list_get_ptr(&events, 2)->time_msec);
    TEST_ASSERT_EQUAL(90000, sb_event_list_get_ptr(&events, 3)->time_msec);
    TEST_ASSERT_EQUAL(90000, sb_event_list_get_ptr(&events, 4)->time_msec);

    /* 5000, 10000, 50000, 90000, 90000 -- inserting at end */
    event.time_msec = 110000;
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_event_list_insert(&events, &event));
    TEST_ASSERT_EQUAL(6, sb_event_list_size(&events));
    TEST_ASSERT_EQUAL(5000, sb_event_list_get_ptr(&events, 0)->time_msec);
    TEST_ASSERT_EQUAL(10000, sb_event_list_get_ptr(&events, 1)->time_msec);
    TEST_ASSERT_EQUAL(50000, sb_event_list_get_ptr(&events, 2)->time_msec);
    TEST_ASSERT_EQUAL(90000, sb_event_list_get_ptr(&events, 3)->time_msec);
    TEST_ASSERT_EQUAL(90000, sb_event_list_get_ptr(&events, 4)->time_msec);
    TEST_ASSERT_EQUAL(110000, sb_event_list_get_ptr(&events, 5)->time_msec);

    /* 5000, 10000, 50000, 90000, 90000, 110000 -- inserting in the middle */
    event.time_msec = 40000;
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_event_list_insert(&events, &event));
    TEST_ASSERT_EQUAL(7, sb_event_list_size(&events));
    TEST_ASSERT_EQUAL(5000, sb_event_list_get_ptr(&events, 0)->time_msec);
    TEST_ASSERT_EQUAL(10000, sb_event_list_get_ptr(&events, 1)->time_msec);
    TEST_ASSERT_EQUAL(40000, sb_event_list_get_ptr(&events, 2)->time_msec);
    TEST_ASSERT_EQUAL(50000, sb_event_list_get_ptr(&events, 3)->time_msec);
    TEST_ASSERT_EQUAL(90000, sb_event_list_get_ptr(&events, 4)->time_msec);
    TEST_ASSERT_EQUAL(90000, sb_event_list_get_ptr(&events, 5)->time_msec);
    TEST_ASSERT_EQUAL(110000, sb_event_list_get_ptr(&events, 6)->time_msec);

    /* 5000, 10000, 40000, 50000, 90000, 90000, 110000 -- inserting with same timestamp in the middle */
    event.time_msec = 40000;
    event.subtype = 3; /* different subtype */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_event_list_insert(&events, &event));
    TEST_ASSERT_EQUAL(8, sb_event_list_size(&events));
    TEST_ASSERT_EQUAL(5000, sb_event_list_get_ptr(&events, 0)->time_msec);
    TEST_ASSERT_EQUAL(10000, sb_event_list_get_ptr(&events, 1)->time_msec);
    TEST_ASSERT_EQUAL(40000, sb_event_list_get_ptr(&events, 2)->time_msec);
    TEST_ASSERT_EQUAL(1, sb_event_list_get_ptr(&events, 2)->subtype);
    TEST_ASSERT_EQUAL(40000, sb_event_list_get_ptr(&events, 3)->time_msec);
    TEST_ASSERT_EQUAL(3, sb_event_list_get_ptr(&events, 3)->subtype);
    TEST_ASSERT_EQUAL(50000, sb_event_list_get_ptr(&events, 4)->time_msec);
    TEST_ASSERT_EQUAL(90000, sb_event_list_get_ptr(&events, 5)->time_msec);
    TEST_ASSERT_EQUAL(90000, sb_event_list_get_ptr(&events, 6)->time_msec);
    TEST_ASSERT_EQUAL(110000, sb_event_list_get_ptr(&events, 7)->time_msec);

    /* Inserting with same timestamp in front */
    event.time_msec = 5000;
    event.subtype = 3; /* different subtype */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_event_list_insert(&events, &event));
    TEST_ASSERT_EQUAL(9, sb_event_list_size(&events));
    TEST_ASSERT_EQUAL(5000, sb_event_list_get_ptr(&events, 0)->time_msec);
    TEST_ASSERT_EQUAL(1, sb_event_list_get_ptr(&events, 0)->subtype);
    TEST_ASSERT_EQUAL(5000, sb_event_list_get_ptr(&events, 1)->time_msec);
    TEST_ASSERT_EQUAL(3, sb_event_list_get_ptr(&events, 1)->subtype);

    /* Inserting with same timestamp at end */
    event.time_msec = 110000;
    event.subtype = 3; /* different subtype */
    TEST_ASSERT_EQUAL(SB_SUCCESS, sb_event_list_insert(&events, &event));
    TEST_ASSERT_EQUAL(10, sb_event_list_size(&events));
    TEST_ASSERT_EQUAL(110000, sb_event_list_get_ptr(&events, 8)->time_msec);
    TEST_ASSERT_EQUAL(1, sb_event_list_get_ptr(&events, 8)->subtype);
    TEST_ASSERT_EQUAL(110000, sb_event_list_get_ptr(&events, 8)->time_msec);
    TEST_ASSERT_EQUAL(3, sb_event_list_get_ptr(&events, 9)->subtype);
}

void test_is_sorted(void)
{
    uint32_t timestamp;
    sb_event_t* event_ptr;

    TEST_ASSERT_TRUE(sb_event_list_is_sorted(&events));

    /* Mix up the event timestamps a bit */
    event_ptr = sb_event_list_get_ptr(&events, 1);
    timestamp = event_ptr->time_msec;
    event_ptr = sb_event_list_get_ptr(&events, 0);
    event_ptr->time_msec = timestamp + 1000; /* later than the second event */

    TEST_ASSERT_FALSE(sb_event_list_is_sorted(&events));

    /* Test special cases */
    sb_event_list_clear(&events);
    TEST_ASSERT_TRUE(sb_event_list_is_sorted(&events));
}

void test_sort(void)
{
    uint32_t timestamp;
    sb_event_t* event_ptr;

    /* Mix up the event timestamps a bit */
    event_ptr = sb_event_list_get_ptr(&events, 1);
    timestamp = event_ptr->time_msec;
    event_ptr = sb_event_list_get_ptr(&events, 0);
    event_ptr->time_msec = timestamp + 1000; /* later than the second event */

    TEST_ASSERT_FALSE(sb_event_list_is_sorted(&events));
    sb_event_list_sort(&events);
    TEST_ASSERT_TRUE(sb_event_list_is_sorted(&events));
}

void test_adjust_timestamps(void)
{
    sb_event_t* event_ptr;

    /* Adjust the timestamps of the pyro events forward by 1000 ms */
    sb_event_list_adjust_timestamps_by_type(&events, SB_EVENT_TYPE_PYRO, 1000);

    /* Check that the timestamps were adjusted correctly */
    event_ptr = sb_event_list_get_ptr(&events, 0);
    TEST_ASSERT_EQUAL(11000, event_ptr->time_msec);
    event_ptr = sb_event_list_get_ptr(&events, 1);
    TEST_ASSERT_EQUAL(51000, event_ptr->time_msec);
    event_ptr = sb_event_list_get_ptr(&events, 2);
    TEST_ASSERT_EQUAL(91000, event_ptr->time_msec);
    event_ptr = sb_event_list_get_ptr(&events, 3);
    TEST_ASSERT_EQUAL(91000, event_ptr->time_msec); /* last one should not change */

    /* Check that the event type is really considered */
    sb_event_list_adjust_timestamps_by_type(&events, SB_EVENT_TYPE_NONE, 1000);
    event_ptr = sb_event_list_get_ptr(&events, 0);
    TEST_ASSERT_EQUAL(11000, event_ptr->time_msec);
    event_ptr = sb_event_list_get_ptr(&events, 1);
    TEST_ASSERT_EQUAL(51000, event_ptr->time_msec);
    event_ptr = sb_event_list_get_ptr(&events, 2);
    TEST_ASSERT_EQUAL(91000, event_ptr->time_msec);
    event_ptr = sb_event_list_get_ptr(&events, 3);
    TEST_ASSERT_EQUAL(91000, event_ptr->time_msec); /* last one should not change */

    /* Adjust the timestamps of the pyro events back by 1000 ms */
    sb_event_list_adjust_timestamps_by_type(&events, SB_EVENT_TYPE_PYRO, -1000);

    /* Check that the timestamps were adjusted correctly */
    event_ptr = sb_event_list_get_ptr(&events, 0);
    TEST_ASSERT_EQUAL(10000, event_ptr->time_msec);
    event_ptr = sb_event_list_get_ptr(&events, 1);
    TEST_ASSERT_EQUAL(50000, event_ptr->time_msec);
    event_ptr = sb_event_list_get_ptr(&events, 2);
    TEST_ASSERT_EQUAL(90000, event_ptr->time_msec);
    event_ptr = sb_event_list_get_ptr(&events, 3);
    TEST_ASSERT_EQUAL(90000, event_ptr->time_msec);

    /* Adjust the timestamps of the pyro events such that they would underflow */
    sb_event_list_adjust_timestamps_by_type(&events, SB_EVENT_TYPE_PYRO, -50000);

    /* Check that the timestamps were adjusted correctly */
    event_ptr = sb_event_list_get_ptr(&events, 0);
    TEST_ASSERT_EQUAL(0, event_ptr->time_msec);
    event_ptr = sb_event_list_get_ptr(&events, 1);
    TEST_ASSERT_EQUAL(0, event_ptr->time_msec);
    event_ptr = sb_event_list_get_ptr(&events, 2);
    TEST_ASSERT_EQUAL(40000, event_ptr->time_msec);
    event_ptr = sb_event_list_get_ptr(&events, 3);
    TEST_ASSERT_EQUAL(40000, event_ptr->time_msec);

    /* Adjust the timestamps of the pyro events such that they would overflow */
    sb_event_list_adjust_timestamps_by_type(&events, SB_EVENT_TYPE_PYRO, INT32_MAX);
    sb_event_list_adjust_timestamps_by_type(&events, SB_EVENT_TYPE_PYRO, INT32_MAX);

    /* Check that the timestamps were adjusted correctly.
     * Note that 2 * INT32_MAX == UINT32_MAX - 1 */
    event_ptr = sb_event_list_get_ptr(&events, 0);
    TEST_ASSERT_EQUAL(UINT32_MAX - 1, event_ptr->time_msec);
    event_ptr = sb_event_list_get_ptr(&events, 1);
    TEST_ASSERT_EQUAL(UINT32_MAX - 1, event_ptr->time_msec);
    event_ptr = sb_event_list_get_ptr(&events, 2);
    TEST_ASSERT_EQUAL(UINT32_MAX, event_ptr->time_msec);
    event_ptr = sb_event_list_get_ptr(&events, 3);
    TEST_ASSERT_EQUAL(UINT32_MAX, event_ptr->time_msec);
}

void test_add_pyro_off_events(void)
{
    sb_error_t retval;

    sb_event_list_get_ptr(&events, 0)->subtype = 1;
    sb_event_list_get_ptr(&events, 1)->subtype = 2;
    sb_event_list_get_ptr(&events, 2)->subtype = 3;
    sb_event_list_get_ptr(&events, 3)->subtype = 4;

    /* Add pyro off events with 1000 ms duration */
    retval = sb_event_list_add_pyro_off_events(&events, 1000);
    TEST_ASSERT_EQUAL(SB_SUCCESS, retval);
    TEST_ASSERT_EQUAL(8, sb_event_list_size(&events));

    /* Check that the pyro off events were added correctly */
    TEST_ASSERT_EQUAL(11000, sb_event_list_get_ptr(&events, 1)->time_msec);
    TEST_ASSERT_EQUAL(SB_EVENT_TYPE_PYRO, sb_event_list_get_ptr(&events, 1)->type);
    TEST_ASSERT_EQUAL(1, sb_event_list_get_ptr(&events, 1)->subtype);
    TEST_ASSERT_EQUAL(UINT32_MAX, sb_event_list_get_ptr(&events, 1)->payload.as_uint32);

    TEST_ASSERT_EQUAL(51000, sb_event_list_get_ptr(&events, 3)->time_msec);
    TEST_ASSERT_EQUAL(SB_EVENT_TYPE_PYRO, sb_event_list_get_ptr(&events, 3)->type);
    TEST_ASSERT_EQUAL(2, sb_event_list_get_ptr(&events, 3)->subtype);
    TEST_ASSERT_EQUAL(UINT32_MAX, sb_event_list_get_ptr(&events, 3)->payload.as_uint32);

    TEST_ASSERT_EQUAL(91000, sb_event_list_get_ptr(&events, 6)->time_msec);
    TEST_ASSERT_EQUAL(SB_EVENT_TYPE_PYRO, sb_event_list_get_ptr(&events, 6)->type);
    TEST_ASSERT_EQUAL(3, sb_event_list_get_ptr(&events, 6)->subtype);
    TEST_ASSERT_EQUAL(UINT32_MAX, sb_event_list_get_ptr(&events, 6)->payload.as_uint32);

    TEST_ASSERT_EQUAL(91000, sb_event_list_get_ptr(&events, 7)->time_msec);
    TEST_ASSERT_EQUAL(SB_EVENT_TYPE_PYRO, sb_event_list_get_ptr(&events, 7)->type);
    TEST_ASSERT_EQUAL(4, sb_event_list_get_ptr(&events, 7)->subtype);
    TEST_ASSERT_EQUAL(UINT32_MAX, sb_event_list_get_ptr(&events, 7)->payload.as_uint32);

    /* Add pyro off events again with a larger allowed duration -- should not
     * add any new events */
    retval = sb_event_list_add_pyro_off_events(&events, 2000);
    TEST_ASSERT_EQUAL(SB_SUCCESS, retval);
    TEST_ASSERT_EQUAL(8, sb_event_list_size(&events));

    /* Add pyro off events again with exactly the same allowed duration --
     * should not add any new events */
    retval = sb_event_list_add_pyro_off_events(&events, 1000);
    TEST_ASSERT_EQUAL(SB_SUCCESS, retval);
    TEST_ASSERT_EQUAL(8, sb_event_list_size(&events));
}

void test_add_pyro_off_events_overlapping(void)
{
    sb_error_t retval;

    sb_event_list_get_ptr(&events, 0)->subtype = 1;
    sb_event_list_get_ptr(&events, 1)->subtype = 2;
    sb_event_list_get_ptr(&events, 2)->subtype = 3;
    sb_event_list_get_ptr(&events, 3)->subtype = 4;

    /* Add pyro off events with 60000 ms duration -- large enough such that
     * on and off events overlap */
    retval = sb_event_list_add_pyro_off_events(&events, 60000);
    TEST_ASSERT_EQUAL(SB_SUCCESS, retval);
    TEST_ASSERT_EQUAL(8, sb_event_list_size(&events));

    /* Check that the pyro off events were added correctly */
    TEST_ASSERT_EQUAL(70000, sb_event_list_get_ptr(&events, 2)->time_msec);
    TEST_ASSERT_EQUAL(SB_EVENT_TYPE_PYRO, sb_event_list_get_ptr(&events, 2)->type);
    TEST_ASSERT_EQUAL(1, sb_event_list_get_ptr(&events, 2)->subtype);
    TEST_ASSERT_EQUAL(UINT32_MAX, sb_event_list_get_ptr(&events, 2)->payload.as_uint32);

    TEST_ASSERT_EQUAL(110000, sb_event_list_get_ptr(&events, 5)->time_msec);
    TEST_ASSERT_EQUAL(SB_EVENT_TYPE_PYRO, sb_event_list_get_ptr(&events, 5)->type);
    TEST_ASSERT_EQUAL(2, sb_event_list_get_ptr(&events, 5)->subtype);
    TEST_ASSERT_EQUAL(UINT32_MAX, sb_event_list_get_ptr(&events, 5)->payload.as_uint32);

    TEST_ASSERT_EQUAL(150000, sb_event_list_get_ptr(&events, 6)->time_msec);
    TEST_ASSERT_EQUAL(SB_EVENT_TYPE_PYRO, sb_event_list_get_ptr(&events, 6)->type);
    TEST_ASSERT_EQUAL(3, sb_event_list_get_ptr(&events, 6)->subtype);
    TEST_ASSERT_EQUAL(UINT32_MAX, sb_event_list_get_ptr(&events, 6)->payload.as_uint32);

    TEST_ASSERT_EQUAL(150000, sb_event_list_get_ptr(&events, 7)->time_msec);
    TEST_ASSERT_EQUAL(SB_EVENT_TYPE_PYRO, sb_event_list_get_ptr(&events, 7)->type);
    TEST_ASSERT_EQUAL(4, sb_event_list_get_ptr(&events, 7)->subtype);
    TEST_ASSERT_EQUAL(UINT32_MAX, sb_event_list_get_ptr(&events, 7)->payload.as_uint32);
}

int main(int argc, char* argv[])
{
    UNITY_BEGIN();

    /* basic tests with pyro_events.skyb */
    RUN_TEST(test_init_empty);
    RUN_TEST(test_init_with_zero_length);
    RUN_TEST(test_loaded_events);
    RUN_TEST(test_loaded_events_in_memory);
    RUN_TEST(test_append_with_earlier_timestamp);
    RUN_TEST(test_insertion);
    RUN_TEST(test_is_sorted);
    RUN_TEST(test_sort);
    RUN_TEST(test_adjust_timestamps);
    RUN_TEST(test_add_pyro_off_events);
    RUN_TEST(test_add_pyro_off_events_overlapping);

    return UNITY_END();
}
