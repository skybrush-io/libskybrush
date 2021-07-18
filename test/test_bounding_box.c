#include <skybrush/utils.h>

#include "unity.h"

void setUp()
{
}

void tearDown()
{
}

void test_boudning_box_expand()
{
    sb_bounding_box_t box = {
        /* .x = */ {/* .min = */ 3, /* .max = */ 11},
        /* .y = */ {/* .min = */ -2, /* .max = */ -2},
        /* .z = */ {/* .min = */ 0, /* .max = */ 5}};

    sb_bounding_box_expand(&box, 2);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, box.x.min);
    TEST_ASSERT_EQUAL_FLOAT(13.0f, box.x.max);
    TEST_ASSERT_EQUAL_FLOAT(-4.0f, box.y.min);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, box.y.max);
    TEST_ASSERT_EQUAL_FLOAT(-2.0f, box.z.min);
    TEST_ASSERT_EQUAL_FLOAT(7.0f, box.z.max);

    sb_bounding_box_expand(&box, -3);
    TEST_ASSERT_EQUAL_FLOAT(4.0f, box.x.min);
    TEST_ASSERT_EQUAL_FLOAT(10.0f, box.x.max);
    TEST_ASSERT_EQUAL_FLOAT(-2.0f, box.y.min);
    TEST_ASSERT_EQUAL_FLOAT(-2.0f, box.y.max);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, box.z.min);
    TEST_ASSERT_EQUAL_FLOAT(4.0f, box.z.max);
}

int main(int argc, char *argv[])
{
    UNITY_BEGIN();

    RUN_TEST(test_boudning_box_expand);

    return UNITY_END();
}