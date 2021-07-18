#include <skybrush/utils.h>

#include "unity.h"

void setUp()
{
}

void tearDown()
{
}

void test_interval_expand()
{
    sb_interval_t interval = {/* .min = */ 3, /* .max = */ 11};

    sb_interval_expand(&interval, 3);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, interval.min);
    TEST_ASSERT_EQUAL_FLOAT(14.0f, interval.max);

    sb_interval_expand(&interval, -2);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, interval.min);
    TEST_ASSERT_EQUAL_FLOAT(12.0f, interval.max);

    sb_interval_expand(&interval, -20);
    TEST_ASSERT_EQUAL_FLOAT(7.0f, interval.min);
    TEST_ASSERT_EQUAL_FLOAT(7.0f, interval.max);
}

int main(int argc, char *argv[])
{
    UNITY_BEGIN();

    RUN_TEST(test_interval_expand);

    return UNITY_END();
}