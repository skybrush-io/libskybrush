#include <skybrush/error.h>

#include "unity.h"

void setUp()
{
}

void tearDown()
{
}

void test_error_to_string()
{
    TEST_ASSERT_EQUAL_STRING("No error", sb_error_to_string(SB_SUCCESS));
    TEST_ASSERT_EQUAL_STRING("Buffer is full", sb_error_to_string(SB_EFULL));
    TEST_ASSERT_EQUAL_STRING("Unspecified failure", sb_error_to_string(-1));
    TEST_ASSERT_EQUAL_STRING("Unspecified failure", sb_error_to_string(31999));
}

int main(int argc, char *argv[])
{
    UNITY_BEGIN();

    RUN_TEST(test_error_to_string);

    return UNITY_END();
}