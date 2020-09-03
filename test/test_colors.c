#include <skybrush/colors.h>

#include "unity.h"

void setUp()
{
}

void tearDown()
{
}

void test_equals()
{
    sb_rgb_color_t red = {255, 0, 0};
    sb_rgb_color_t another_red = {255, 0, 0};
    sb_rgb_color_t white = {255, 255, 255};

    TEST_ASSERT_TRUE(sb_rgb_color_equals(red, another_red));
    TEST_ASSERT_TRUE(sb_rgb_color_equals(white, SB_COLOR_WHITE));

    TEST_ASSERT_FALSE(sb_rgb_color_equals(red, SB_COLOR_BLACK));
    TEST_ASSERT_FALSE(sb_rgb_color_equals(SB_COLOR_WHITE, SB_COLOR_BLACK));
}

int main(int argc, char *argv[])
{
    UNITY_BEGIN();

    RUN_TEST(test_equals);

    return UNITY_END();
}