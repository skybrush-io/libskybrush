#include <string.h>
#include <skybrush/error.h>
#include <skybrush/utils.h>

#include "unity.h"

void setUp()
{
}

void tearDown()
{
}

void test_ap_crc32()
{
    const char *buf = "hello-world";
    uint32_t value;

    value = sb_ap_crc32_update(0, (void *)buf, strlen(buf));
    TEST_ASSERT_EQUAL_HEX32(0xda53b3b7, value);
}

int main(int argc, char *argv[])
{
    UNITY_BEGIN();

    RUN_TEST(test_ap_crc32);

    return UNITY_END();
}