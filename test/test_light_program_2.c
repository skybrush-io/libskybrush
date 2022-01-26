#include <skybrush/formats/binary.h>
#include <skybrush/lights.h>

#include "unity.h"
#include "utils.h"

void setUp()
{
}

void tearDown()
{
}

void test_file_without_light_program()
{
    sb_light_program_t program;
    sb_error_t retval;

    FILE* fp;
    int fd;

    fp = fopen("fixtures/forward_left_back_no_lights.skyb", "rb");
    fd = fp != 0 ? fileno(fp) : -1;
    if (fd < 0) {
        abort();
    }

    retval = sb_light_program_init_from_binary_file(&program, fd);

    fclose(fp);

    TEST_ASSERT_EQUAL(SB_ENOENT, retval);
}

int main(int argc, char* argv[])
{
    UNITY_BEGIN();

    RUN_TEST(test_file_without_light_program);

    return UNITY_END();
}
