# Fuzz targets. fuzz_show takes a whole show file, fuzz_block takes the type
# and the body of a single block. With LIBSKYBRUSH_ENABLE_FUZZING, they are
# linked with libFuzzer; otherwise they are linked with a driver that runs
# the target on the files given on the command line, which is used to replay
# the test fixtures as a regression test.
file(GLOB FUZZ_FIXTURES "${PROJECT_SOURCE_DIR}/test/fixtures/*.skyb")

foreach(target fuzz_show fuzz_block)
    if(LIBSKYBRUSH_ENABLE_FUZZING)
        add_executable(${target} fuzz_show.c)
        target_link_options(${target} PRIVATE -fsanitize=fuzzer)
    else()
        add_executable(${target} fuzz_show.c replay_main.c)
    endif()
    target_link_libraries(${target} PRIVATE skybrush)
endforeach()

target_compile_definitions(fuzz_block PRIVATE FUZZ_WRAP_BLOCK)

if(NOT LIBSKYBRUSH_ENABLE_FUZZING)
    add_test(NAME fuzz_show_replay COMMAND fuzz_show ${FUZZ_FIXTURES})

    find_program(LIBSKYBRUSH_PYTHON3 NAMES python3 python)
    if(LIBSKYBRUSH_PYTHON3)
        set(FUZZ_BLOCK_SEEDS "${CMAKE_CURRENT_BINARY_DIR}/block_seeds")
        add_test(
            NAME fuzz_block_seeds
            COMMAND ${LIBSKYBRUSH_PYTHON3} ${CMAKE_CURRENT_SOURCE_DIR}/make_block_seeds.py
                ${FUZZ_BLOCK_SEEDS} ${FUZZ_FIXTURES}
        )
        set_tests_properties(fuzz_block_seeds PROPERTIES FIXTURES_SETUP fuzz_block_seeds)
        add_test(
            NAME fuzz_block_replay
            COMMAND ${CMAKE_COMMAND}
                -DREPLAY=$<TARGET_FILE:fuzz_block> -DSEEDS=${FUZZ_BLOCK_SEEDS}
                -P ${CMAKE_CURRENT_SOURCE_DIR}/replay_directory.cmake
        )
        set_tests_properties(fuzz_block_replay PROPERTIES FIXTURES_REQUIRED fuzz_block_seeds)
    endif()
endif()
