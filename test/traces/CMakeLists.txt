# Golden traces of the binary show file format. The show files and their
# traces are maintained together with the specification of the file format;
# the test regenerates the traces with the current version of the library and
# compares them with the golden traces.
set(LIBSKYBRUSH_SKYB_TRACES
    "${PROJECT_SOURCE_DIR}/../skybrush-file-formats-doc/spec/skyb/traces"
    CACHE PATH "Path to the golden traces of the Skybrush binary show file format")

add_executable(generate_trace generate_trace.c)
target_link_libraries(generate_trace PUBLIC skybrush)

find_program(LIBSKYBRUSH_PYTHON3 NAMES python3 python)
if(LIBSKYBRUSH_PYTHON3 AND EXISTS "${LIBSKYBRUSH_SKYB_TRACES}/tolerances.json")
    add_test(
        NAME traces
        COMMAND ${LIBSKYBRUSH_PYTHON3} ${CMAKE_CURRENT_SOURCE_DIR}/compare_traces.py
            --generator $<TARGET_FILE:generate_trace>
            --traces ${LIBSKYBRUSH_SKYB_TRACES}
    )

    # Regenerates the golden traces with the current version of the library
    add_custom_target(update_traces
        COMMAND ${LIBSKYBRUSH_PYTHON3} ${CMAKE_CURRENT_SOURCE_DIR}/compare_traces.py
            --generator $<TARGET_FILE:generate_trace>
            --traces ${LIBSKYBRUSH_SKYB_TRACES}
            --update
        DEPENDS generate_trace
        COMMENT "Regenerating the golden traces of the binary show file format"
    )
elseif(LIBSKYBRUSH_REQUIRE_SPEC_TESTS)
    message(FATAL_ERROR "Golden traces of the binary show file format not found at ${LIBSKYBRUSH_SKYB_TRACES}, or Python is missing")
else()
    message(STATUS "Golden traces of the binary show file format not found; trace tests will be skipped")
endif()
