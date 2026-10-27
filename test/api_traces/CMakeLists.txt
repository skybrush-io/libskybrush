# Golden traces of the API-level behaviour of the library: what it computes
# from the show files, as opposed to what can be read from them (see the
# traces of the file format). The traces are stored in this directory and
# the show files are taken from the test fixtures.
add_executable(generate_api_trace generate_api_trace.c ../traces/trace_output.c)
target_link_libraries(generate_api_trace PUBLIC skybrush)

set(API_TRACE_FIXTURES
    basic_show_with_yaw
    forward_left_back
    forward_left_back_v2
    hover_3m_with_gcs_control_block
    hover_3m_with_rth_plan
    light_program_with_wait_until_cmd
    multiple_vertical_landing_segments
    pyro_events
    real_show
)
set(API_TRACE_SHOW_FILES)
foreach(name ${API_TRACE_FIXTURES})
    list(APPEND API_TRACE_SHOW_FILES "${PROJECT_SOURCE_DIR}/test/fixtures/${name}.skyb")
endforeach()

find_program(LIBSKYBRUSH_PYTHON3 NAMES python3 python)
if(LIBSKYBRUSH_PYTHON3)
    add_test(
        NAME api_traces
        COMMAND ${LIBSKYBRUSH_PYTHON3} ${PROJECT_SOURCE_DIR}/test/traces/compare_traces.py
            --generator $<TARGET_FILE:generate_api_trace>
            --traces ${CMAKE_CURRENT_SOURCE_DIR}
            --show-files ${API_TRACE_SHOW_FILES}
    )

    # Regenerates the golden API traces with the current version of the library
    add_custom_target(update_api_traces
        COMMAND ${LIBSKYBRUSH_PYTHON3} ${PROJECT_SOURCE_DIR}/test/traces/compare_traces.py
            --generator $<TARGET_FILE:generate_api_trace>
            --traces ${CMAKE_CURRENT_SOURCE_DIR}
            --show-files ${API_TRACE_SHOW_FILES}
            --update
        DEPENDS generate_api_trace
        COMMENT "Regenerating the golden API traces"
    )
endif()

add_executable(generate_api_scenarios generate_api_scenarios.c ../traces/trace_output.c)
target_link_libraries(generate_api_scenarios PUBLIC skybrush)

if(LIBSKYBRUSH_PYTHON3)
    add_test(
        NAME api_scenarios
        COMMAND ${LIBSKYBRUSH_PYTHON3} ${PROJECT_SOURCE_DIR}/test/traces/compare_traces.py
            --generator $<TARGET_FILE:generate_api_scenarios>
            --traces ${CMAKE_CURRENT_SOURCE_DIR}
            --standalone scenarios
    )

    # Regenerates the golden trace of the scenarios
    add_custom_target(update_api_scenarios
        COMMAND ${LIBSKYBRUSH_PYTHON3} ${PROJECT_SOURCE_DIR}/test/traces/compare_traces.py
            --generator $<TARGET_FILE:generate_api_scenarios>
            --traces ${CMAKE_CURRENT_SOURCE_DIR}
            --standalone scenarios
            --update
        DEPENDS generate_api_scenarios
        COMMENT "Regenerating the golden trace of the API scenarios"
    )
endif()
