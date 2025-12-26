add_library(
    skybrush

    buffer.c
    crc32.c
    error.c
    parsing.c
    utils.c

    conductor/time_axis.c

    events/events.c

    formats/binary.c

    lights/colors.c
    lights/error_handler.cpp
    lights/executor.cpp
    lights/loop_stack.cpp
    lights/program.cpp
    lights/transition.cpp
    lights/trigger.cpp

    rth_plan/rth_plan.c

    trajectory/builder.c
    trajectory/poly.c
    trajectory/trajectory.c
    trajectory/stats.c

    yaw_control/yaw_control.c
)

set_property(TARGET skybrush PROPERTY C_STANDARD 99)

target_include_directories(
    skybrush
    PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_compile_options(
    skybrush
    PRIVATE
    -Wdouble-promotion
)

# The line below is not okay; it overwrites the installed library every time
# we run "make install", even if it did not change. As a result, ArduCopter
# rebuilds itself all the time when libskybrush is used as a dependency.
# install(TARGETS skybrush)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/libskybrush.a DESTINATION lib)
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/skybrush DESTINATION include)

if(LIBSKYBRUSH_BUILD_TOOLS)
    add_subdirectory(tools)
endif()
