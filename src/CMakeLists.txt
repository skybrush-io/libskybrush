add_library(
    skybrush

    crc32.c
    error.c

    formats/binary.c

    lights/colors.c
    lights/error_handler.cpp
    lights/executor.cpp
    lights/loop_stack.cpp
    lights/program.cpp
    lights/transition.cpp
    lights/trigger.cpp

    trajectory/poly.c
    trajectory/trajectory.c
)

set_property(TARGET skybrush PROPERTY C_STANDARD 99)

target_include_directories(
    skybrush
    PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

# The line below is not okay; it overwrites the installed library every time
# we run "make install", even if it did not change. As a result, ArduCopter
# rebuilds itself all the time when libskybrush is used as a dependency.
# install(TARGETS skybrush)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/libskybrush.a DESTINATION lib)
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/skybrush DESTINATION include)


