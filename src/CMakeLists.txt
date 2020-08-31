add_library(
    skybrush
    error.c
    poly.c
    trajectory.c
    formats/binary.c
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


