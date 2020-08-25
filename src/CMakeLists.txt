add_library(
    skybrush
    error.c
    poly.c
    trajectory.c
    formats/binary.c
)

target_include_directories(
    skybrush
    PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

install(TARGETS skybrush)
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/skybrush DESTINATION include)


