function(add_benchmark NAME)
    add_executable(bench_${NAME} bench_${NAME}.c)
    target_link_libraries(bench_${NAME} PUBLIC skybrush)
endfunction()

function(add_fixture NAME)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/../fixtures/${NAME} ${CMAKE_CURRENT_BINARY_DIR}/fixtures/${NAME} COPYONLY)
endfunction()

add_fixture(real_show.skyb)

add_benchmark(get_duration)
add_benchmark(player)
