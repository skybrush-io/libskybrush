function(add_unity_test NAME)
    add_executable(test_${NAME} test_${NAME}.c)
    target_link_libraries(test_${NAME} PUBLIC skybrush unity)
    add_test(${NAME} test_${NAME})
endfunction()

function(add_fixture NAME)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/fixtures/${NAME} ${CMAKE_CURRENT_BINARY_DIR}/fixtures/${NAME} COPYONLY)
endfunction()

add_fixture(test.skyb)
add_fixture(hover_3m.skyb)

add_unity_test(binary_parser)
add_unity_test(colors)
add_unity_test(light_program)
add_unity_test(light_player)
add_unity_test(poly)
add_unity_test(trajectory)
add_unity_test(trajectory_player)

