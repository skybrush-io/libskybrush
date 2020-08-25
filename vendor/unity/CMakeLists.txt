add_library(unity STATIC unity.c)
target_compile_definitions(unity PUBLIC UNITY_INCLUDE_DOUBLE)
target_include_directories(unity PUBLIC .)
