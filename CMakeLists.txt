# A C library that implements the Skybrush binary trajectory format (and many
# other things related to the Skybrush software suite)

cmake_minimum_required(VERSION 3.10)

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
endif()

project(libskybrush C CXX)

# Write compile_commands.json into the build directory. clangd finds it
# without further configuration if the build directory is called build/
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set the path where we look for additional CMake modules
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/etc/cmake)

# Platform-dependent checks
# include(CheckFunctionExists)
# check_function_exists(fileno HAVE_FILENO)

# Set C and C++ standard levels
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 11)

# Turn on all compiler warnings and ensure that frame pointers are not omitted
# so we can debug properly even when the binary is compiled with optimizations
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Werror -pedantic -Wno-unknown-pragmas -Wno-float-equal -fno-omit-frame-pointer -funwind-tables")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -pedantic -Wno-unknown-pragmas -Wno-float-equal -fno-omit-frame-pointer -funwind-tables")

# Do not fuse multiplications and additions into FMA instructions. Whether the
# compiler does this by default depends on the compiler and the target
# architecture, and the different rounding makes results that are converted to
# integers (e.g., light colors in the middle of a fade) differ by one between
# platforms
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -ffp-contract=off")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffp-contract=off")

# Force colored output when using Ninja
if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fcolor-diagnostics")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fcolor-diagnostics")
elseif (CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fdiagnostics-color=always")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fdiagnostics-color=always")
endif()

# Add configuration for profiling
set(CMAKE_C_FLAGS_PROFILE "-g -pg")
set(CMAKE_CXX_FLAGS_PROFILE "-g -pg")

# Add switches to include sanitizers
include(sanitizers)

# Specify whether supplementary tools should be built
option(LIBSKYBRUSH_BUILD_TOOLS "Build supplementary tools" OFF)

# Check for code coverage support
option(LIBSKYBRUSH_ENABLE_CODE_COVERAGE "Enable code coverage calculation" OFF)
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME AND LIBSKYBRUSH_ENABLE_CODE_COVERAGE)
    include(CodeCoverage)
    append_coverage_compiler_flags()
    set(GCOVR_ADDITIONAL_ARGS --gcov-delete --exclude-lines-by-pattern .*assert.*)
    setup_target_for_coverage_gcovr_html(
    NAME coverage
    EXECUTABLE "${CMAKE_COMMAND}" "--build" "${CMAKE_BINARY_DIR}" "--target" "test"
    EXCLUDE "/Applications/Xcode*" "/Library/Developer*" "test/*" "vendor/unity/*"
  )
endif()

# Use cppcheck for static checks if available
option(LIBSKYBRUSH_ENABLE_CODE_ANALYSIS "Enable code analysis" OFF)
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME AND LIBSKYBRUSH_ENABLE_CODE_ANALYSIS)
    find_program(CPPCHECK cppcheck)
    if(CPPCHECK)
        set(CMAKE_C_CPPCHECK "${CPPCHECK}" "--enable=warning,style" "--inconclusive" "--inline-suppr"
        "--template='{file}:{line}: warning: {id} ({severity}): {message}'")
        set(CMAKE_CXX_CPPCHECK "${CPPCHECK}" "--enable=warning,style" "--inconclusive" "--inline-suppr"
        "--template='{file}:{line}: warning: {id} ({severity}): {message}'")
    endif()
endif()

# Build the fuzz targets with libFuzzer (Clang only). The library itself is
# instrumented for coverage-guided fuzzing as well. Without this option, the
# fuzz targets are built as standalone programs that replay a corpus.
option(LIBSKYBRUSH_ENABLE_FUZZING "Build the fuzz targets with libFuzzer" OFF)
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME AND LIBSKYBRUSH_ENABLE_FUZZING)
    if(NOT CMAKE_C_COMPILER_ID MATCHES "Clang")
        message(FATAL_ERROR "LIBSKYBRUSH_ENABLE_FUZZING requires Clang")
    endif()
    add_compile_options(-fsanitize=fuzzer-no-link)
endif()

# Traverse source code
add_subdirectory(src)
add_subdirectory(vendor)

# Specify whether the tests that need the specification of the binary show
# file format (conformance test cases and golden traces) are mandatory. When
# disabled, these tests are skipped if the specification is not found.
option(LIBSKYBRUSH_REQUIRE_SPEC_TESTS "Fail if the conformance test cases or golden traces are not found" OFF)

# Enable unit test support
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    include(CTest)
    if(BUILD_TESTING)
        add_subdirectory(test)
    endif()
endif()
