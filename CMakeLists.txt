# A C library that implements the Skybrush binary trajectory format (and many
# other things related to the Skybrush software suite)

cmake_minimum_required(VERSION 3.5)

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
endif()

project(libskybrush C CXX)

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
set(CMAKE_C_FLAGS "-Wall -Werror -Wno-unknown-pragmas -fno-omit-frame-pointer -funwind-tables")

# Force colored output when using Ninja
if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fcolor-diagnostics")
elseif (CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fdiagnostics-color=always")
endif()

# Add configuration for profiling
set(CMAKE_C_FLAGS_PROFILE "-g -pg")
set(CMAKE_CXX_FLAGS_PROFILE "-g -pg")

# Add switches to include sanitizers
include(sanitizers)

# Check for code coverage support
option(LIBSKYBRUSH_ENABLE_CODE_COVERAGE "Enable code coverage calculation" OFF)
if(LIBSKYBRUSH_ENABLE_CODE_COVERAGE)
  include(CodeCoverage)
  append_coverage_compiler_flags()
  setup_target_for_coverage_lcov(
    NAME coverage
    EXECUTABLE "${CMAKE_COMMAND}" "--build" "${CMAKE_BINARY_DIR}" "--target" "test"
    EXCLUDE "/Applications/Xcode*" "test/*" "vendor/unity/*"
  )
endif()

# Use cppcheck for static checks if available
option(LIBSKYBRUSH_ENABLE_CODE_ANALYSIS "Enable code analysis" OFF)
if(LIBSKYBRUSH_ENABLE_CODE_ANALYSIS)
  find_program(CPPCHECK cppcheck)
  if(CPPCHECK)
    set(CMAKE_C_CPPCHECK "${CPPCHECK}" "--enable=warning,style" "--inconclusive" "--inline-suppr"
        "--template='{file}:{line}: warning: {id} ({severity}): {message}'")
    set(CMAKE_CXX_CPPCHECK "${CPPCHECK}" "--enable=warning,style" "--inconclusive" "--inline-suppr"
        "--template='{file}:{line}: warning: {id} ({severity}): {message}'")
  endif()
endif()

# Traverse source code
add_subdirectory(src)
add_subdirectory(vendor)

# Enable unit test support
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
  include(CTest)
    if(BUILD_TESTING)
    add_subdirectory(test)
  endif()
endif()

