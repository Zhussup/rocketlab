cmake_minimum_required(VERSION 3.24)

project(rocketlab
  VERSION 0.1.0
  DESCRIPTION "Space mission simulator"
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel)
endif()

option(ROCKETLAB_BUILD_TESTS "Build unit tests" ON)
option(ROCKETLAB_BUILD_TUI "Build the FTXUI terminal client" OFF)
option(ROCKETLAB_BUILD_GUI "Build the GUI client" OFF)
option(ROCKETLAB_WERROR "Treat warnings as errors" OFF)

# --- shared compiler flags -------------------------------------------------
add_library(rocketlab_flags INTERFACE)
target_compile_options(rocketlab_flags INTERFACE
  $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall;-Wextra;-Wpedantic;-Wshadow;-Wnon-virtual-dtor>)
if(ROCKETLAB_WERROR)
  target_compile_options(rocketlab_flags INTERFACE
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Werror>)
endif()

include(FetchContent)
add_subdirectory(libs/core)

if(ROCKETLAB_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
