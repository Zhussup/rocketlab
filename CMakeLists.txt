cmake_minimum_required(VERSION 3.24)

# C as well as C++, because the Lua interpreter is C and there is no C++
# translation unit that could stand in for it.
project(rocketlab
  VERSION 0.1.0
  DESCRIPTION "Space mission simulator"
  LANGUAGES C CXX)

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
option(ROCKETLAB_BUILD_GUI "Build the GLFW/ImGui client" OFF)
option(ROCKETLAB_WERROR "Treat warnings as errors" OFF)

# GLFW needs the X11 extension headers, which are not installed system-wide on
# this machine. A local extraction of the Debian -dev packages under
# ~/.local/opt/x11dev is searched when it exists; see NOTES.md for how it was
# built. On a machine with libxrandr-dev and friends installed properly this
# block does nothing and the system paths are used.
if(ROCKETLAB_BUILD_GUI AND EXISTS "$ENV{HOME}/.local/opt/x11dev/usr")
  list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/.local/opt/x11dev/usr")
  list(APPEND CMAKE_INCLUDE_PATH "$ENV{HOME}/.local/opt/x11dev/usr/include")
  list(APPEND CMAKE_LIBRARY_PATH
    "$ENV{HOME}/.local/opt/x11dev/usr/lib/x86_64-linux-gnu")
  message(STATUS "Using the local X11 development prefix under ~/.local/opt/x11dev")
endif()

# --- shared compiler flags -------------------------------------------------
add_library(rocketlab_flags INTERFACE)
target_compile_options(rocketlab_flags INTERFACE
  $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall;-Wextra;-Wpedantic;-Wshadow;-Wnon-virtual-dtor>)
if(ROCKETLAB_WERROR)
  target_compile_options(rocketlab_flags INTERFACE
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Werror>)
endif()

include(FetchContent)

FetchContent_Declare(nlohmann_json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG v3.11.3
  GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(nlohmann_json)

add_subdirectory(libs/core)
add_subdirectory(libs/scenario)
add_subdirectory(libs/proto)
add_subdirectory(libs/flight)
add_subdirectory(libs/simhost)
add_subdirectory(libs/render)
add_subdirectory(libs/hud)

add_subdirectory(apps/cli)

if(ROCKETLAB_BUILD_TUI)
  add_subdirectory(apps/tui)
endif()

if(ROCKETLAB_BUILD_GUI)
  add_subdirectory(apps/gui)
endif()

if(ROCKETLAB_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
