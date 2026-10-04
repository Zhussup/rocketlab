add_library(rocketlab_core STATIC
  src/orbital.cpp
  src/time.cpp)

target_include_directories(rocketlab_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(rocketlab_core PRIVATE rocketlab_flags)

add_library(rocketlab::core ALIAS rocketlab_core)
