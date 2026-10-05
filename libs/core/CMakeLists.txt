add_library(rocketlab_core STATIC
  src/assembly.cpp
  src/atmosphere.cpp
  src/docking.cpp
  src/catalog.cpp
  src/orbital.cpp
  src/part.cpp
  src/time.cpp
  src/vessel.cpp
  src/world.cpp)

target_include_directories(rocketlab_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(rocketlab_core PRIVATE rocketlab_flags)

add_library(rocketlab::core ALIAS rocketlab_core)
