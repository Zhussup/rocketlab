# Scene building: snapshot + camera in, screen-space draw list out. No UI
# toolkit is linked here, which is what lets the TUI, the GUI and the SVG dump
# share one implementation of the map.
add_library(rocketlab_render STATIC
  src/camera.cpp
  src/camera3d.cpp
  src/scene.cpp
  src/svg.cpp
  src/text.cpp)
add_library(rocketlab::render ALIAS rocketlab_render)

target_include_directories(rocketlab_render
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_compile_features(rocketlab_render PUBLIC cxx_std_20)
target_link_libraries(rocketlab_render
  PUBLIC
    rocketlab::proto
  PRIVATE
    rocketlab_flags)
