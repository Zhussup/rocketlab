# Snapshot -> human-readable rows. Shared by every client so that two of them
# cannot disagree about what an orbit reads as.
add_library(rocketlab_hud STATIC
  src/readout.cpp)
add_library(rocketlab::hud ALIAS rocketlab_hud)

target_include_directories(rocketlab_hud
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_compile_features(rocketlab_hud PUBLIC cxx_std_20)
target_link_libraries(rocketlab_hud
  PUBLIC
    rocketlab::proto
  PRIVATE
    rocketlab_flags)
