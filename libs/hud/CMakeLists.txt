# Snapshot -> human-readable rows. Shared by every client so that two of them
# cannot disagree about what an orbit reads as.
#
# Since M6 it also formats a stack being assembled, which is why it links core.
# That does not weaken the client/daemon split: an assembly is a *design*, a
# document of part names, and building one is arithmetic over the catalogue
# rather than simulation state. Rule 5 is about prediction, and nothing here
# predicts anything.
add_library(rocketlab_hud STATIC
  src/assembly.cpp
  src/editor.cpp
  src/readout.cpp)
add_library(rocketlab::hud ALIAS rocketlab_hud)

target_include_directories(rocketlab_hud
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_compile_features(rocketlab_hud PUBLIC cxx_std_20)
target_link_libraries(rocketlab_hud
  PUBLIC
    rocketlab::core
    rocketlab::proto
  PRIVATE
    rocketlab_flags)
