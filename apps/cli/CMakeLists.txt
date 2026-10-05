add_executable(rocketlab-cli main.cpp)

target_link_libraries(rocketlab-cli
  PRIVATE
    rocketlab::core
    rocketlab::scenario
    rocketlab::simhost
    rocketlab::render
    rocketlab::hud
    rocketlab_flags)

set_target_properties(rocketlab-cli PROPERTIES OUTPUT_NAME "rocketlab")
