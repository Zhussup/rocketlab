add_library(rocketlab_scenario STATIC
  src/json_io.cpp)

target_include_directories(rocketlab_scenario
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

# nlohmann_json is PRIVATE on purpose: it appears in the implementation only,
# so consumers do not inherit its compile time.
target_link_libraries(rocketlab_scenario
  PUBLIC rocketlab::core
  PRIVATE nlohmann_json::nlohmann_json rocketlab_flags)

add_library(rocketlab::scenario ALIAS rocketlab_scenario)
