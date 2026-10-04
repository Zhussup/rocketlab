add_library(rocketlab_simhost STATIC
  src/local_sim_source.cpp)
add_library(rocketlab::simhost ALIAS rocketlab_simhost)

target_include_directories(rocketlab_simhost
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_compile_features(rocketlab_simhost PUBLIC cxx_std_20)
target_link_libraries(rocketlab_simhost
  PUBLIC
    rocketlab::core
    rocketlab::proto
  PRIVATE
    rocketlab_flags)
