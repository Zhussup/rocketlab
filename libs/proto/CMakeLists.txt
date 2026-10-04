add_library(rocketlab_proto STATIC
  src/name.cpp)
add_library(rocketlab::proto ALIAS rocketlab_proto)

target_include_directories(rocketlab_proto
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_compile_features(rocketlab_proto PUBLIC cxx_std_20)
target_link_libraries(rocketlab_proto PRIVATE rocketlab_flags)
