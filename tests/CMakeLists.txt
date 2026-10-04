FetchContent_Declare(Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG v3.7.1
  GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(Catch2)

add_executable(rocketlab_tests
  test_main.cpp
  test_orbital.cpp
  test_time.cpp)

target_link_libraries(rocketlab_tests
  PRIVATE
    rocketlab::core
    Catch2::Catch2WithMain
    rocketlab_flags)

include(${catch2_SOURCE_DIR}/extras/Catch.cmake)
catch_discover_tests(rocketlab_tests)
