FetchContent_Declare(Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG v3.7.1
  GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(Catch2)

add_executable(rocketlab_tests
  test_main.cpp
  test_orbital.cpp
  test_time.cpp
  test_world.cpp
  test_atmosphere.cpp
  test_assembly.cpp
  test_docking.cpp
  test_client.cpp
  test_camera3d.cpp
  test_hud.cpp
  test_hud_assembly.cpp
  test_editor.cpp
  test_vessel.cpp
  test_flight.cpp)

# One test loads the scenario in `scenarios/` and the script in `scripts/`, and
# a test that can only find them from one working directory is a test that
# breaks the first time someone runs the binary by hand.
target_compile_definitions(rocketlab_tests
  PRIVATE ROCKETLAB_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

target_link_libraries(rocketlab_tests
  PRIVATE
    rocketlab::core
    rocketlab::scenario
    rocketlab::simhost
    rocketlab::render
    rocketlab::hud
    rocketlab::flight
    Catch2::Catch2WithMain
    rocketlab_flags)

include(${catch2_SOURCE_DIR}/extras/Catch.cmake)
catch_discover_tests(rocketlab_tests)
