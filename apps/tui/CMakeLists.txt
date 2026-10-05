FetchContent_Declare(ftxui
  GIT_REPOSITORY https://github.com/ArthurSonzogni/FTXUI.git
  GIT_TAG v6.1.9
  GIT_SHALLOW TRUE)

# Only the screen and DOM modules: the extra components are a widget library we
# do not use, and not building them keeps configure time down.
set(FTXUI_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(FTXUI_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(FTXUI_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(ftxui)

add_executable(rocketlab_tui
  main.cpp
  canvas_backend.cpp)

target_link_libraries(rocketlab_tui
  PRIVATE
    rocketlab::core
    rocketlab::scenario
    rocketlab::simhost
    rocketlab::render
    rocketlab::hud
    ftxui::screen
    ftxui::dom
    ftxui::component
    rocketlab_flags)
