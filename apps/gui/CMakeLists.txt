# The graphical client: a GLFW window with Dear ImGui drawing the same scene the
# terminal client draws, from the same camera, through libs/render.
#
# Neither dependency leaks past this directory. libs/render stays
# renderer-agnostic and the scene it builds is handed to ImDrawList here, which
# is the fourth backend for the same primitives (SVG, characters, FTXUI canvas,
# ImDrawList).

FetchContent_Declare(glfw
  GIT_REPOSITORY https://github.com/glfw/glfw.git
  GIT_TAG 3.4
  GIT_SHALLOW TRUE)

# GLFW's own tests and examples pull in a lot of code that would never run here,
# and its documentation build needs doxygen.
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)

# 3.4 turns on Wayland as well as X11, and its Wayland half needs
# wayland-scanner plus the wayland protocol packages, none of which are here.
# This build is X11 only, which is what the environment has.
set(GLFW_BUILD_WAYLAND OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_X11 ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(glfw)

# Dear ImGui has no build system of its own, so its sources are listed here.
# Note what is *not* here: no extra OpenGL loader. Since 1.82 the OpenGL 3
# backend carries its own `imgui_impl_opengl3_loader.h`, which is one fewer
# dependency and one fewer thing to keep in step with the driver.
FetchContent_Declare(imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG v1.91.5
  GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(imgui)

add_library(imgui_backends STATIC
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp)

target_include_directories(imgui_backends PUBLIC
  ${imgui_SOURCE_DIR}
  ${imgui_SOURCE_DIR}/backends)
target_link_libraries(imgui_backends PUBLIC glfw)

# ImGui is third-party code and does not build clean under this project's
# warning set, so it is compiled without it rather than with a pile of
# exceptions to a rule that exists for our code.
target_compile_options(imgui_backends PRIVATE -w)
add_library(rocketlab::imgui ALIAS imgui_backends)

find_package(OpenGL REQUIRED)

add_executable(rocketlab_gui
  main.cpp
  imgui_backend.cpp)

target_link_libraries(rocketlab_gui
  PRIVATE
    rocketlab::core
    rocketlab::scenario
    rocketlab::simhost
    rocketlab::render
    rocketlab::hud
    rocketlab::imgui
    OpenGL::GL
    rocketlab_flags)

# GLFW dlopens and dlcloses the X11 libraries at run time.
if(UNIX AND NOT APPLE)
  target_link_libraries(rocketlab_gui PRIVATE ${CMAKE_DL_LIBS})
endif()
