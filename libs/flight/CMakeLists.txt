# The flight computer: a Lua interpreter the host owns, plus the interface that
# lets anything else hold one without knowing what it is.
#
# Lua is fetched as source and built here rather than through a package, for the
# same reason as everything else in this project: there is no passwordless sudo
# on this machine, and a simulator whose build depends on what happens to be
# installed is a simulator that stops building. `lua/lua` ships no CMakeLists.txt,
# so `SOURCE_SUBDIR` points at a directory that does not exist — the documented
# way to say "download this, do not add it to the build" — and the sources are
# compiled directly below.
include(FetchContent)

FetchContent_Declare(lua
  GIT_REPOSITORY https://github.com/lua/lua.git
  GIT_TAG v5.4.8
  GIT_SHALLOW TRUE
  SOURCE_SUBDIR cmake-does-not-exist)
FetchContent_MakeAvailable(lua)

file(GLOB rocketlab_lua_sources "${lua_SOURCE_DIR}/*.c")

# The standalone interpreter and its bytecode compiler, which a library wants
# neither of.
list(REMOVE_ITEM rocketlab_lua_sources
  "${lua_SOURCE_DIR}/lua.c"
  "${lua_SOURCE_DIR}/luac.c")
# Lua's own test instrumentation, not part of a release build.
list(REMOVE_ITEM rocketlab_lua_sources "${lua_SOURCE_DIR}/ltests.c")
# The amalgamation that #includes every other file in the directory, so leaving
# it in links the whole interpreter twice.
list(REMOVE_ITEM rocketlab_lua_sources "${lua_SOURCE_DIR}/onelua.c")

# Libraries a flight computer is not given, left out here rather than removed
# from the global table at run time so that the code is not in the binary at
# all. `linit.c` goes with them, because `luaL_openlibs` is the only thing that
# would open them; the rest are opened one at a time in `lua_program.cpp`.
#
#   ldblib.c   debug:  reaches the registry and can call debug.sethook, which
#                      would let a script disarm its own instruction budget
#   liolib.c   io:     the filesystem
#   loslib.c   os:     the clock, the environment, and os.exit
#   loadlib.c  package: require, and with it the dynamic loader
#   lcorolib.c coroutine: hooks are per-thread, so a coroutine would run with
#                      the count hook never firing and the budget would stop
#                      being a bound
list(REMOVE_ITEM rocketlab_lua_sources
  "${lua_SOURCE_DIR}/linit.c"
  "${lua_SOURCE_DIR}/ldblib.c"
  "${lua_SOURCE_DIR}/liolib.c"
  "${lua_SOURCE_DIR}/loslib.c"
  "${lua_SOURCE_DIR}/loadlib.c"
  "${lua_SOURCE_DIR}/lcorolib.c")

add_library(rocketlab_lua STATIC ${rocketlab_lua_sources})
# SYSTEM, because the interpreter is not ours to keep warning-clean and the
# project builds with -Werror. The library is pinned to a tag and fetched
# rather than edited, so there is nothing to fix upstream anyway.
target_include_directories(rocketlab_lua SYSTEM PUBLIC ${lua_SOURCE_DIR})
set_target_properties(rocketlab_lua PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(rocketlab_flight STATIC src/lua_program.cpp)
add_library(rocketlab::flight ALIAS rocketlab_flight)

target_include_directories(rocketlab_flight
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_compile_features(rocketlab_flight PUBLIC cxx_std_20)
target_link_libraries(rocketlab_flight
  PUBLIC
    rocketlab::core
  PRIVATE
    rocketlab_lua
    rocketlab_flags)
