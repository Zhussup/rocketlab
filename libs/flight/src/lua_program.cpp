#include "rocketlab/flight/lua_program.hpp"

#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

// Lua's headers carry no `extern "C"` guard of their own — `lua.hpp`, which
// used to provide one, was dropped in 5.4 — so C++ callers are expected to wrap
// them. Hence the standard headers first: the wrapper has to cover Lua's
// declarations, and it must not cover `<stdarg.h>`, `<stddef.h>` or
// `<stdio.h>`, which lauxlib.h pulls in. Including them here means Lua's
// includes are already satisfied and there is nothing left inside the block
// but Lua.
#include <cstdarg>
#include <cstddef>
#include <cstdio>

extern "C" {
#include "lauxlib.h"
#include "lua.h"
#include "lualib.h"
}

namespace rocketlab::flight {

/// The interpreter, and everything that has to travel with it.
///
/// Declared here rather than inside the anonymous namespace below for a
/// mundane reason: it is a member of `LuaProgram`, and a nested class has to be
/// defined in a scope that encloses its class. The anonymous namespace is
/// *inside* this one, so defining it there is a different (and wrong) thing.
///
/// The ledger is reached from inside Lua callbacks with `lua_getallocf`, which
/// is the one pointer guaranteed to travel with the `lua_State` into every hook
/// and every C function. A global would make two programs in one process share
/// a ledger; this does not.
struct LuaProgram::Impl {
  Impl() = default;
  Impl(const Impl&) = delete;
  Impl& operator=(const Impl&) = delete;
  /// Owns the interpreter, so that every early return out of `compile` — of
  /// which there are four — closes it without each having to remember to.
  ~Impl() {
    if (L != nullptr) {
      lua_close(L);
      L = nullptr;
    }
  }

  lua_State* L{nullptr};
  std::string source;
  std::string chunk_name;
  Limits limits;

  Status status{Status::Idle};
  std::string message;

  std::uint64_t total_instructions{0};
  std::uint64_t tick_instructions{0};
  /// Set by the hook when it raises, so a reader can tell a budget overrun from
  /// a genuine script error without parsing the message.
  bool budget_exceeded{false};

  std::size_t bytes{0};

  /// The state under which this tick's script runs. Valid only inside a tick.
  const Input* input{nullptr};
  /// What the script has asked for so far this tick.
  Output pending;
  bool abort_requested{false};
};

namespace {

/// Globals removed before any script code runs.
///
/// These all live in `lbaselib.c` alongside `error`, `pcall` and `type`, so they
/// cannot be left out at link time the way the four libraries below are.
/// `dofile`, `loadfile` and `load` are the other ways to reach a file or a
/// string of code; `print` writes to a stream a daemon has no business writing
/// to; `collectgarbage` lets a script ask for a collection at a moment of its
/// choosing, which is a way to make a run's memory behaviour depend on
/// something other than the run, and it can also shrink the ledger the memory
/// cap is counted in. `log` is offered in place of `print` and goes to the
/// program's own message buffer.
///
/// The names for the libraries that are not linked at all are listed here too.
/// They are already absent, and saying so anyway means a reader does not have
/// to check the build file to find out.
constexpr const char* kRemovedGlobals[] = {
    "io",    "os",    "package", "debug",  "coroutine", "dofile", "loadfile",
    "load",  "require", "print", "collectgarbage", "warn",
};

/// Removed as well, as a sub-table.
///
/// `math.random` is libc's `rand`, whose sequence is not specified by C and
/// differs between implementations. A simulation that is supposed to reproduce
/// bit for bit cannot have it. `math.randomseed` goes with it so that the
/// absence is unambiguous rather than a call that silently does nothing.
constexpr const char* kRemovedMathFields[] = {"random", "randomseed"};

/// `coroutine` is removed too, and for a subtler reason than the rest.
///
/// Hooks are per-thread: `lua_sethook` arms the budget on one `lua_State`, and
/// `coroutine.create` makes another one. A script that moved its work into a
/// coroutine would therefore run without the count hook ever firing, which
/// turns the instruction budget from a guarantee into a suggestion and makes
/// `while true do end` inside a coroutine hang the daemon. There is no way to
/// arm a hook on a thread that does not exist yet, so the only sound answer is
/// not to offer coroutines at all.

using Impl = LuaProgram::Impl;

[[nodiscard]] Impl* impl_of(lua_State* L) {
  void* ud = nullptr;
  (void)lua_getallocf(L, &ud);
  return static_cast<Impl*>(ud);
}

/// Allocator that refuses to let the interpreter past `limits.memory_bytes`.
///
/// Lua turns a null return into its own "not enough memory" error, which is
/// raised where the allocation was attempted and is therefore catchable by the
/// same `pcall` that catches everything else. A script that builds an unbounded
/// table hits this; without it, it would hit the host's memory instead.
void* limited_alloc(void* ud, void* ptr, std::size_t osize, std::size_t nsize) {
  auto* impl = static_cast<Impl*>(ud);

  if (nsize == 0) {
    // Free. `osize` is the size that was allocated.
    impl->bytes = impl->bytes > osize ? impl->bytes - osize : 0;
    std::free(ptr);
    return nullptr;
  }

  if (ptr == nullptr) {
    // Fresh allocation. `osize` carries an object-type tag here, not a size, so
    // treating it as one would corrupt the ledger.
    if (impl->bytes + nsize > impl->limits.memory_bytes) {
      return nullptr;
    }
    void* fresh = std::malloc(nsize);
    if (fresh != nullptr) {
      impl->bytes += nsize;
    }
    return fresh;
  }

  // Resize. `osize` is the old size, so only growth is charged, and a shrink
  // gives memory back rather than leaving the ledger creeping upward forever.
  if (nsize > osize) {
    if (impl->bytes + (nsize - osize) > impl->limits.memory_bytes) {
      return nullptr;
    }
  }
  void* resized = std::realloc(ptr, nsize);
  if (resized == nullptr) {
    return nullptr;
  }
  if (nsize > osize) {
    impl->bytes += nsize - osize;
  } else {
    impl->bytes = impl->bytes > (osize - nsize) ? impl->bytes - (osize - nsize) : 0;
  }
  return resized;
}

/// Counts VM instructions and raises once one tick has spent its allowance.
///
/// The hook is armed with a count of one, so the number it reports is the
/// number of instructions actually executed rather than a rounded-off
/// multiple. A control law runs a few hundred instructions a tick and a tick
/// happens at control rate, so the exactness costs nothing worth having.
void budget_hook(lua_State* L, lua_Debug* /*ar*/) {
  Impl* impl = impl_of(L);
  ++impl->total_instructions;
  ++impl->tick_instructions;
  if (impl->tick_instructions > impl->limits.instructions_per_tick) {
    impl->budget_exceeded = true;
    // Formatted into a plain buffer rather than passed to `luaL_error` as
    // varargs. Lua's own formatter understands `%s %d %f %p %c %I %U %%` and
    // nothing else, so a `%llu` reaches the script as "invalid option '%l' to
    // 'lua_pushfstring'" — an error message about the error message. The buffer
    // is also POD, which matters here: `luaL_error` does not return, and
    // unwinding a C++ temporary across it would leak.
    char text[128];
    std::snprintf(text, sizeof(text),
                  "flight computer exceeded its budget of %llu instructions in one tick",
                  static_cast<unsigned long long>(impl->limits.instructions_per_tick));
    luaL_error(L, "%s", text);
  }
}

void set_number(lua_State* L, const char* field, double value) {
  lua_pushnumber(L, value);
  lua_setfield(L, -2, field);
}

void set_integer(lua_State* L, const char* field, lua_Integer value) {
  lua_pushinteger(L, value);
  lua_setfield(L, -2, field);
}

/// The direction the attitude request resolves to right now.
[[nodiscard]] core::Vec3 prograde_of(const Input& in) noexcept {
  return normalized(in.state.v);
}

[[nodiscard]] core::Vec3 radial_of(const Input& in) noexcept {
  return normalized(in.state.r);
}

[[nodiscard]] core::Vec3 normal_of(const Input& in) noexcept {
  return normalized(cross(in.state.r, in.state.v));
}

// --- the `ship` table ------------------------------------------------------
//
// The whole scripting surface. Everything here is either a read-only number
// refreshed by the host before each call, or a request appended to the tick's
// pending output. Nothing is applied while the script is running, so a script
// cannot observe its own side effects halfway through and take a different
// branch depending on them.

int l_set_throttle(lua_State* L) {
  const double throttle = static_cast<double>(luaL_checknumber(L, 1));
  Impl* impl = impl_of(L);
  impl->pending.throttle = throttle < 0.0 ? 0.0 : (throttle > 1.0 ? 1.0 : throttle);
  return 0;
}

int l_jettison(lua_State* L) {
  impl_of(L)->pending.stage = true;
  return 0;
}

int l_point(lua_State* L) {
  Impl* impl = impl_of(L);
  const double x = static_cast<double>(luaL_checknumber(L, 1));
  const double y = static_cast<double>(luaL_checknumber(L, 2));
  const double z = static_cast<double>(luaL_optnumber(L, 3, 0.0));
  impl->pending.thrust_dir = core::Vec3{x, y, z};
  impl->pending.set_attitude = true;
  return 0;
}

/// Shared body of the named attitudes. Each resolves against the state the
/// host handed over at the start of the tick, which is the only state the
/// script can see — so a script asking for prograde gets the direction that was
/// prograde when it was asked.
template <core::Vec3 (*Resolve)(const Input&)>
int l_point_named(lua_State* L) {
  Impl* impl = impl_of(L);
  impl->pending.thrust_dir = Resolve(*impl->input);
  impl->pending.set_attitude = true;
  return 0;
}

int l_point_prograde(lua_State* L) {
  // Zero is the prograde convention the whole program already speaks. Setting
  // it to the velocity vector would freeze the burn along today's velocity
  // instead of following it, which is a different manoeuvre.
  Impl* impl = impl_of(L);
  impl->pending.thrust_dir = core::Vec3{};
  impl->pending.set_attitude = true;
  return 0;
}

int l_point_retrograde(lua_State* L) {
  Impl* impl = impl_of(L);
  impl->pending.thrust_dir = -prograde_of(*impl->input);
  impl->pending.set_attitude = true;
  return 0;
}

int l_point_radial(lua_State* L) { return l_point_named<radial_of>(L); }
int l_point_antradial(lua_State* L) {
  Impl* impl = impl_of(L);
  impl->pending.thrust_dir = -radial_of(*impl->input);
  impl->pending.set_attitude = true;
  return 0;
}
int l_point_normal(lua_State* L) { return l_point_named<normal_of>(L); }
int l_point_antinormal(lua_State* L) {
  Impl* impl = impl_of(L);
  impl->pending.thrust_dir = -normal_of(*impl->input);
  impl->pending.set_attitude = true;
  return 0;
}

/// Stops the program for good. The vessel stays flyable by hand.
int l_abort(lua_State* L) {
  Impl* impl = impl_of(L);
  impl->abort_requested = true;
  if (lua_gettop(L) >= 1 && lua_isstring(L, 1)) {
    impl->message = lua_tostring(L, 1);
  } else {
    impl->message = "stopped by the script";
  }
  return 0;
}

/// The script's own debugging channel. Appends to the program's message buffer,
/// newest first, so the reason a script did something survives to be read.
int l_log(lua_State* L) {
  Impl* impl = impl_of(L);
  // Counted before the buffer exists: `luaL_buffinit` pushes a placeholder onto
  // the stack, so asking afterwards would count that as an argument and log a
  // stray `userdata: 0x...` after every line.
  const int n = lua_gettop(L);

  luaL_Buffer buffer;
  luaL_buffinit(L, &buffer);
  for (int i = 1; i <= n; ++i) {
    std::size_t length = 0;
    const char* text = luaL_tolstring(L, i, &length);
    if (i > 1) {
      luaL_addchar(&buffer, '\t');
    }
    if (text != nullptr) {
      luaL_addlstring(&buffer, text, length);
    }
    lua_pop(L, 1);  // the string luaL_tolstring pushed
  }
  luaL_pushresult(&buffer);

  const char* text = lua_tostring(L, -1);
  impl->message = text != nullptr ? text : "";
  // Back to the arguments the caller passed, so the placeholder the buffer may
  // have left behind does not become part of this function's return.
  lua_settop(L, n);
  return 0;
}

/// The standard libraries a script is given, and by omission the ones it is not.
///
/// `luaL_openlibs` is not called and `linit.c` is not compiled in, because four
/// of the libraries it would open are ones this project has no use for and two
/// of them are a hole in the sandbox:
///
///   * `debug` can reach the registry and call `debug.sethook`, which would let
///     a script disarm its own instruction budget. A budget a script can switch
///     off is not a budget.
///   * `os` and `io` are the clock and the filesystem.
///   * `package` is `require`, and with it `loadlib`'s dynamic loader.
///   * `coroutine` would run without the count hook, since hooks are per-thread.
///
/// Leaving them out at link time is worth more than deleting them at run time:
/// the code is not merely unreachable from a script, it is not in the binary.
/// What remains of the removal below is the handful of base-library functions
/// that share `lbaselib.c` with `error` and `pcall` and so cannot be dropped
/// without the rest of the file.
void open_libraries(lua_State* L) {
  const luaL_Reg libraries[] = {
      {LUA_GNAME, luaopen_base},           {LUA_TABLIBNAME, luaopen_table},
      {LUA_STRLIBNAME, luaopen_string},    {LUA_MATHLIBNAME, luaopen_math},
      {LUA_UTF8LIBNAME, luaopen_utf8},     {nullptr, nullptr},
  };
  for (const luaL_Reg* library = libraries; library->func != nullptr; ++library) {
    luaL_requiref(L, library->name, library->func, 1);
    lua_pop(L, 1);
  }
}

/// Rebuilds the globals, and the `ship` table with them.
///
/// Called on every `lua_State` this class creates, so a clone is sandboxed by
/// the same code that sandboxed the original rather than by a copy of it that
/// might have drifted.
void install_sandbox(lua_State* L) {
  open_libraries(L);

  for (const char* name : kRemovedGlobals) {
    lua_pushnil(L);
    lua_setglobal(L, name);
  }
  lua_getglobal(L, "math");
  if (lua_istable(L, -1)) {
    for (const char* field : kRemovedMathFields) {
      lua_pushnil(L);
      lua_setfield(L, -2, field);
    }
  }
  lua_pop(L, 1);
}

const luaL_Reg kShipFunctions[] = {
    {"set_throttle", l_set_throttle},   {"jettison", l_jettison},
    {"point", l_point},                 {"point_prograde", l_point_prograde},
    {"point_retrograde", l_point_retrograde},
    {"point_radial", l_point_radial},   {"point_antradial", l_point_antradial},
    {"point_normal", l_point_normal},   {"point_antinormal", l_point_antinormal},
    {"abort", l_abort},                 {"log", l_log},
    {nullptr, nullptr},
};

/// Pushes the `ship` table, creating it if the script has thrown it away.
void push_ship(lua_State* L) {
  lua_getglobal(L, "ship");
  if (lua_istable(L, -1)) {
    return;
  }
  lua_pop(L, 1);

  lua_newtable(L);
  luaL_setfuncs(L, kShipFunctions, 0);
  lua_pushvalue(L, -1);
  lua_setglobal(L, "ship");
}

/// Writes this tick's telemetry into `ship`.
///
/// Angles are radians and distances metres, as everywhere else. Nothing here
/// is a prediction: every field is either read straight off the state or is an
/// osculating element of it.
void refresh_telemetry(lua_State* L, const Impl& impl) {
  const Input& in = *impl.input;
  push_ship(L);
  const int ship = lua_gettop(L);

  set_number(L, "met", in.met);
  set_number(L, "mu", in.mu);
  set_number(L, "body_radius", in.body_radius);
  set_number(L, "altitude", in.altitude);
  set_number(L, "speed", norm(in.state.v));
  // The distance from the centre of the parent, so that `altitude` is exactly
  // `radius - body_radius`. Not the vessel's own size, which no control law has
  // ever wanted and which would put a second meaning on the same word.
  set_number(L, "radius", norm(in.state.r));

  const core::OrbitalElements& el = in.elements;
  lua_pushboolean(L, el.degenerate ? 0 : 1);
  lua_setfield(L, ship, "elements_valid");

  // Every element field is written even when the state is degenerate. The flag
  // above is the honest signal; leaving stale numbers behind a flag is how a
  // script ends up steering by last revolution's apoapsis.
  set_number(L, "periapsis", el.degenerate ? 0.0 : el.periapsis());
  set_number(L, "apoapsis", el.degenerate || el.e >= 1.0 ? 0.0 : el.apoapsis());
  set_number(L, "eccentricity", el.e);
  set_number(L, "inclination", el.i);
  set_number(L, "period", el.degenerate || el.e >= 1.0 ? 0.0 : el.period(in.mu));
  set_number(L, "time_to_apoapsis", in.time_to_apoapsis);
  set_number(L, "time_to_periapsis", in.time_to_periapsis);

  if (in.vessel != nullptr) {
    const core::Vessel& vessel = *in.vessel;
    set_integer(L, "stage_index", vessel.current_stage);
    set_integer(L, "stage_count", vessel.stage_count);
    set_number(L, "mass", vessel.mass());
    set_number(L, "propellant", vessel.propellant_left());
    set_number(L, "propellant_capacity", vessel.propellant_capacity());
    set_number(L, "stage_propellant", vessel.stage_propellant(vessel.current_stage));
    set_number(L, "thrust", vessel.thrust());
    set_number(L, "delta_v", remaining_delta_v(vessel));
    lua_pushboolean(L, vessel.can_thrust() ? 1 : 0);
    lua_setfield(L, ship, "has_thrust");
    lua_pushboolean(L, vessel.empty() ? 0 : 1);
    lua_setfield(L, ship, "has_parts");
  } else {
    set_integer(L, "stage_index", 0);
    set_integer(L, "stage_count", 0);
    set_number(L, "mass", 0.0);
    set_number(L, "propellant", 0.0);
    set_number(L, "propellant_capacity", 0.0);
    set_number(L, "stage_propellant", 0.0);
    set_number(L, "thrust", 0.0);
    set_number(L, "delta_v", 0.0);
    lua_pushboolean(L, 0);
    lua_setfield(L, ship, "has_thrust");
    lua_pushboolean(L, 0);
    lua_setfield(L, ship, "has_parts");
  }

  lua_pop(L, 1);
}

/// The body of one tick, run under a `pcall` so that anything raised anywhere
/// in it — including by the allocation cap while refreshing telemetry — lands
/// in the caller as an error string instead of escaping to Lua's panic handler
/// and taking the daemon with it.
int tick_entry(lua_State* L) {
  auto* impl = static_cast<Impl*>(lua_touserdata(L, 1));
  refresh_telemetry(L, *impl);

  lua_getglobal(L, "update");
  if (!lua_isfunction(L, -1)) {
    return luaL_error(L, "%s: no update() function to call",
                      impl->chunk_name.empty() ? "script" : impl->chunk_name.c_str());
  }
  lua_call(L, 0, 0);
  return 0;
}

/// Prepends a Lua traceback to an error message. Which line of a control law
/// went wrong is the first thing its author wants and the last thing a bare
/// message gives.
int traceback_handler(lua_State* L) {
  const char* text = lua_tostring(L, 1);
  if (text == nullptr) {
    lua_pushliteral(L, "flight computer: error object is not a string");
    return 1;
  }
  luaL_traceback(L, L, text, 1);
  return 1;
}

/// Records an error and stops the program for good.
void fault(Impl& impl, std::string text) {
  impl.status = Status::Faulted;
  impl.message = std::move(text);
}

}  // namespace

const char* to_string(Status status) noexcept {
  switch (status) {
    case Status::Idle:
      return "idle";
    case Status::Running:
      return "running";
    case Status::Finished:
      return "finished";
    case Status::Faulted:
      return "faulted";
  }
  return "unknown";
}

LuaProgram::LuaProgram(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

LuaProgram::~LuaProgram() = default;

std::unique_ptr<LuaProgram> LuaProgram::compile(std::string_view source, std::string_view chunk_name,
                                                const Limits& limits, std::string& error) {
  auto impl = std::make_unique<Impl>();
  impl->source = std::string(source);
  impl->chunk_name = std::string(chunk_name);
  impl->limits = limits;

  // A floor on the cap. The interpreter's own libraries are loaded before any
  // script code runs, and they need some tens of kilobytes; a cap below that
  // would not fault a script politely, it would fail while opening the standard
  // library and reach Lua's panic handler, which aborts. Refusing to go below
  // the floor is better than an abort that looks like a crash in the host.
  constexpr std::size_t kMinimumMemory = 256U << 10U;
  if (impl->limits.memory_bytes < kMinimumMemory) {
    impl->limits.memory_bytes = kMinimumMemory;
  }

  impl->L = lua_newstate(limited_alloc, impl.get());
  if (impl->L == nullptr) {
    error = "flight computer: could not create an interpreter";
    return nullptr;
  }
  lua_State* L = impl->L;
  install_sandbox(L);

  // The budget is armed before a single line of the script runs, so a chunk
  // that loops at load time has to pay for it too — the top level of a Lua file
  // is code like any other.
  lua_sethook(L, budget_hook, LUA_MASKCOUNT, 1);

  std::string text = impl->source;
  if (text.empty() || text.back() != '\n') {
    text.push_back('\n');
  }
  if (luaL_loadbufferx(L, text.data(), text.size(), impl->chunk_name.c_str(), "t") != LUA_OK) {
    error = lua_tostring(L, -1) != nullptr ? lua_tostring(L, -1) : "flight computer: syntax error";
    lua_pop(L, 1);
    return nullptr;
  }

  impl->status = Status::Idle;
  auto program = std::unique_ptr<LuaProgram>(new LuaProgram(std::move(impl)));
  // `impl` is moved-from from here down; the object it pointed at is still
  // alive, owned by `program`, and this is the pointer to it.
  Impl* live = program->impl_.get();

  // Run the chunk once, at load, so that a syntax-clean but broken script fails
  // where its author is looking rather than an hour into a mission.
  if (lua_pcall(L, 0, 0, 0) != LUA_OK) {
    const char* message = lua_tostring(L, -1);
    error = message != nullptr ? message : "flight computer: error while loading";
    lua_pop(L, 1);
    return nullptr;
  }

  lua_getglobal(L, "update");
  const bool has_update = lua_isfunction(L, -1) != 0;
  lua_pop(L, 1);
  if (!has_update) {
    error = std::string(chunk_name) + ": no update() function is defined";
    return nullptr;
  }

  // Loading a chunk is itself code and its instructions were genuinely spent,
  // but the counters are a mission readout and a script author counting them
  // means "since it started flying", not "including the file it was read from".
  live->total_instructions = 0;
  live->tick_instructions = 0;

  return program;
}

std::unique_ptr<LuaProgram> LuaProgram::compile_file(const std::string& path, const Limits& limits,
                                                     std::string& error) {
  std::ifstream file(path);
  if (!file) {
    error = "flight computer: cannot read '" + path + "'";
    return nullptr;
  }
  std::ostringstream contents;
  contents << file.rdbuf();
  if (!file.good() && !file.eof()) {
    error = "flight computer: error reading '" + path + "'";
    return nullptr;
  }
  return compile(contents.str(), path, limits, error);
}

bool LuaProgram::update(const Input& in, Output& out) {
  Impl& impl = *impl_;
  if (impl.status == Status::Faulted || impl.status == Status::Finished) {
    return false;
  }

  impl.input = &in;
  impl.pending = Output{};
  impl.abort_requested = false;
  impl.budget_exceeded = false;
  impl.tick_instructions = 0;

  lua_State* L = impl.L;
  const int base = lua_gettop(L);

  lua_pushcfunction(L, traceback_handler);
  lua_pushcfunction(L, tick_entry);
  lua_pushlightuserdata(L, &impl);
  const int rc = lua_pcall(L, 1, 0, base + 1);

  if (rc != LUA_OK) {
    const char* text = lua_tostring(L, -1);
    fault(impl, text != nullptr ? text : "flight computer: unknown error");
    lua_settop(L, base);
    impl.input = nullptr;
    return false;
  }
  lua_settop(L, base);

  impl.input = nullptr;
  out = impl.pending;

  if (impl.abort_requested) {
    impl.status = Status::Finished;
    return false;
  }
  impl.status = Status::Running;
  return true;
}

std::unique_ptr<Program> LuaProgram::clone() const {
  std::string error;
  std::unique_ptr<LuaProgram> copy =
      LuaProgram::compile(impl_->source, impl_->chunk_name, impl_->limits, error);
  // A program that compiled once compiles again: nothing between the two calls
  // can change the source or the sandbox. A null here would be a bug in this
  // file rather than a bad script, and quietly returning null would turn it
  // into a mystery at the call site.
  if (copy == nullptr) {
    return nullptr;
  }
  return copy;
}

Status LuaProgram::status() const noexcept { return impl_->status; }

std::string_view LuaProgram::message() const noexcept { return impl_->message; }

std::uint64_t LuaProgram::instructions() const noexcept { return impl_->total_instructions; }

std::uint64_t LuaProgram::last_tick_instructions() const noexcept { return impl_->tick_instructions; }

}  // namespace rocketlab::flight
