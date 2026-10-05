// A flight computer written in Lua.
//
// Three properties, and each is a design decision rather than a feature:
//
//   * Sandboxed. The interpreter is built with no `io`, no `os`, no `package`,
//     no `debug`, no `dofile`/`loadfile`/`require` and no `print`, and it
//     allocates through an allocator that refuses to go past a byte cap. A
//     script can therefore neither touch the host's filesystem nor exhaust its
//     memory, and there is no path from a scenario file the user was handed to
//     code that runs outside this class.
//
//   * Deterministic. The same script on the same state produces the same
//     commands, the same number of executed instructions and the same error, on
//     any machine. That rules out `math.random` (which is libc's `rand`, and
//     whose sequence differs between implementations), the clock and the
//     filesystem, all of which are removed rather than merely discouraged. It
//     is why `Input` carries a mission time and not a timestamp.
//
//   * Bounded. A count hook charges the script for the VM instructions it
//     executes and raises once it passes the budget, so `while true do end`
//     faults the program instead of hanging the daemon. The budget is per tick,
//     which bounds the cost of a tick rather than the length of a mission.
//
// Note what is *not* here: no `lua.h` in this header. The interpreter is an
// implementation detail behind a pimpl, so a translation unit that merely holds
// a `LuaProgram` does not pay for Lua's headers and cannot accidentally start
// using them.

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>

#include "rocketlab/flight/program.hpp"

namespace rocketlab::flight {

class LuaProgram final : public Program {
 public:
  /// Opaque. Declared here only so that the definition in the `.cpp` can be a
  /// member of this class, which is what lets it reach the private constructor
  /// below; the type is incomplete everywhere else, so nobody can do anything
  /// with it but hold a pointer.
  struct Impl;

  struct Limits {
    /// VM instructions one `update` call may execute before it is faulted.
    ///
    /// A control law that reads a dozen numbers and sets a throttle uses a few
    /// hundred; a hundred thousand is three orders of magnitude of headroom,
    /// and still costs well under a millisecond.
    std::uint64_t instructions_per_tick{100000};
    /// Ceiling on the bytes the interpreter may hold [B], enforced by the
    /// allocator. A script that builds an unbounded table hits this rather than
    /// the host's OOM killer.
    std::size_t memory_bytes{1U << 20U};
  };

  /// Compiles `source`. On failure returns null and fills `error` with the
  /// compiler's message, prefixed with `chunk_name`.
  [[nodiscard]] static std::unique_ptr<LuaProgram> compile(std::string_view source,
                                                           std::string_view chunk_name,
                                                           const Limits& limits, std::string& error);

  /// Reads and compiles a file. Split from `compile` so that a caller that has
  /// the source in hand — a test, or the assembly editor later on — need not go
  /// through the filesystem at all.
  [[nodiscard]] static std::unique_ptr<LuaProgram> compile_file(const std::string& path,
                                                                const Limits& limits,
                                                                std::string& error);

  ~LuaProgram() override;
  LuaProgram(const LuaProgram&) = delete;
  LuaProgram& operator=(const LuaProgram&) = delete;

  bool update(const Input& in, Output& out) override;
  [[nodiscard]] std::unique_ptr<Program> clone() const override;
  [[nodiscard]] Status status() const noexcept override;
  [[nodiscard]] std::string_view message() const noexcept override;
  [[nodiscard]] std::uint64_t instructions() const noexcept override;

  /// Instructions the most recent tick used, which is the number a script
  /// author tuning a control law actually wants. The running total above is for
  /// the budget readout.
  [[nodiscard]] std::uint64_t last_tick_instructions() const noexcept;

 private:
  explicit LuaProgram(std::unique_ptr<Impl> impl);

  std::unique_ptr<Impl> impl_;
};

}  // namespace rocketlab::flight
