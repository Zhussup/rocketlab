// The seam between a vessel's flight computer and the simulation.
//
// A flight computer is not a client. It runs inside the host, on the host's
// side of the `SimSource` boundary, which is what stops rule 5 from having an
// exception carved out for it: a script sees the same instantaneous state a
// client sees, and its only way to affect the world is to ask for a throttle, a
// stage or an attitude, exactly as a player pressing a key would.
//
// The interface is deliberately narrow and dependency-free. `libs/core` has no
// third-party dependencies and neither does this file; the Lua engine that
// implements it lives behind `LuaProgram` and drags the interpreter with it,
// so anything that only wants to *hold* a program pays for nothing.

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>

#include "rocketlab/core/orbital.hpp"
#include "rocketlab/core/vessel.hpp"

namespace rocketlab::flight {

/// What a flight computer is allowed to know.
///
/// Read-only, instantaneous, and computed by the host before the call. Nothing
/// here is a prediction: anything a script could work out from these numbers is
/// a prediction the script is making, and that is fine, because a flight
/// computer is allowed to plan. A client is not, which is why this struct never
/// crosses the `SimSource` boundary in the other direction.
struct Input {
  /// Seconds since the scenario epoch. Never a wall clock.
  double met{0.0};
  /// Parent-frame state, the frame the physics is valid in.
  core::StateVector state;
  double mu{0.0};
  double body_radius{0.0};
  /// Radius above the body's surface [m].
  double altitude{0.0};
  /// Osculating elements about the parent. `degenerate` marks the states for
  /// which they mean nothing.
  core::OrbitalElements elements;

  /// Seconds until the next apoapsis and periapsis. Derived from the conic, not
  /// from stepping the orbit, and zero for an unbound one — see
  /// `core::time_to_apoapsis`. A control law that has to burn at apoapsis needs
  /// these, and re-deriving them in a script is how two Kepler solvers end up
  /// disagreeing with each other.
  double time_to_apoapsis{0.0};
  double time_to_periapsis{0.0};

  /// The vessel, or null for an entity with no parts. A program attached to a
  /// point mass can still steer, it just has no fuel to burn.
  const core::Vessel* vessel{nullptr};

  [[nodiscard]] bool has_vessel() const noexcept { return vessel != nullptr; }
  [[nodiscard]] bool has_thrust() const noexcept {
    return vessel != nullptr && vessel->can_thrust();
  }
};

/// What it is allowed to ask for. Applied by the host after the call returns,
/// never during it.
struct Output {
  /// 0..1. `throttle` is absolute, not a delta: a script that forgets to set it
  /// every tick coasts, which is visible immediately rather than after the
  /// vessel has quietly flown off course.
  double throttle{0.0};

  /// Request to drop the current stage. Applied after every program has run, so
  /// that adding debris cannot invalidate the iteration that produced it.
  bool stage{false};

  /// Thrust direction in the parent frame, when `set_attitude` says to use it.
  /// A zero vector means "along the velocity" — the same prograde convention
  /// the rest of the program uses.
  core::Vec3 thrust_dir{};

  /// Whether this tick says anything about the attitude at all.
  ///
  /// False — the default, and what a script that only sets the throttle means —
  /// leaves the vessel pointing where it is. That is what a control law which
  /// established an attitude earlier and now only wants to throttle needs, and
  /// it cannot be said by zeroing `thrust_dir`, because zero already means
  /// prograde. A script that wants a prograde burn says `point_prograde`, or
  /// flies one from a vessel that has never pointed anywhere.
  bool set_attitude{false};
};

enum class Status : std::uint8_t {
  Idle,      // never run
  Running,   // ran and asked for nothing it cannot have
  Finished,  // asked to stop, or its `update` disappeared
  Faulted,   // raised an error or ran out of budget; will not be run again
};

[[nodiscard]] const char* to_string(Status status) noexcept;

/// One vessel's flight computer.
class Program {
 public:
  virtual ~Program() = default;
  Program() = default;
  Program(const Program&) = delete;
  Program& operator=(const Program&) = delete;

  /// Runs one control tick. Returns false once the program has stopped for
  /// good, after which the host stops calling it.
  ///
  /// Implementations must not throw: a script that raises is a `Faulted`
  /// program, not a dead daemon.
  virtual bool update(const Input& in, Output& out) = 0;

  /// A fresh copy, in the state it had when this one was loaded.
  ///
  /// Needed because predicting a trajectory means flying a copy of the world
  /// forward, and the honest prediction of a scripted vessel is one that flies
  /// the script's own plan. A clone that could not be made would force the
  /// trajectory query to pretend the vessel coasts, which is a different
  /// future from the one that will happen.
  [[nodiscard]] virtual std::unique_ptr<Program> clone() const = 0;

  [[nodiscard]] virtual Status status() const noexcept = 0;

  /// True once the program has stopped and will not be run again.
  ///
  /// The host checks this to avoid calling a finished program every control
  /// tick for the rest of the mission. It is not the same as `update` returning
  /// false, which is the *moment* it stopped; this is every tick after.
  [[nodiscard]] bool stopped() const noexcept {
    const Status state = status();
    return state == Status::Finished || state == Status::Faulted;
  }

  /// Human-readable detail: the error, the reason it stopped, or the script's
  /// own last log line. Empty when there is nothing to say.
  [[nodiscard]] virtual std::string_view message() const noexcept = 0;

  /// Instructions executed since the program started, for the budget readout.
  [[nodiscard]] virtual std::uint64_t instructions() const noexcept = 0;
};

}  // namespace rocketlab::flight
