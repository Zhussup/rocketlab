// Client -> daemon commands.
//
// Deliberately a small, closed set. A client can change what is being watched
// and how fast time runs, and can ask questions; it cannot reach into the
// simulation. Every command is plain data so it can be queued through a ring
// buffer in shared memory.

#pragma once

#include <cstdint>

namespace rocketlab::proto {

enum class CommandKind : std::uint8_t {
  /// `value` is the new warp factor. 0 pauses.
  SetWarp,
  /// `value` is +1 or -1: step to the next rung of the warp ladder.
  StepWarp,
  /// `target` is the entity to centre the camera on. 0 means "none".
  SelectTarget,
  /// Cycle the selection to the next (+1) or previous (-1) controllable
  /// entity, which is what the tab key sends.
  CycleTarget,
  /// Remove an entity. `target` is its id.
  Remove,
  /// Drop the entity's current stage. `target` is its id; 0 means the
  /// selection.
  Stage,
  /// Set the throttle of an entity, 0..1. `target` as for `Stage`, `value` is
  /// the throttle.
  SetThrottle,
  /// Enable (`value` non-zero) or disable the entity's flight computer.
  /// `target` as for `Stage`. Disabling leaves the vessel exactly as it is —
  /// on whatever throttle the script last set — so a pilot can take over from
  /// a program mid-burn without the engine cutting out from under them.
  SetComputer,
  /// Join the entity `target` onto the currently selected one, which survives.
  ///
  /// The selection is the survivor because it is the one the camera is on and
  /// the one being flown: a pilot lines up with a target, not the other way
  /// round. `target` of 0, or an entity that is not there, does nothing.
  ///
  /// The host applies the same predicate a client used to decide whether to
  /// offer the command, and declines quietly when it fails — see
  /// `core::dock_block`, which is where the numbers live.
  Dock,
  /// Stop the daemon.
  Quit,
};

struct Command {
  CommandKind kind{CommandKind::SetWarp};
  std::uint8_t pad[7]{};
  double value{0.0};
  std::uint64_t target{0};

  [[nodiscard]] static Command set_warp(double factor) noexcept {
    Command command;
    command.kind = CommandKind::SetWarp;
    command.value = factor;
    return command;
  }
  [[nodiscard]] static Command step_warp(int direction) noexcept {
    Command command;
    command.kind = CommandKind::StepWarp;
    command.value = static_cast<double>(direction);
    return command;
  }
  [[nodiscard]] static Command select(std::uint64_t id) noexcept {
    Command command;
    command.kind = CommandKind::SelectTarget;
    command.target = id;
    return command;
  }
  [[nodiscard]] static Command cycle_target(int direction) noexcept {
    Command command;
    command.kind = CommandKind::CycleTarget;
    command.value = static_cast<double>(direction);
    return command;
  }
  [[nodiscard]] static Command stage(std::uint64_t id = 0) noexcept {
    Command command;
    command.kind = CommandKind::Stage;
    command.target = id;
    return command;
  }
  [[nodiscard]] static Command set_throttle(double throttle, std::uint64_t id = 0) noexcept {
    Command command;
    command.kind = CommandKind::SetThrottle;
    command.value = throttle;
    command.target = id;
    return command;
  }
  [[nodiscard]] static Command set_computer(bool enabled, std::uint64_t id = 0) noexcept {
    Command command;
    command.kind = CommandKind::SetComputer;
    command.value = enabled ? 1.0 : 0.0;
    command.target = id;
    return command;
  }
  [[nodiscard]] static Command dock(std::uint64_t id) noexcept {
    Command command;
    command.kind = CommandKind::Dock;
    command.target = id;
    return command;
  }
};

}  // namespace rocketlab::proto
