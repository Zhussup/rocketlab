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
};

}  // namespace rocketlab::proto
