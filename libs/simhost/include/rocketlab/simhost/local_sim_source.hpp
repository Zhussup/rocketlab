#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "rocketlab/core/world.hpp"
#include "rocketlab/flight/lua_program.hpp"
#include "rocketlab/proto/sim_source.hpp"

namespace rocketlab::simhost {

/// A flight computer attached to one entity.
///
/// The program is held here rather than in the `World` for two reasons. The
/// first is that a `World` gets copied whenever a trajectory is predicted, and
/// a Lua interpreter cannot be copied — putting one inside would have made the
/// copy impossible rather than merely expensive. The second is where the thing
/// belongs: a flight computer is part of the host, on the same side of the
/// `SimSource` boundary as the physics, and the `World` is the physics.
struct ComputerAttachment {
  core::EntityId entity{core::kInvalidEntity};
  std::unique_ptr<flight::Program> program;
  /// Where it was loaded from, for the readout and for re-cloning.
  std::string path;
  /// Mirrored into the snapshot so a client can tell a script that is flying
  /// from a script that has given up.
  flight::Status status{flight::Status::Idle};
  std::string message;

  /// Instructions executed since the program started, or zero with none
  /// attached. A readout of it is the only way to see that a script is spending
  /// its whole budget and is about to be stopped.
  [[nodiscard]] std::uint64_t instructions() const noexcept {
    return program != nullptr ? program->instructions() : 0;
  }
};

/// A SimSource backed by a World living in this process.
///
/// It exists so the client layer can be built and tested without spawning a
/// daemon, and so `apps/simd` and `apps/tui` share one implementation of the
/// publishing logic. It is not a licence for a client to touch the World: the
/// World is private here, and a client holding a SimSource sees exactly what
/// it would see talking to a daemon.
class LocalSimSource final : public proto::SimSource {
 public:
  /// The scenario's epoch becomes the clock's start, and the first controllable
  /// entity becomes the initial camera target.
  explicit LocalSimSource(core::World world);

  /// Builds the world and loads any flight computers the scenario names.
  ///
  /// Throws `std::runtime_error` if a named script will not compile or does not
  /// define `update()`. That is deliberate and matches what an unknown part
  /// name does: a script that cannot be loaded is a typo in the scenario, and a
  /// vessel that silently flew on without its autopilot would be a mystery
  /// discovered an hour into the mission rather than at the moment of loading.
  [[nodiscard]] static LocalSimSource from_scenario(const core::Scenario& scenario);

  [[nodiscard]] const proto::Snapshot& snapshot() const override;
  void pump(double wall_dt) override;
  void send(const proto::Command& command) override;
  [[nodiscard]] bool query_trajectory(std::uint64_t id, double horizon,
                                     std::vector<proto::Vec3d>& out) const override;
  [[nodiscard]] std::string_view describe() const override;

  /// Largest simulation span between event checks, in seconds.
  void set_max_step(core::Seconds step) noexcept { max_step_ = step; }
  [[nodiscard]] core::Seconds max_step() const noexcept { return max_step_; }

  /// How often flight computers are run [s].
  ///
  /// A control law that ran only once per `max_step` would be steering on
  /// minute-old information during a burn, which is not a control law. The step
  /// is therefore shortened to this period whenever a program is attached, so
  /// that what a script decides is applied for at most this long. It is also
  /// what makes a scripted flight reproducible: the decisions depend on the
  /// control rate and on nothing else, so a client that asks for a different
  /// `max_step` gets the same mission.
  void set_control_period(core::Seconds period) noexcept { control_period_ = period; }
  [[nodiscard]] core::Seconds control_period() const noexcept { return control_period_; }

  /// Compiles `path` and attaches it to an entity, replacing any program
  /// already there. Returns false and fills `error` if it will not load.
  bool attach_script(core::EntityId id, const std::string& path, std::string& error);

  void detach_script(core::EntityId id);
  [[nodiscard]] bool has_script(core::EntityId id) const noexcept;

  /// Stops every attached program from being run, without unloading any of
  /// them. A disabled program keeps its own state, so re-enabling resumes its
  /// plan rather than restarting it.
  void set_computers_enabled(bool enabled) noexcept { computers_enabled_ = enabled; }
  [[nodiscard]] bool computers_enabled() const noexcept { return computers_enabled_; }

  [[nodiscard]] const std::vector<ComputerAttachment>& computers() const noexcept {
    return computers_;
  }

  /// Budgets new programs are compiled with. Existing ones keep the limits they
  /// were built under, so raising this does not retroactively change a script
  /// that is already flying.
  void set_script_limits(const flight::LuaProgram::Limits& limits) noexcept {
    script_limits_ = limits;
  }
  [[nodiscard]] const flight::LuaProgram::Limits& script_limits() const noexcept {
    return script_limits_;
  }

  /// True once a Quit command has arrived.
  [[nodiscard]] bool quit_requested() const noexcept { return quit_; }

  /// Samples a trajectory query is resolved into.
  static constexpr std::size_t kTrajectorySamples = 256;

 private:
  void publish();
  void apply(const proto::Command& command);
  void cycle_target(int direction);
  /// Runs every attached program once against the world as it stands.
  void run_computers();
  /// Advances by `span`, running the computers at the control rate on the way.
  void advance_with_control(core::Seconds span);

  core::World world_;
  core::FrameTimer timer_;
  proto::Snapshot snapshot_{};
  /// Simulation time the mission started at, so a flight computer can be told
  /// how long it has been flying rather than what the Julian date is.
  core::Seconds epoch_tdb_{0.0};
  core::Seconds max_step_{60.0};
  core::Seconds control_period_{1.0};
  std::vector<ComputerAttachment> computers_;
  flight::LuaProgram::Limits script_limits_{};
  bool computers_enabled_{true};
  bool quit_{false};
};

}  // namespace rocketlab::simhost
