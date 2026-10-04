#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>
#include <vector>

#include "rocketlab/core/world.hpp"
#include "rocketlab/proto/sim_source.hpp"

namespace rocketlab::simhost {

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

  /// True once a Quit command has arrived.
  [[nodiscard]] bool quit_requested() const noexcept { return quit_; }

  /// Samples a trajectory query is resolved into.
  static constexpr std::size_t kTrajectorySamples = 256;

 private:
  void publish();
  void apply(const proto::Command& command);
  void cycle_target(int direction);

  core::World world_;
  core::FrameTimer timer_;
  proto::Snapshot snapshot_{};
  core::Seconds max_step_{60.0};
  bool quit_{false};
};

}  // namespace rocketlab::simhost
