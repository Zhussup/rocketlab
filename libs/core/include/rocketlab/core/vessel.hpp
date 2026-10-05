#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "rocketlab/core/orbital.hpp"
#include "rocketlab/core/part.hpp"

namespace rocketlab::core {

/// One part in a stack, and the stage it is jettisoned with.
struct StackPart {
  std::string part;
  int stage{0};
};

/// What left the vessel when a stage was dropped.
///
/// The remaining propellant comes with it, because a stage that still had fuel
/// in it is heavier debris than one that did not, and the world has to be able to
/// say by how much.
struct JettisonedStage {
  std::vector<Part> parts;
  double propellant{0.0};  // [kg] still in its tanks at separation

  [[nodiscard]] bool empty() const noexcept { return parts.empty(); }
  [[nodiscard]] double mass() const noexcept;
  [[nodiscard]] double radius() const noexcept;
};

/// A vessel: the parts it is made of, resolved against the catalogue, and the
/// propellant it has left.
///
/// Staging is by number, and stage 0 fires first. Everything tagged 0 is the
/// booster: its engines light at the start and the whole group — engines, tanks
/// and all — leaves when the stage is jettisoned. A stage's engines are fed by
/// the tanks that carry the same number, so a stage with tanks but no engine is
/// a coast stage, and a stage with an engine but no tank has nothing to burn.
///
/// Parts are held in stack order, top first, because that is the order a human
/// reads a rocket in. Nothing in the arithmetic depends on it.
struct Vessel {
  std::vector<Part> parts;
  std::vector<int> stage;         // parallel to `parts`
  std::vector<double> propellant; // remaining per part [kg]
  int stage_count{0};
  int current_stage{0};

  /// 0..1. Zero means the engines are off, and there is no separate "engine on"
  /// flag to get out of step with it.
  double throttle{0.0};

  /// Thrust direction in the parent body frame. A zero vector means "along the
  /// velocity": a prograde burn, which is what a client that has not said
  /// otherwise means, and what a flight computer will override.
  Vec3 thrust_dir{};

  [[nodiscard]] bool empty() const noexcept { return parts.empty(); }

  /// Current mass: structure plus whatever propellant is still aboard [kg].
  [[nodiscard]] double mass() const noexcept;
  [[nodiscard]] double propellant_left() const noexcept;
  [[nodiscard]] double propellant_capacity() const noexcept;

  /// What the tanks tagged `stage` still hold [kg].
  [[nodiscard]] double stage_propellant(int stage) const noexcept;

  /// Removes up to `kilograms` from the current stage's tanks, spread over them
  /// in proportion to what each holds. Returns what was actually taken, which is
  /// less than asked for when the stage is nearly dry — the caller must use the
  /// returned figure, or it will burn propellant the vessel does not have.
  double draw_propellant(double kilograms) noexcept;

  /// The widest part, and the sum of the lengths [m]. Used for rendering.
  [[nodiscard]] double radius() const noexcept;
  [[nodiscard]] double length() const noexcept;

  /// Total vacuum thrust of the current stage's engines at the current throttle
  /// [N]. Zero when the stage has no engine, or when its own tanks are dry: an
  /// engine is fed by the tanks of its own stage, so a stage that carries no
  /// tank of its own has nothing to burn.
  [[nodiscard]] double thrust() const noexcept;

  /// Thrust-weighted mean exhaust velocity of the current stage [m/s]. Mixing
  /// engines of different Isp this way is what makes the combined stage behave
  /// as one engine of the same total impulse.
  [[nodiscard]] double exhaust_velocity() const noexcept;

  /// True when the current stage can produce thrust right now.
  [[nodiscard]] bool can_thrust() const noexcept;

  /// Drops the current stage: its parts leave the vessel and the next stage
  /// becomes current. Empty when there was nothing left to drop, which is the
  /// case for the last populated stage — a vessel always keeps a core to fly.
  [[nodiscard]] JettisonedStage jettison();
};

/// The vessel that results from `absorbed` joining `target`.
///
/// The arriving vessel's parts are put at the top of the stack and numbered one
/// stage band above everything the target already had, and its tanks keep
/// whatever they were carrying. Both follow from the same fact: the absorbed
/// vessel is not part of the target's staging. It does not fire with the
/// target's booster and it is not dropped with it, so it cannot share its stage
/// numbers, and the only place a stack can put something that is neither is
/// above it.
///
/// The consequence worth knowing: docking is not symmetric. `a` absorbing `b`
/// leaves `a` flying its own stages first, and `b`'s stages are what is left
/// afterwards.
[[nodiscard]] Vessel dock_vessels(const Vessel& target, const Vessel& absorbed);

/// What one stage weighs, how hard it pushes, and how much velocity it is worth.
///
/// The masses are the ones Tsiolkovsky's equation wants: `ignition_mass` is the
/// whole vessel at the moment this stage lights, `final_mass` the same vessel
/// once this stage's propellant is gone and before anything is dropped.
struct StageReport {
  int index{0};
  double dry_mass{0.0};       // [kg] after this stage's propellant is spent
  double propellant{0.0};     // [kg] this stage burns
  double thrust{0.0};         // [N] at full throttle
  double isp{0.0};            // [s] thrust-weighted
  double ignition_mass{0.0};  // [kg]
  double final_mass{0.0};     // [kg]
  double delta_v{0.0};        // [m/s]
  double burn_time{0.0};      // [s] at full throttle, or infinite with no thrust
};

/// The stage table of a vessel with full tanks, in firing order.
[[nodiscard]] std::vector<StageReport> stage_table(const Vessel& vessel);

/// Sum of `stage_table`'s delta-v [m/s]. The number a mission planner wants
/// before launch.
[[nodiscard]] double total_delta_v(const Vessel& vessel);

/// What the stages still attached are worth given the propellant actually
/// left, including the stage currently burning.
///
/// Recomputed from the tanks rather than tracked down as it is spent, so it
/// cannot drift from the fuel it is supposed to describe.
[[nodiscard]] double remaining_delta_v(const Vessel& vessel);

/// Resolves a stack against the catalogue and fills the tanks.
///
/// Throws `std::invalid_argument` on a part the catalogue does not have: a
/// scenario that names a part that does not exist is a typo, and a vessel that
/// silently weighed nothing would fly beautifully.
[[nodiscard]] Vessel build_vessel(const std::vector<StackPart>& stack,
                                  const PartCatalogue& catalogue);

}  // namespace rocketlab::core
