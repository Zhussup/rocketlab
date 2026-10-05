#pragma once

#include <optional>
#include <vector>

#include "rocketlab/core/body.hpp"
#include "rocketlab/core/docking.hpp"
#include "rocketlab/core/entity.hpp"
#include "rocketlab/core/scenario.hpp"
#include "rocketlab/core/time.hpp"

namespace rocketlab::core {

/// The simulation: a body tree, a clock, and the entities moving through it.
///
/// The world owns everything and knows nothing about who is watching. There is
/// no input handling, no rendering and no networking in here — clients attach
/// to it from the outside.
class World {
 public:
  explicit World(BodySystem bodies);

  [[nodiscard]] static World from_scenario(const Scenario& scenario);

  [[nodiscard]] const BodySystem& bodies() const noexcept { return bodies_; }
  [[nodiscard]] SimClock& clock() noexcept { return clock_; }
  [[nodiscard]] const SimClock& clock() const noexcept { return clock_; }

  /// Returns the id assigned to the stored entity.
  EntityId add(Entity entity);

  /// Returns false if there was no such entity.
  bool remove(EntityId id);

  [[nodiscard]] const Entity* find(EntityId id) const noexcept;
  [[nodiscard]] Entity* find(EntityId id) noexcept;

  [[nodiscard]] const std::vector<Entity>& entities() const noexcept { return entities_; }

  /// Drops the entity's current stage, leaving the dropped parts behind as a new
  /// piece of debris sharing the old state.
  ///
  /// The debris matters: a spent booster that simply vanished would make the
  /// orbit of everything around it a lie, and the whole point of the parts model
  /// is that the mass that went up is the mass that has to be accounted for.
  /// Returns the id of the debris, or `kInvalidEntity` when there was nothing to
  /// drop.
  EntityId jettison_stage(EntityId id);

  /// Joins `absorbed` onto `target`. The target survives and keeps its id; the
  /// absorbed entity is gone.
  ///
  /// Returns the surviving id, or `kInvalidEntity` when the attempt was
  /// refused — `dock_block` on the same pair says why, and the client is
  /// expected to have asked it already, so a refusal here means the two
  /// disagreed about the state rather than about the rules.
  ///
  /// Mass and momentum are conserved; the position is not moved. See
  /// `dock_vessels` for what happens to the two stacks.
  EntityId dock(EntityId target, EntityId absorbed,
                const DockingLimits& limits = kDefaultDockingLimits);

  /// The entity's state in the root frame, which is what a renderer wants.
  [[nodiscard]] StateVector root_state(const Entity& entity) const;

  /// Advances simulation time by `sim_dt`, never moving the clock in one jump
  /// larger than `max_step`.
  ///
  /// The cap is not an accuracy limit — the propagation is analytic and exact
  /// for any step — it is the granularity at which events are noticed, and the
  /// interval a sphere-of-influence crossing is solved for within. A crossing
  /// is placed exactly, by bisection, but a step that contains two of them
  /// still resolves only the first.
  void advance(Seconds sim_dt, Seconds max_step = 60.0);

 private:
  void step(Seconds dt);

  /// Moves one entity across `dt`, analytically when it is coasting through
  /// vacuum and numerically when it is under thrust or in air.
  void propagate_entity(Entity& entity, Seconds dt) const;

  /// One entity under thrust or drag. Gravity is no longer the only force, so
  /// the analytic propagator cannot be used and the state is integrated.
  void integrate(Entity& entity, const CelestialBody& body, Seconds dt, bool drag) const;

  /// The instant within a step at which the entity stops belonging to the body
  /// that currently dominates it.
  [[nodiscard]] Seconds soi_crossing(const Entity& start, Seconds tdb0, Seconds dt) const;

  /// Re-expresses the entity's state against `to`, exactly.
  void reframe(Entity& entity, BodyId to, Seconds tdb) const;

  [[nodiscard]] StateVector root_state_at(const Entity& entity, Seconds tdb) const;

  /// Anything the refinement did not already re-frame.
  void reframe_escaped_entities(Seconds tdb);

  BodySystem bodies_;
  SimClock clock_;
  std::vector<Entity> entities_;
  EntityId next_id_{1};

  /// The state an entity had at the start of the step being taken.
  ///
  /// It exists so that a step which turns out to straddle a sphere-of-
  /// influence boundary can be undone and flown again in two pieces, and it is
  /// a member rather than a local so that its buffers are reused instead of
  /// reallocated on every step of every entity.
  Entity scratch_;
};

}  // namespace rocketlab::core
