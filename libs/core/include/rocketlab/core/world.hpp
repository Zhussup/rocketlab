#pragma once

#include <optional>
#include <vector>

#include "rocketlab/core/body.hpp"
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

  /// The entity's state in the root frame, which is what a renderer wants.
  [[nodiscard]] StateVector root_state(const Entity& entity) const;

  /// Advances simulation time by `sim_dt`, never moving the clock in one jump
  /// larger than `max_step`.
  ///
  /// The cap is not an accuracy limit — the propagation is analytic and exact
  /// for any step — it is the granularity at which events are noticed. A
  /// sphere-of-influence crossing is only detected when a step ends, so the
  /// cap bounds how far past the boundary an entity can travel unnoticed.
  void advance(Seconds sim_dt, Seconds max_step = 60.0);

 private:
  void step(Seconds dt);
  void reframe_escaped_entities(Seconds tdb);

  BodySystem bodies_;
  SimClock clock_;
  std::vector<Entity> entities_;
  EntityId next_id_{1};
};

}  // namespace rocketlab::core
