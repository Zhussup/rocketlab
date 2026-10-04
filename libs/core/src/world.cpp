#include "rocketlab/core/world.hpp"

#include <algorithm>
#include <stdexcept>

namespace rocketlab::core {
namespace {

constexpr double kDegreesToRadians = 0.017453292519943295;

}  // namespace

const char* to_string(EntityKind kind) noexcept {
  switch (kind) {
    case EntityKind::Vessel:
      return "vessel";
    case EntityKind::Debris:
      return "debris";
  }
  return "unknown";
}

StateVector state_from_altitudes(const CelestialBody& body, double periapsis_altitude,
                                 double apoapsis_altitude, double inclination_deg, double raan_deg,
                                 double argp_deg, double true_anomaly_deg) {
  if (periapsis_altitude < 0.0 || apoapsis_altitude < 0.0) {
    throw std::invalid_argument("rocketlab: orbit altitude cannot be negative");
  }
  if (apoapsis_altitude < periapsis_altitude) {
    throw std::invalid_argument("rocketlab: apoapsis cannot be below periapsis");
  }

  const double periapsis = body.radius + periapsis_altitude;
  const double apoapsis = body.radius + apoapsis_altitude;

  OrbitalElements el;
  el.e = (apoapsis - periapsis) / (apoapsis + periapsis);
  el.p = apoapsis * (1.0 - el.e);  // p = a(1 - e^2), and a = (rp + ra) / 2
  el.i = inclination_deg * kDegreesToRadians;
  el.raan = wrap_angle(raan_deg * kDegreesToRadians);
  el.argp = wrap_angle(argp_deg * kDegreesToRadians);
  el.nu = wrap_angle(true_anomaly_deg * kDegreesToRadians);
  return elements_to_rv(el, body.mu);
}

World::World(BodySystem bodies) : bodies_(std::move(bodies)) {}

World World::from_scenario(const Scenario& scenario) {
  World world(BodySystem::solar_system());
  world.clock_.set_tdb(scenario.epoch.tdb);

  for (const ScenarioEntity& spec : scenario.entities) {
    const std::optional<BodyId> parent = world.bodies_.find(spec.parent_body);
    if (!parent.has_value()) {
      throw std::runtime_error("rocketlab: scenario names an unknown body: " + spec.parent_body);
    }
    const CelestialBody& body = world.bodies_.body(*parent);

    Entity entity;
    entity.kind = EntityKind::Vessel;
    entity.name = spec.name;
    entity.parent = *parent;
    entity.state = state_from_altitudes(body, spec.periapsis_altitude, spec.apoapsis_altitude,
                                        spec.inclination_deg, spec.raan_deg, spec.argp_deg,
                                        spec.true_anomaly_deg);
    entity.mass = spec.mass;
    entity.radius = spec.radius;
    entity.controllable = spec.controllable;
    world.add(std::move(entity));
  }
  return world;
}

EntityId World::add(Entity entity) {
  entity.id = next_id_++;
  entities_.push_back(std::move(entity));
  return entities_.back().id;
}

bool World::remove(EntityId id) {
  const auto it = std::find_if(entities_.begin(), entities_.end(),
                               [id](const Entity& e) { return e.id == id; });
  if (it == entities_.end()) {
    return false;
  }
  entities_.erase(it);
  return true;
}

const Entity* World::find(EntityId id) const noexcept {
  const auto it = std::find_if(entities_.begin(), entities_.end(),
                               [id](const Entity& e) { return e.id == id; });
  return it == entities_.end() ? nullptr : &*it;
}

Entity* World::find(EntityId id) noexcept {
  const auto it = std::find_if(entities_.begin(), entities_.end(),
                               [id](const Entity& e) { return e.id == id; });
  return it == entities_.end() ? nullptr : &*it;
}

StateVector World::root_state(const Entity& entity) const {
  const StateVector frame = bodies_.root_state(entity.parent, clock_.tdb());
  return StateVector{frame.r + entity.state.r, frame.v + entity.state.v};
}

void World::advance(Seconds sim_dt, Seconds max_step) {
  if (sim_dt <= 0.0) {
    return;
  }
  if (max_step <= 0.0) {
    max_step = sim_dt;
  }

  Seconds remaining = sim_dt;
  while (remaining > 0.0) {
    const double dt = std::min(remaining, max_step);
    step(dt);
    remaining -= dt;
  }
}

void World::step(Seconds dt) {
  for (Entity& entity : entities_) {
    const double mu = bodies_.body(entity.parent).mu;
    entity.state = propagate(entity.state, mu, dt);
  }
  clock_.advance(dt);
  reframe_escaped_entities(clock_.tdb());
}

void World::reframe_escaped_entities(Seconds tdb) {
  for (Entity& entity : entities_) {
    // root_state is computed from the entity's current parent, which is still
    // the old one at this point. Recomposing after the switch is exact, so the
    // re-framing introduces no discontinuity in position or velocity.
    const StateVector root = root_state(entity);
    const BodyId dominant = bodies_.dominant_body(root.r, tdb);
    if (dominant == entity.parent) {
      continue;
    }
    const StateVector frame = bodies_.root_state(dominant, tdb);
    entity.parent = dominant;
    entity.state.r = root.r - frame.r;
    entity.state.v = root.v - frame.v;
  }
}

}  // namespace rocketlab::core
