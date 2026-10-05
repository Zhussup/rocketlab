#include "rocketlab/core/world.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "rocketlab/core/constants.hpp"

namespace rocketlab::core {
namespace {

constexpr double kDegreesToRadians = 0.017453292519943295;

/// Largest integration step taken while an engine is lit [s].
///
/// Chosen so that RK4's truncation error stays far below anything measurable
/// while a burn of a few minutes costs a few hundred steps. A thrust arc is
/// short — minutes against orbits of hours — so propagating one is not what
/// makes time warp expensive, and there is no reason to be clever about it.
constexpr double kThrustSubstep = 2.0;

/// Largest integration step taken in air [s].
///
/// Tighter than the thrust step because the density is what varies: it falls
/// off over a scale height, so a step that spans several scale heights would
/// integrate a force that had already gone away. One second is comfortably
/// inside the shortest scale height in the catalogue (6 km, at the Earth's
/// surface, against 7.7 km/s in low orbit), and this simulator models decay
/// rather than entry, so the violent few minutes where even that is too coarse
/// are not in scope.
constexpr double kAtmosphereSubstep = 1.0;

/// Drag coefficient of a blunt body. 2.2 is the textbook value for a flat
/// plate normal to the flow and the usual figure for free-molecule flow; the
/// answer moves by a few per cent for anything in 2.0-2.4, which is well
/// inside the error the exponential atmosphere already carries.
constexpr double kDragCoefficient = 2.2;

constexpr double kPi = 3.141592653589793;

/// The frontal area of a body of mean radius `r` [m^2], which is what the drag
/// term wants. The vessel's widest part is the radius the catalogue keeps, so
/// a stack of tanks presents the disc its tanks make.
constexpr double frontal_area(double radius) noexcept { return kPi * radius * radius; }

/// Halvings used to place a sphere-of-influence crossing inside a step. Forty
/// eight takes a minute down to below the resolution of a double in seconds,
/// and the trajectory is smooth, so there is nothing to gain past it.
constexpr int kCrossingIterations = 48;

/// Position, velocity and mass of one vessel under thrust.
///
/// Mass is carried as a state variable rather than held constant across a step.
/// That is what makes the burn obey the rocket equation exactly: the
/// acceleration uses the mass at each of RK4's four stages, so the velocity
/// change integrates to v_e*ln(m0/m1) rather than to the value you would get by
/// pretending the vessel weighs what it did when the engine lit.
struct BurnState {
  Vec3 r;
  Vec3 v;
  double m{0.0};
};

/// Everything acting on an entity across one step, in the frame of its parent.
///
/// Passed by reference into the derivative rather than captured, because the
/// derivative is evaluated at four different states and drag is the one force
/// here that depends on where the state is: the density falls off with radius,
/// so the drag term has to be recomputed at each of RK4's stages rather than
/// held constant across the step.
struct ForceModel {
  double mu{0.0};
  Vec3 thrust_dir{};
  double thrust{0.0};   // [N]
  double exhaust{0.0};  // [m/s]
  /// The body being orbited, for its atmosphere and its spin. Null when the
  /// step is in vacuum, which is also how drag is switched off.
  const CelestialBody* body{nullptr};
  double drag_cd_area{0.0};  // Cd*A [m^2]
};

[[nodiscard]] BurnState burn_derivative(const BurnState& s, const ForceModel& forces) noexcept {
  const double radius = norm(s.r);
  const double gravity = radius > 0.0 ? -forces.mu / (radius * radius * radius) : 0.0;
  Vec3 acceleration = s.r * gravity;

  if (forces.thrust > 0.0 && s.m > 0.0) {
    acceleration += forces.thrust_dir * (forces.thrust / s.m);
  }

  // Drag pulls along the flow, not along the velocity: the air it is moving
  // through is itself going somewhere, and it is the speed relative to the air
  // that decides the force. The distinction is worth about 6% in low Earth
  // orbit, which is the size of the effect the drag is there to model.
  if (forces.body != nullptr && forces.drag_cd_area > 0.0 && s.m > 0.0) {
    const Vec3 flow = s.v - forces.body->surface_velocity(s.r);
    const double density = forces.body->density_at(s.r);
    if (density > 0.0) {
      acceleration -= flow * (0.5 * density * forces.drag_cd_area * norm(flow) / s.m);
    }
  }

  const double flow_rate = (forces.thrust > 0.0 && forces.exhaust > 0.0)
                               ? -forces.thrust / forces.exhaust
                               : 0.0;
  return BurnState{s.v, acceleration, flow_rate};
}

/// a + b * scale, component-wise.
[[nodiscard]] BurnState combine(const BurnState& a, const BurnState& b, double scale) noexcept {
  return BurnState{a.r + b.r * scale, a.v + b.v * scale, a.m + b.m * scale};
}

[[nodiscard]] BurnState rk4_step(const BurnState& s, double h, const ForceModel& forces) noexcept {
  const BurnState k1 = burn_derivative(s, forces);
  const BurnState k2 = burn_derivative(combine(s, k1, 0.5 * h), forces);
  const BurnState k3 = burn_derivative(combine(s, k2, 0.5 * h), forces);
  const BurnState k4 = burn_derivative(combine(s, k3, h), forces);

  const double sixth = h / 6.0;
  return BurnState{
      s.r + (k1.r + k2.r * 2.0 + k3.r * 2.0 + k4.r) * sixth,
      s.v + (k1.v + k2.v * 2.0 + k3.v * 2.0 + k4.v) * sixth,
      s.m + (k1.m + k2.m * 2.0 + k3.m * 2.0 + k4.m) * sixth,
  };
}

/// The catalogue is the same for every world, so it is built once. It is read
/// only after construction, which is why this needs no locking.
[[nodiscard]] const PartCatalogue& stock_catalogue() {
  static const PartCatalogue catalogue = PartCatalogue::stock();
  return catalogue;
}

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

    // A vessel built from parts weighs what its parts weigh. The scenario's own
    // mass and radius are then ignored rather than added to: two sources for one
    // figure is how a vessel ends up with a mass nobody can explain.
    if (!spec.parts.empty()) {
      entity.vessel = build_vessel(spec.parts, stock_catalogue());
      entity.mass = entity.vessel.mass();
      if (entity.vessel.radius() > 0.0) {
        entity.radius = entity.vessel.radius();
      }
    }
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
  const Seconds tdb0 = clock_.tdb();
  for (Entity& entity : entities_) {
    const BodyId parent = entity.parent;

    // The copy is what makes the crossing exact: the step is undone and flown
    // again in two pieces, and re-running the propagation needs the state it
    // started from. It is only *used* when a boundary is actually crossed, and
    // the member keeps its capacity between steps, so the ordinary step copies
    // bytes and allocates nothing.
    scratch_ = entity;

    propagate_entity(entity, dt);

    if (bodies_.dominant_body(root_state_at(entity, tdb0 + dt).r, tdb0 + dt) == parent) {
      continue;
    }

    // The step straddles a sphere-of-influence boundary. Flying its whole
    // length in the old body's gravity would be wrong in the part that belongs
    // to the new one, so the step is cut at the boundary and flown as two arcs.
    const Seconds crossing = soi_crossing(scratch_, tdb0, dt);
    entity = scratch_;
    propagate_entity(entity, crossing);
    reframe(entity,
            bodies_.dominant_body(root_state_at(entity, tdb0 + crossing).r, tdb0 + crossing),
            tdb0 + crossing);
    propagate_entity(entity, dt - crossing);
  }
  clock_.advance(dt);
  reframe_escaped_entities(clock_.tdb());
}

/// True when the entity's path passes through air during the step.
///
/// Testing the periapsis rather than the current altitude is what makes the
/// switch to numerical integration happen *before* the air is reached. A
/// vessel on a decaying orbit falls hundreds of kilometres per step at high
/// warp, so a test on where it is now would step straight over the whole upper
/// atmosphere and notice nothing. For a hyperbola the periapsis is the closest
/// approach, so the same test covers a flyby that dips in.
[[nodiscard]] bool crosses_atmosphere(const Entity& entity, const CelestialBody& body) {
  if (body.atmosphere.empty() || !(entity.radius > 0.0)) {
    return false;
  }
  const OrbitalElements elements = rv_to_elements(entity.state, body.mu);
  const double periapsis = elements.degenerate ? norm(entity.state.r) : elements.periapsis();
  return periapsis < body.radius + body.atmosphere.layers.back().altitude;
}

void World::propagate_entity(Entity& entity, Seconds dt) const {
  if (!(dt > 0.0)) {
    return;
  }
  const CelestialBody& body = bodies_.body(entity.parent);
  const bool drag = crosses_atmosphere(entity, body);
  if (entity.vessel.can_thrust() || drag) {
    integrate(entity, body, dt, drag);
  } else {
    // Analytic, and exact for any step: a warp factor of 100000 costs what 1
    // costs. Only thrust and air take that away.
    entity.state = propagate(entity.state, body.mu, dt);
  }
}

void World::integrate(Entity& entity, const CelestialBody& body, Seconds dt, bool drag) const {
  Vessel& vessel = entity.vessel;
  const double exhaust = vessel.exhaust_velocity();
  Seconds remaining = dt;

  ForceModel forces;
  forces.mu = body.mu;
  forces.exhaust = exhaust;
  forces.body = drag ? &body : nullptr;
  forces.drag_cd_area = drag ? kDragCoefficient * frontal_area(entity.radius) : 0.0;

  // The direction is held fixed for the whole step rather than recomputed at
  // every substep. A step is at most a minute and a client's commanded direction
  // does not change inside one; recalculating it would make the result depend on
  // how the step happened to be subdivided, which is exactly the kind of
  // dependence that makes a simulation unreproducible.
  Vec3 dir = vessel.thrust_dir;
  if (norm_squared(dir) <= 0.0) {
    dir = normalized(entity.state.v);  // prograde
  }
  forces.thrust_dir = normalized(dir);

  // A burn is smooth enough for the thrust step; air is not, so a step spent in
  // an atmosphere is capped tighter whichever else is going on.
  const double cap = drag ? kAtmosphereSubstep : kThrustSubstep;

  while (remaining > 1e-9) {
    forces.thrust = vessel.thrust();

    double h = std::min(remaining, cap);
    double drawn = 0.0;
    if (forces.thrust > 0.0 && exhaust > 0.0) {
      const double mass_flow = forces.thrust / exhaust;  // [kg/s]
      drawn = vessel.draw_propellant(mass_flow * h);
      if (drawn > 0.0) {
        // The step taken is the one the propellant paid for. Deriving it from
        // the fuel actually burned, rather than from the step that was asked
        // for, is what stops a stage that runs dry mid-step from being credited
        // with thrust it had nothing left to produce.
        h = drawn / mass_flow;
      } else {
        forces.thrust = 0.0;  // the tanks are dry
        drawn = 0.0;
      }
    }
    if (!(h > 1e-9)) {
      break;
    }

    // Nothing acting but gravity: the analytic propagator is exact for any
    // step, so there is no reason to keep integrating once the engines are off
    // and the air is behind. This is also what makes a coast cost the same at
    // a warp factor of 100000 as at 1.
    if (forces.thrust <= 0.0 && !drag) {
      entity.state = propagate(entity.state, body.mu, remaining);
      remaining = 0.0;
      break;
    }

    // Mass is carried as a state variable rather than held constant across a
    // step. That is what makes the burn obey the rocket equation exactly: the
    // acceleration uses the mass at each of RK4's four stages, so the velocity
    // change integrates to v_e*ln(m0/m1) rather than to the value you would get
    // by pretending the vessel weighs what it did when the engine lit.
    const double start_mass = vessel.empty() ? entity.mass : vessel.mass() + drawn;
    const BurnState before{entity.state.r, entity.state.v, start_mass};
    const BurnState after = rk4_step(before, h, forces);
    entity.state.r = after.r;
    entity.state.v = after.v;
    remaining -= h;
  }

  if (!vessel.empty()) {
    entity.mass = vessel.mass();
  }
}

Seconds World::soi_crossing(const Entity& start, Seconds tdb0, Seconds dt) const {
  const BodyId from = start.parent;
  Seconds low = 0.0;
  Seconds high = dt;
  for (int i = 0; i < kCrossingIterations; ++i) {
    const Seconds mid = 0.5 * (low + high);
    Entity probe = start;
    propagate_entity(probe, mid);
    const Vec3 root = root_state_at(probe, tdb0 + mid).r;
    if (bodies_.dominant_body(root, tdb0 + mid) == from) {
      low = mid;
    } else {
      high = mid;
    }
  }
  return high;
}

StateVector World::root_state_at(const Entity& entity, Seconds tdb) const {
  const StateVector frame = bodies_.root_state(entity.parent, tdb);
  return StateVector{frame.r + entity.state.r, frame.v + entity.state.v};
}

void World::reframe(Entity& entity, BodyId to, Seconds tdb) const {
  if (to == entity.parent) {
    return;
  }
  // Recomposing after the switch is exact, so the re-framing introduces no
  // discontinuity in position or velocity.
  const StateVector root = root_state_at(entity, tdb);
  const StateVector frame = bodies_.root_state(to, tdb);
  entity.parent = to;
  entity.state.r = root.r - frame.r;
  entity.state.v = root.v - frame.v;
}

void World::reframe_escaped_entities(Seconds tdb) {
  for (Entity& entity : entities_) {
    // Anything the refinement already handled agrees with `dominant_body` here
    // and costs one comparison.
    reframe(entity, bodies_.dominant_body(root_state_at(entity, tdb).r, tdb), tdb);
  }
}

EntityId World::jettison_stage(EntityId id) {
  Entity* entity = find(id);
  if (entity == nullptr || entity->vessel.empty()) {
    return kInvalidEntity;
  }

  const int dropping = entity->vessel.current_stage;
  JettisonedStage stage = entity->vessel.jettison();
  if (stage.empty()) {
    return kInvalidEntity;
  }
  entity->mass = entity->vessel.mass();

  // The debris leaves with the parent's state at the instant of separation. The
  // separation impulse is not modelled, so the two fly in company until
  // something pushes them apart — which is honest for a spring-loaded decoupler
  // and keeps the mass accounted for.
  Entity debris;
  debris.kind = EntityKind::Debris;
  debris.name = entity->name + " stage " + std::to_string(dropping);
  debris.parent = entity->parent;
  debris.state = entity->state;
  debris.mass = stage.mass();
  debris.radius = stage.radius();
  debris.controllable = false;
  return add(std::move(debris));
}

EntityId World::dock(EntityId target_id, EntityId absorbed_id, const DockingLimits& limits) {
  Entity* target = find(target_id);
  Entity* absorbed = find(absorbed_id);
  if (target == nullptr || absorbed == nullptr) {
    return kInvalidEntity;
  }
  if (dock_block(*target, *absorbed, limits) != DockBlock::None) {
    return kInvalidEntity;
  }

  // Momentum, not just mass. A docking is perfectly inelastic, and the closing
  // speed is capped at a walking pace, so the velocity this produces is within
  // a few metres per second of the target's own — but it is the correct few,
  // and a station catching a probe should barely notice while the reverse
  // should not.
  const double m_target = target->mass;
  const double m_absorbed = absorbed->mass;
  const double total = m_target + m_absorbed;
  const Vec3 velocity = total > 0.0
                            ? (target->state.v * m_target + absorbed->state.v * m_absorbed) / total
                            : target->state.v;

  // The position stays where the target was. Moving it to the barycentre would
  // shift the object the camera is following by a metre for no reason anyone
  // asked for; momentum is what has to be conserved, and it is.
  target->vessel = dock_vessels(target->vessel, absorbed->vessel);
  target->mass = target->vessel.mass();
  if (target->vessel.radius() > 0.0) {
    target->radius = target->vessel.radius();
  }
  target->controllable = target->controllable || absorbed->controllable;
  target->state.v = velocity;

  const EntityId survivor = target->id;
  remove(absorbed_id);
  return survivor;
}

}  // namespace rocketlab::core
