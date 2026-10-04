#include "rocketlab/simhost/local_sim_source.hpp"

#include <algorithm>
#include <cstddef>
#include <utility>

namespace rocketlab::simhost {

namespace {

namespace proto = rocketlab::proto;

[[nodiscard]] proto::Vec3d to_wire(const core::Vec3& v) noexcept {
  return proto::Vec3d{v.x, v.y, v.z};
}

/// Fills the orbit summary fields. Everything here is read off the elements,
/// never propagated, so publishing a frame stays cheap at any warp factor.
void summarise(const core::Entity& entity, const core::CelestialBody& body,
               proto::EntitySnapshot& out) {
  out.periapsis = 0.0;
  out.apoapsis = 0.0;
  out.eccentricity = 0.0;
  out.inclination = 0.0;
  out.period = 0.0;

  const core::OrbitalElements el = core::rv_to_elements(entity.state, body.mu);
  if (el.degenerate) {
    out.flags = out.flags | proto::Flags::Degenerate;
    return;
  }

  out.eccentricity = el.e;
  out.inclination = el.i;
  out.periapsis = el.periapsis();
  out.apoapsis = el.apoapsis();
  out.period = el.period(body.mu);

  // `apoapsis` returns infinity for a parabola and the period is undefined
  // for anything unbound, so those are flagged rather than quietly published
  // as numbers a HUD would then print.
  if (el.e >= 1.0) {
    out.flags = out.flags | proto::Flags::Escaping;
  }
}

}  // namespace

LocalSimSource::LocalSimSource(core::World world) : world_(std::move(world)) {
  // Centre on something sensible straight away: the first controllable entity,
  // falling back to whatever exists. A camera with no target would have to be
  // special-cased by every client, and this is cheaper.
  for (const core::Entity& entity : world_.entities()) {
    if (entity.controllable) {
      snapshot_.selected = entity.id;
      break;
    }
  }
  if (snapshot_.selected == core::kInvalidEntity && !world_.entities().empty()) {
    snapshot_.selected = world_.entities().front().id;
  }
  publish();
}

LocalSimSource LocalSimSource::from_scenario(const core::Scenario& scenario) {
  core::World world = core::World::from_scenario(scenario);
  world.clock().set_tdb(scenario.epoch.tdb);
  return LocalSimSource(std::move(world));
}

const proto::Snapshot& LocalSimSource::snapshot() const { return snapshot_; }

std::string_view LocalSimSource::describe() const { return "local"; }

void LocalSimSource::pump(double wall_dt) {
  // The one place the wall clock enters. It is converted to a duration
  // immediately and never stored as a timestamp.
  const double frame_wall = timer_.tick();
  const double elapsed = wall_dt > 0.0 ? wall_dt : frame_wall;
  const core::Seconds span = world_.clock().frame_span(elapsed);

  // Even a paused world republishes, because a command may have changed the
  // selection and clients redraw off the sequence number.
  if (span > 0.0) {
    world_.advance(span, max_step_);
  }
  publish();
}

void LocalSimSource::send(const proto::Command& command) { apply(command); }

void LocalSimSource::apply(const proto::Command& command) {
  switch (command.kind) {
    case proto::CommandKind::SetWarp:
      world_.clock().set_warp(command.value);
      break;
    case proto::CommandKind::StepWarp:
      world_.clock().step_warp(command.value > 0.0 ? 1 : (command.value < 0.0 ? -1 : 0));
      break;
    case proto::CommandKind::SelectTarget:
      // A stale id from a client that missed a removal just clears the
      // selection instead of being an error.
      snapshot_.selected = world_.find(static_cast<core::EntityId>(command.target)) != nullptr
                               ? command.target
                               : core::kInvalidEntity;
      break;
    case proto::CommandKind::CycleTarget:
      cycle_target(command.value > 0.0 ? 1 : -1);
      break;
    case proto::CommandKind::Remove:
      if (world_.remove(static_cast<core::EntityId>(command.target))) {
        if (snapshot_.selected == command.target) {
          cycle_target(1);
        }
      }
      break;
    case proto::CommandKind::Quit:
      quit_ = true;
      break;
  }
}

void LocalSimSource::cycle_target(int direction) {
  const std::vector<core::Entity>& entities = world_.entities();
  if (entities.empty()) {
    snapshot_.selected = core::kInvalidEntity;
    return;
  }

  // Only controllable entities are worth cycling to; debris is reachable by
  // clicking, not by tabbing.
  std::vector<core::EntityId> candidates;
  for (const core::Entity& entity : entities) {
    if (entity.controllable) {
      candidates.push_back(entity.id);
    }
  }
  if (candidates.empty()) {
    for (const core::Entity& entity : entities) {
      candidates.push_back(entity.id);
    }
  }

  const auto current = std::find(candidates.begin(), candidates.end(),
                                 static_cast<core::EntityId>(snapshot_.selected));
  std::size_t index = 0;
  if (current == candidates.end()) {
    index = direction > 0 ? 0 : candidates.size() - 1;
  } else {
    const std::ptrdiff_t position = current - candidates.begin();
    const std::ptrdiff_t count = static_cast<std::ptrdiff_t>(candidates.size());
    index = static_cast<std::size_t>(((position + direction) % count + count) % count);
  }
  snapshot_.selected = candidates[index];
}

void LocalSimSource::publish() {
  ++snapshot_.sequence;
  snapshot_.tdb = world_.clock().tdb();
  snapshot_.warp = world_.clock().warp();

  const core::BodySystem& system = world_.bodies();
  snapshot_.body_count = 0;
  for (const core::CelestialBody& body : system.bodies()) {
    if (snapshot_.body_count >= proto::kMaxBodies) {
      break;
    }
    proto::BodySnapshot& out = snapshot_.bodies[snapshot_.body_count++];
    out = proto::BodySnapshot{};
    out.id = body.id;
    out.parent = body.parent;
    out.mu = body.mu;
    out.radius = body.radius;
    out.soi_radius = body.soi_radius;
    out.name.assign(body.name);
    out.position = to_wire(system.root_state(body.id, snapshot_.tdb).r);
  }

  snapshot_.entity_count = 0;
  for (const core::Entity& entity : world_.entities()) {
    if (snapshot_.entity_count >= proto::kMaxEntities) {
      break;
    }
    proto::EntitySnapshot& out = snapshot_.entities[snapshot_.entity_count++];
    out = proto::EntitySnapshot{};
    out.id = entity.id;
    out.kind = entity.kind == core::EntityKind::Debris ? proto::Kind::Debris : proto::Kind::Vessel;
    out.name.assign(entity.name);
    out.mass = entity.mass;
    out.radius = entity.radius;
    out.position = to_wire(entity.state.r);
    out.velocity = to_wire(entity.state.v);

    // Sent alongside the relative state so a client can compose the root
    // position for rendering without asking again, and without either side
    // having to undo a subtraction to get relative motion back.
    const core::StateVector parent_root = system.root_state(entity.parent, snapshot_.tdb);
    out.parent_position = to_wire(parent_root.r);
    out.parent_velocity = to_wire(parent_root.v);

    if (entity.controllable) {
      out.flags = out.flags | proto::Flags::Controllable;
    }
    if (entity.parent != core::kInvalidBody) {
      summarise(entity, system.body(entity.parent), out);
    }
  }
}

bool LocalSimSource::query_trajectory(std::uint64_t id, double horizon,
                                      std::vector<proto::Vec3d>& out) const {
  out.clear();
  const core::Entity* entity = world_.find(static_cast<core::EntityId>(id));
  if (entity == nullptr) {
    return false;
  }
  if (!(horizon > 0.0)) {
    return true;
  }

  // The prediction runs on a copy of the world and steps it exactly the way
  // the live simulation will. That costs a copy per query, which is nothing
  // next to the alternative: reimplementing the propagation here would give a
  // second, silently divergent answer to "where will this be", and it would
  // not know about sphere-of-influence crossings at all.
  core::World scratch = world_;

  out.reserve(kTrajectorySamples + 1);
  const double sample_dt = horizon / static_cast<double>(kTrajectorySamples);
  for (std::size_t i = 0; i <= kTrajectorySamples; ++i) {
    const core::Entity* current = scratch.find(static_cast<core::EntityId>(id));
    if (current == nullptr) {
      break;
    }
    out.push_back(to_wire(scratch.root_state(*current).r));
    if (i < kTrajectorySamples) {
      // One step per sample, but never coarser than the live step, so the
      // predicted path bends where the real one would.
      scratch.advance(sample_dt, std::min(max_step_, sample_dt));
    }
  }
  return true;
}

}  // namespace rocketlab::simhost
