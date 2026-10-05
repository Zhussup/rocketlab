#include "rocketlab/simhost/local_sim_source.hpp"

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <utility>

namespace rocketlab::simhost {

namespace {

namespace proto = rocketlab::proto;

/// Builds the read-only view a flight computer is allowed.
///
/// It is the same state a client sees in a snapshot, for the same reason: a
/// program that could see the `World` would be a second place where physics
/// happens, and the two would eventually disagree.
[[nodiscard]] flight::Input make_input(const core::World& world, const core::Entity& entity,
                                      core::Seconds met) {
  const core::CelestialBody& body = world.bodies().body(entity.parent);

  flight::Input in;
  in.met = met;
  in.state = entity.state;
  in.mu = body.mu;
  in.body_radius = body.radius;
  in.altitude = core::norm(entity.state.r) - body.radius;
  in.elements = core::rv_to_elements(entity.state, body.mu);
  in.time_to_apoapsis = core::time_to_apoapsis(in.elements, body.mu);
  in.time_to_periapsis = core::time_to_periapsis(in.elements, body.mu);
  if (!entity.vessel.empty()) {
    in.vessel = &entity.vessel;
  }
  return in;
}

[[nodiscard]] double clamp_throttle(double value) noexcept {
  return value < 0.0 ? 0.0 : (value > 1.0 ? 1.0 : value);
}

/// Advances a scratch world the same way the live one will be advanced,
/// running a clone of some entity's flight computer as it goes.
///
/// Without this, a scripted vessel's predicted path would be the path it would
/// take if it did nothing — which is not a prediction of the mission, it is a
/// picture of a different one. The clone is what makes the two agree: the same
/// program, from the same state, at the same control rate, has to reach the same
/// answer, and if it does not then the run was never reproducible to begin with.
void advance_prediction(core::World& world, flight::Program* program, core::EntityId id,
                        core::Seconds met, core::Seconds span, core::Seconds max_step,
                        core::Seconds control_period) {
  const bool scripting = program != nullptr && control_period > 0.0;
  const core::Seconds step = scripting ? std::min(max_step, control_period) : max_step;

  core::Seconds remaining = span;
  while (remaining > 0.0) {
    const core::Seconds dt = std::min(remaining, step);

    if (scripting) {
      const core::Entity* entity = world.find(id);
      if (entity == nullptr) {
        return;
      }
      flight::Output out;
      if (program->update(make_input(world, *entity, met), out)) {
        // Re-found because `jettison_stage` may have appended debris and moved
        // the storage out from under the pointer above.
        core::Entity* target = world.find(id);
        if (target != nullptr && !target->vessel.empty()) {
          target->vessel.throttle = clamp_throttle(out.throttle);
          if (out.set_attitude) {
            target->vessel.thrust_dir = out.thrust_dir;
          }
        }
        if (out.stage) {
          world.jettison_stage(id);
        }
      }
    }

    world.advance(dt, dt);
    met += dt;
    remaining -= dt;
  }
}

[[nodiscard]] proto::ComputerState to_wire(flight::Status status) noexcept {
  switch (status) {
    case flight::Status::Idle:
      return proto::ComputerState::Idle;
    case flight::Status::Running:
      return proto::ComputerState::Running;
    case flight::Status::Finished:
      return proto::ComputerState::Finished;
    case flight::Status::Faulted:
      return proto::ComputerState::Faulted;
  }
  return proto::ComputerState::None;
}

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
  // The clock already reads the scenario's epoch here, which is what makes
  // mission elapsed time count from the start of the mission rather than from
  // J2000.
  epoch_tdb_ = world_.clock().tdb();

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

  LocalSimSource source(std::move(world));

  // Scripts are loaded after the world exists, because compiling one needs
  // nothing from the world but attaching it needs the entity ids the world just
  // handed out.
  std::vector<ComputerAttachment> pending;
  for (const core::ScenarioEntity& spec : scenario.entities) {
    if (spec.script.empty()) {
      continue;
    }
    const core::Entity* entity = nullptr;
    for (const core::Entity& candidate : source.world_.entities()) {
      if (candidate.name == spec.name) {
        entity = &candidate;
        break;
      }
    }
    if (entity == nullptr) {
      continue;
    }

    std::string error;
    std::unique_ptr<flight::LuaProgram> program =
        flight::LuaProgram::compile_file(spec.script, source.script_limits_, error);
    if (program == nullptr) {
      throw std::runtime_error(error);
    }

    ComputerAttachment attachment;
    attachment.entity = entity->id;
    attachment.path = spec.script;
    attachment.program = std::move(program);
    pending.push_back(std::move(attachment));
  }
  source.computers_ = std::move(pending);
  source.publish();
  return source;
}

bool LocalSimSource::attach_script(core::EntityId id, const std::string& path, std::string& error) {
  if (world_.find(id) == nullptr) {
    error = "flight computer: no such entity";
    return false;
  }
  std::unique_ptr<flight::LuaProgram> program =
      flight::LuaProgram::compile_file(path, script_limits_, error);
  if (program == nullptr) {
    return false;
  }

  detach_script(id);
  ComputerAttachment attachment;
  attachment.entity = id;
  attachment.path = path;
  attachment.program = std::move(program);
  computers_.push_back(std::move(attachment));
  publish();
  return true;
}

void LocalSimSource::detach_script(core::EntityId id) {
  const auto it = std::remove_if(computers_.begin(), computers_.end(),
                                 [id](const ComputerAttachment& a) { return a.entity == id; });
  computers_.erase(it, computers_.end());
}

bool LocalSimSource::has_script(core::EntityId id) const noexcept {
  return std::any_of(computers_.begin(), computers_.end(),
                     [id](const ComputerAttachment& a) { return a.entity == id; });
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
    advance_with_control(span);
  }
  publish();
}

void LocalSimSource::advance_with_control(core::Seconds span) {
  core::Seconds remaining = span;

  // Without a program there is nothing to control and the world can be stepped
  // in one go, which is what keeps time warp cheap. With one, the step is
  // shortened to the control period so that no decision is applied for longer
  // than that.
  const bool scripting = !computers_.empty() && computers_enabled_ && control_period_ > 0.0;
  const core::Seconds step = scripting ? std::min(max_step_, control_period_) : max_step_;

  while (remaining > 0.0) {
    const core::Seconds dt = std::min(remaining, step);
    if (scripting) {
      run_computers();
    }
    // `max_step` is passed as `dt` so that `advance` takes exactly one step:
    // the subdivision is this loop's job now, because the control period has to
    // interleave with it.
    world_.advance(dt, dt);
    remaining -= dt;
  }
}

void LocalSimSource::run_computers() {
  const core::Seconds met = world_.clock().tdb() - epoch_tdb_;

  // Staging is collected and applied after every program has run: `jettison_stage`
  // appends an entity, which can move the vector the loop below is walking.
  std::vector<core::EntityId> to_stage;

  for (ComputerAttachment& attachment : computers_) {
    flight::Program* program = attachment.program.get();
    if (program == nullptr) {
      continue;
    }
    // A program that has already stopped is not run again — but the vessel
    // still holds the throttle it was given on its last tick, which is the
    // point: see the note after `update` below.
    if (program->stopped()) {
      continue;
    }

    const core::Entity* entity = world_.find(attachment.entity);
    if (entity == nullptr) {
      // The entity was removed from under the program; the program goes with it.
      attachment.program.reset();
      attachment.status = flight::Status::Finished;
      attachment.message = "the entity was removed";
      continue;
    }

    const flight::Input in = make_input(world_, *entity, met);
    flight::Output out;
    const bool alive = program->update(in, out);

    attachment.status = program->status();
    attachment.message = std::string(program->message());

    // The tick that stops a program is still a command, and applying it is what
    // lets `ship.abort()` cut the throttle on its way out. Dropping the output of
    // that last tick left the vessel burning at whatever the tick before had
    // asked for, with nobody left to stop it — a Hohmann transfer that carried on
    // burning past circularisation until it was hyperbolic.
    //
    // A program that *faulted* has no last word worth trusting, so none of it is
    // applied and the host cuts the throttle itself. An autopilot that has just
    // lost control must not leave the engine lit.
    if (!alive && attachment.status == flight::Status::Faulted) {
      out = flight::Output{};
    }

    if (out.stage) {
      to_stage.push_back(attachment.entity);
    }

    core::Entity* target = world_.find(attachment.entity);
    if (target == nullptr || target->vessel.empty()) {
      continue;
    }
    target->vessel.throttle = clamp_throttle(out.throttle);
    if (out.set_attitude) {
      target->vessel.thrust_dir = out.thrust_dir;
    }
  }

  for (const core::EntityId id : to_stage) {
    world_.jettison_stage(id);
  }
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
    case proto::CommandKind::Stage: {
      // An omitted target means "whatever is selected", which is what a client
      // with only one thing to stage should have to say.
      const core::EntityId id = command.target != 0 ? static_cast<core::EntityId>(command.target)
                                                    : static_cast<core::EntityId>(snapshot_.selected);
      world_.jettison_stage(id);
      break;
    }
    case proto::CommandKind::SetThrottle: {
      const core::EntityId id = command.target != 0 ? static_cast<core::EntityId>(command.target)
                                                    : static_cast<core::EntityId>(snapshot_.selected);
      if (core::Entity* entity = world_.find(id)) {
        entity->vessel.throttle = std::min(1.0, std::max(0.0, command.value));
      }
      break;
    }
    case proto::CommandKind::SetComputer:
      set_computers_enabled(command.value != 0.0);
      break;
    case proto::CommandKind::Dock: {
      // The selection is the survivor, so this is "join that thing onto what I
      // am flying". A stale or absent target is not an error: the client that
      // sent it was looking at an older frame, and there is nothing useful to
      // do about that but decline.
      const core::EntityId onto = static_cast<core::EntityId>(snapshot_.selected);
      const core::EntityId joining = static_cast<core::EntityId>(command.target);
      const core::EntityId survivor = world_.dock(onto, joining);
      if (survivor != core::kInvalidEntity) {
        // The absorbed entity is gone, so anything pointing at it has to move.
        // The survivor is already the selection, so the camera stays put.
        snapshot_.selected = survivor;
      }
      break;
    }
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
    out.parent = entity.parent;
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
      out.air_density = system.body(entity.parent).density_at(entity.state.r);
    }

    if (!entity.vessel.empty()) {
      out.stage = static_cast<std::uint32_t>(std::max(0, entity.vessel.current_stage));
      out.stage_count = static_cast<std::uint32_t>(std::max(0, entity.vessel.stage_count));
      out.throttle = entity.vessel.throttle;
      out.thrust = entity.vessel.thrust();
      out.propellant = entity.vessel.propellant_left();
      out.propellant_capacity = entity.vessel.propellant_capacity();
      // Delta-v is derived from the propellant left, so it is worked out here
      // rather than being tracked down as it is spent. There is then no way for
      // it to drift from the tanks it describes.
      out.delta_v = core::remaining_delta_v(entity.vessel);
    }

    for (const ComputerAttachment& attachment : computers_) {
      if (attachment.entity != entity.id) {
        continue;
      }
      out.computer = attachment.program != nullptr ? to_wire(attachment.status)
                                                   : proto::ComputerState::None;
      out.computer_message.assign(attachment.message);
      out.computer_instructions = static_cast<double>(attachment.instructions());
      break;
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
  const core::BodyId anchor = entity->parent;

  // A scripted vessel is predicted by flying the script, not by pretending the
  // vessel coasts. The program is cloned first so that the prediction cannot
  // leave a mark on the one that is actually flying.
  std::unique_ptr<flight::Program> program;
  for (const ComputerAttachment& attachment : computers_) {
    if (attachment.entity == static_cast<core::EntityId>(id) && attachment.program != nullptr) {
      program = attachment.program->clone();
      break;
    }
  }
  core::Seconds met = world_.clock().tdb() - epoch_tdb_;

  out.reserve(kTrajectorySamples + 1);
  const double sample_dt = horizon / static_cast<double>(kTrajectorySamples);
  for (std::size_t i = 0; i <= kTrajectorySamples; ++i) {
    const core::Entity* current = scratch.find(static_cast<core::EntityId>(id));
    if (current == nullptr) {
      break;
    }

    // Subtracting the anchor body's position at the same instant converts the
    // root-frame state back into the parent frame, which is the frame the
    // client will compose against. It stays correct across a re-framing,
    // because the root state is always the true one.
    const core::Vec3 root = scratch.root_state(*current).r;
    const core::Vec3 anchor_now = scratch.bodies().root_state(anchor, scratch.clock().tdb()).r;
    out.push_back(to_wire(root - anchor_now));

    if (i < kTrajectorySamples) {
      // One step per sample, but never coarser than the live step, so the
      // predicted path bends where the real one would.
      advance_prediction(scratch, program.get(), static_cast<core::EntityId>(id), met, sample_dt,
                         std::min(max_step_, sample_dt), control_period_);
      met += sample_dt;
    }
  }
  return true;
}

}  // namespace rocketlab::simhost
