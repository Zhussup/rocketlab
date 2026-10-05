// Tests for docking: the predicate that decides whether two vessels may join,
// and the merge that happens when they do.
//
// Two things are worth testing and are easy to get wrong in opposite
// directions. The predicate has to be a *gap*, not a centre distance — a
// big vessel and a small one touching have centres far apart — and the merge
// has to conserve mass and momentum while leaving the survivor in charge of its
// own staging. A docking that quietly renumbered the target's stages would
// leave a vessel whose next jettison drops the wrong half.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <string>
#include <vector>

#include "rocketlab/core/docking.hpp"
#include "rocketlab/core/part.hpp"
#include "rocketlab/core/vessel.hpp"
#include "rocketlab/core/world.hpp"
#include "rocketlab/hud/readout.hpp"
#include "rocketlab/proto/command.hpp"
#include "rocketlab/simhost/local_sim_source.hpp"

using namespace rocketlab;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

namespace core = rocketlab::core;
namespace proto = rocketlab::proto;
namespace simhost = rocketlab::simhost;

/// A two-stage tug, so the survivor has staging to keep straight.
std::vector<core::StackPart> tug_stack() {
  return {
      {"probe.core", 1},      {"tank.fl100", 1}, {"engine.terrier", 1},
      {"decoupler.large", 0}, {"tank.fl400", 0}, {"engine.skipper", 0},
  };
}

/// Something light to arrive in.
std::vector<core::StackPart> probe_stack() {
  return {{"probe.core", 0}, {"tank.fl100", 0}, {"engine.ant", 0}};
}

/// A world with the solar system in it and no entities. Docking needs a body
/// tree — the entities' frames are bodies — but nothing else.
core::World empty_world() {
  core::Scenario scenario;
  scenario.name = "docking test";
  return core::World::from_scenario(scenario);
}

/// A vessel entity sitting at `r` with velocity `v`, built from `stack`.
///
/// Its mass and radius come from the parts, exactly as a scenario's do. A
/// helper that let the test set them by hand would be testing a vessel the
/// simulation never produces, and the merge reads both from the vessel.
core::Entity vessel_at(const core::World& world, std::string name, const core::Vec3& r,
                       const core::Vec3& v, const std::vector<core::StackPart>& stack) {
  core::Entity entity;
  entity.kind = core::EntityKind::Vessel;
  entity.name = std::move(name);
  entity.parent = *world.bodies().find("Earth");
  entity.state = core::StateVector{r, v};
  entity.controllable = true;
  entity.vessel = core::build_vessel(stack, core::PartCatalogue::stock());
  entity.mass = entity.vessel.mass();
  entity.radius = entity.vessel.radius() > 0.0 ? entity.vessel.radius() : 1.0;
  return entity;
}

}  // namespace

TEST_CASE("a client reaches the host's verdict from the snapshot alone", "[hud][docking]") {
  // The host decides with `core::dock_block` on two `Entity`s and the client
  // with `hud::dock_block` on two `EntitySnapshot`s. Those are two pieces of
  // code answering one question, which is exactly the arrangement that drifts,
  // so both are built here from one set of numbers and made to agree.
  const core::World world = empty_world();
  const core::BodyId earth = *world.bodies().find("Earth");
  const core::BodyId moon = *world.bodies().find("Moon");

  struct Vessel {
    core::EntityId id;
    core::BodyId parent;
    core::Vec3 r;
    core::Vec3 v;
    double radius;
    bool built{true};
  };

  const auto host_side = [](const Vessel& spec) {
    core::Entity entity;
    entity.kind = core::EntityKind::Vessel;
    entity.id = spec.id;
    entity.parent = spec.parent;
    entity.state = core::StateVector{spec.r, spec.v};
    entity.radius = spec.radius;
    entity.mass = 1000.0;
    if (spec.built) {
      entity.vessel = core::build_vessel({{"probe.core", 0}}, core::PartCatalogue::stock());
    }
    return entity;
  };
  const auto client_side = [](const Vessel& spec) {
    proto::EntitySnapshot snapshot{};
    snapshot.id = spec.id;
    snapshot.parent = spec.parent;
    snapshot.position = proto::Vec3d{spec.r.x, spec.r.y, spec.r.z};
    snapshot.velocity = proto::Vec3d{spec.v.x, spec.v.y, spec.v.z};
    snapshot.radius = spec.radius;
    snapshot.stage_count = spec.built ? 1 : 0;
    snapshot.kind = proto::Kind::Vessel;
    return snapshot;
  };

  const auto agrees = [&](const Vessel& target, const Vessel& absorbed) {
    return hud::dock_block(client_side(target), client_side(absorbed)) ==
           core::dock_block(host_side(target), host_side(absorbed), core::kDefaultDockingLimits);
  };

  SECTION("the geometry is the same arithmetic on both sides") {
    const Vessel target{1, earth, {7.0e6, 0.0, 0.0}, {0.0, 7000.0, 0.0}, 4.0};
    const Vessel absorbed{2, earth, {7.0e6 + 12.0, 3.0, 0.0}, {0.0, 7001.0, -2.0}, 3.0};

    const core::DockingGeometry from_snapshot =
        hud::dock_geometry(client_side(target), client_side(absorbed));
    const core::DockingGeometry from_entities =
        core::docking_geometry(host_side(target), host_side(absorbed));

    CHECK_THAT(from_snapshot.separation, WithinAbs(from_entities.separation, 0.0));
    CHECK_THAT(from_snapshot.combined_radius, WithinAbs(from_entities.combined_radius, 0.0));
    CHECK_THAT(from_snapshot.closing_speed, WithinAbs(from_entities.closing_speed, 0.0));
  }

  SECTION("every rule gives the host's answer") {
    // A walk across the interesting side of each rule: touching and not,
    // slow and not, and each of the three structural refusals.
    const Vessel target{1, earth, {7.0e6, 0.0, 0.0}, {0.0, 7000.0, 0.0}, 4.0};
    const Vessel touching{2, earth, {7.0e6 + 9.0, 0.0, 0.0}, {0.0, 7000.5, 0.0}, 4.0};
    const Vessel overlapping{2, earth, {7.0e6 + 6.0, 0.0, 0.0}, {0.0, 7000.5, 0.0}, 4.0};
    const Vessel too_far{2, earth, {7.0e6 + 20.0, 0.0, 0.0}, {0.0, 7000.5, 0.0}, 4.0};
    const Vessel too_fast{2, earth, {7.0e6 + 9.0, 0.0, 0.0}, {0.0, 7010.0, 0.0}, 4.0};
    const Vessel self{1, earth, {7.0e6, 0.0, 0.0}, {0.0, 7000.0, 0.0}, 4.0};
    Vessel point_mass = touching;
    point_mass.built = false;
    const Vessel elsewhere{2, moon, {7.0e6 + 9.0, 0.0, 0.0}, {0.0, 7000.5, 0.0}, 4.0};

    CHECK(agrees(target, touching));
    CHECK(agrees(target, overlapping));
    CHECK(agrees(target, too_far));
    CHECK(agrees(target, too_fast));
    CHECK(agrees(target, self));
    CHECK(agrees(point_mass, target));
    CHECK(agrees(target, point_mass));
    CHECK(agrees(target, elsewhere));

    // And the verdicts themselves, so that agreement on "None" everywhere
    // could not pass for agreement.
    CHECK(hud::dock_block(client_side(target), client_side(touching)) == core::DockBlock::None);
    CHECK(hud::dock_block(client_side(target), client_side(too_far)) == core::DockBlock::TooFar);
    CHECK(hud::dock_block(client_side(target), client_side(too_fast)) == core::DockBlock::TooFast);
    CHECK(hud::dock_block(client_side(target), client_side(self)) == core::DockBlock::SameEntity);
    CHECK(hud::dock_block(client_side(target), client_side(point_mass)) == core::DockBlock::NoParts);
    CHECK(hud::dock_block(client_side(target), client_side(elsewhere)) ==
          core::DockBlock::DifferentParent);
  }

  SECTION("the structure is decided from ids the snapshot carries") {
    // Nothing here needs a state to be read: a refusal that no manoeuvre would
    // change is answered before the numbers are looked at, so a client can grey
    // out the key without measuring anything.
    const Vessel here{1, earth, {7.0e6, 0.0, 0.0}, {0.0, 7000.0, 0.0}, 4.0};
    Vessel far_away{2, earth, {1.0e11, 0.0, 0.0}, {0.0, 7000.0, 0.0}, 4.0};

    proto::EntitySnapshot other = client_side(far_away);
    other.id = here.id;
    CHECK(hud::dock_block(client_side(here), other) == core::DockBlock::SameEntity);

    other = client_side(far_away);
    other.stage_count = 0;
    CHECK(hud::dock_block(client_side(here), other) == core::DockBlock::NoParts);

    other = client_side(far_away);
    other.parent = moon;
    CHECK(hud::dock_block(client_side(here), other) == core::DockBlock::DifferentParent);
  }
}

TEST_CASE("the docking predicate measures the gap, not the centres", "[core][docking]") {
  const core::DockingLimits limits;  // the shipped defaults: 5 m, 2 m/s

  core::DockingGeometry geometry;
  geometry.combined_radius = 8.0;

  SECTION("two large vessels touching at their surfaces may dock") {
    // Centres 13 m apart with 8 m of combined radius: a 5 m gap, which is
    // exactly the clearance and therefore allowed. A predicate written on the
    // centre distance would need a different tolerance for every pair of
    // sizes, which is the reason it is written this way.
    geometry.separation = 13.0;
    geometry.closing_speed = 0.5;
    CHECK(core::can_dock(geometry, limits));
  }

  SECTION("a millimetre more of gap is too far") {
    geometry.separation = 13.001;
    geometry.closing_speed = 0.5;
    CHECK(core::dock_block(geometry, limits) == core::DockBlock::TooFar);
  }

  SECTION("distance is reported before speed, because it is the fixable one") {
    geometry.separation = 1.0e6;
    geometry.closing_speed = 1.0e4;
    CHECK(core::dock_block(geometry, limits) == core::DockBlock::TooFar);
  }

  SECTION("close but too fast is a collision, not a docking") {
    geometry.separation = 10.0;
    geometry.closing_speed = 2.001;
    CHECK(core::dock_block(geometry, limits) == core::DockBlock::TooFast);
  }

  SECTION("a state that has gone non-finite is refused rather than merged") {
    geometry.separation = std::nan("");
    geometry.closing_speed = 0.0;
    CHECK(core::dock_block(geometry, limits) == core::DockBlock::TooFar);
  }
}

TEST_CASE("the docking geometry is read off the shared frame", "[core][docking]") {
  const core::StateVector target{{7.0e6, 0.0, 0.0}, {0.0, 7000.0, 0.0}};
  const core::StateVector absorbed{{7.0e6 + 3.0, 4.0, 0.0}, {0.0, 7001.0, -2.0}};

  const core::DockingGeometry geometry = core::docking_geometry(target, absorbed, 3.0, 2.0);

  CHECK_THAT(geometry.separation, WithinAbs(5.0, 1e-9));
  CHECK_THAT(geometry.combined_radius, WithinAbs(5.0, 1e-9));
  CHECK_THAT(geometry.closing_speed, WithinAbs(std::sqrt(1.0 + 4.0), 1e-9));
  // Touching, but the three-metre-per-second closing speed is too fast.
  CHECK(core::dock_block(geometry, core::kDefaultDockingLimits) == core::DockBlock::TooFast);
}

TEST_CASE("docking refuses the pairs that are not a docking", "[core][docking]") {
  core::World world = empty_world();
  const core::DockingLimits limits;

  core::Entity target =
      vessel_at(world, "Station", {7.0e6, 0.0, 0.0}, {0.0, 7000.0, 0.0}, tug_stack());
  target.id = 1;
  core::Entity nearby =
      vessel_at(world, "Probe", {7.0e6 + 5.0, 0.0, 0.0}, {0.0, 7000.5, 0.0}, probe_stack());
  nearby.id = 2;

  SECTION("an entity cannot dock with itself") {
    CHECK(core::dock_block(target, target, limits) == core::DockBlock::SameEntity);
  }

  SECTION("a point mass has no stack to contribute") {
    core::Entity point_mass = nearby;
    point_mass.vessel = core::Vessel{};
    CHECK(core::dock_block(target, point_mass, limits) == core::DockBlock::NoParts);
    CHECK(core::dock_block(point_mass, target, limits) == core::DockBlock::NoParts);
  }

  SECTION("two vessels around different bodies are not in one frame") {
    core::Entity elsewhere = nearby;
    elsewhere.parent = *world.bodies().find("Moon");
    CHECK(core::dock_block(target, elsewhere, limits) == core::DockBlock::DifferentParent);
  }

  SECTION("the structure is checked before the geometry") {
    // Same entity *and* a million metres apart: the structural answer is the
    // one worth giving, because no amount of flying would change it.
    core::Entity far = target;
    far.state.r = {1.0e9, 0.0, 0.0};
    CHECK(core::dock_block(far, far, limits) == core::DockBlock::SameEntity);
  }
}

TEST_CASE("a docking joins the stacks and keeps the survivor flying", "[core][docking]") {
  core::World world = empty_world();

  const std::vector<core::StackPart> tug = tug_stack();
  const std::vector<core::StackPart> probe = probe_stack();

  core::Entity target = vessel_at(world, "Station", {7.0e6, 0.0, 0.0}, {0.0, 7000.0, 0.0}, tug);
  target.vessel.current_stage = 1;  // part-way through: stage 0 already gone
  target.vessel.throttle = 0.5;
  const core::EntityId target_id = world.add(std::move(target));

  core::Entity arriving =
      vessel_at(world, "Probe", {7.0e6 + 5.0, 0.0, 0.0}, {0.0, 7001.0, 0.0}, probe);
  const core::EntityId arriving_id = world.add(std::move(arriving));

  const core::Entity before_target = *world.find(target_id);
  const core::Entity before_arriving = *world.find(arriving_id);

  const core::EntityId survivor = world.dock(target_id, arriving_id);
  REQUIRE(survivor == target_id);

  SECTION("the absorbed entity is gone and the survivor remains") {
    CHECK(world.find(arriving_id) == nullptr);
    REQUIRE(world.find(target_id) != nullptr);
    CHECK(world.entities().size() == 1);
  }

  SECTION("the two stacks became one, the arriving one on top") {
    const core::Entity& joined = *world.find(target_id);
    CHECK(joined.vessel.parts.size() == tug.size() + probe.size());

    // Top first: the arriving probe's core is now the topmost part.
    CHECK(joined.vessel.parts.front().name == probe.front().part);
    CHECK(joined.vessel.parts.back().name == tug.back().part);
    CHECK(joined.vessel.propellant.size() == joined.vessel.parts.size());
  }

  SECTION("the arriving stages are numbered above everything the target had") {
    const core::Entity& joined = *world.find(target_id);
    // The tug used stages 0 and 1, so the probe's parts move up by two and the
    // joined vessel has three stages. Nothing shares a number.
    CHECK(joined.vessel.stage_count == 3);
    for (std::size_t i = 0; i < probe.size(); ++i) {
      CHECK(joined.vessel.stage[i] == 2 + probe[i].stage);
    }
    for (std::size_t i = 0; i < tug.size(); ++i) {
      CHECK(joined.vessel.stage[probe.size() + i] == tug[i].stage);
    }
    // Every part still carries its own propellant: a tank that arrived full is
    // full, and a stage that was already half spent still is. Checked as a
    // total, because the split between the tanks is not what docking decides.
    CHECK_THAT(joined.vessel.propellant_left(),
               WithinRel(before_target.vessel.propellant_left() +
                             before_arriving.vessel.propellant_left(),
                         1e-12));
  }

  SECTION("the survivor keeps its own current stage, throttle and steering") {
    const core::Entity& joined = *world.find(target_id);
    CHECK(joined.vessel.current_stage == 1);
    CHECK_THAT(joined.vessel.throttle, WithinAbs(0.5, 1e-12));
  }

  SECTION("mass is the sum and radius is the wider of the two") {
    const core::Entity& joined = *world.find(target_id);
    CHECK_THAT(joined.mass, WithinRel(before_target.mass + before_arriving.mass, 1e-12));
    CHECK_THAT(joined.mass, WithinRel(joined.vessel.mass(), 1e-12));
    CHECK(joined.radius >= before_target.radius);
  }

  SECTION("momentum is conserved and the survivor barely notices") {
    const core::Entity& joined = *world.find(target_id);
    const double total = before_target.mass + before_arriving.mass;
    const double expected =
        (before_target.state.v.y * before_target.mass +
         before_arriving.state.v.y * before_arriving.mass) / total;
    CHECK_THAT(joined.state.v.y, WithinAbs(expected, 1e-9));
    // One metre per second of closing speed against a much heavier station:
    // the velocity moves, but by a fraction of a metre per second.
    CHECK(joined.state.v.y > before_target.state.v.y);
    CHECK(joined.state.v.y < before_target.state.v.y + 0.5);
    // The position is not moved: it is the object the camera was following.
    CHECK_THAT(joined.state.r.x, WithinAbs(before_target.state.r.x, 1e-12));
  }
}

TEST_CASE("a docking the rules refuse changes nothing", "[core][docking]") {
  core::World world = empty_world();

  core::Entity target =
      vessel_at(world, "Station", {7.0e6, 0.0, 0.0}, {0.0, 7000.0, 0.0}, tug_stack());
  const core::EntityId target_id = world.add(std::move(target));
  core::Entity far = vessel_at(world, "Probe", {1.0e9, 0.0, 0.0}, {0.0, 7000.0, 0.0},
                               probe_stack());
  const core::EntityId far_id = world.add(std::move(far));

  CHECK(world.dock(target_id, far_id) == core::kInvalidEntity);
  CHECK(world.dock(target_id, target_id) == core::kInvalidEntity);
  CHECK(world.dock(target_id, 9999) == core::kInvalidEntity);

  // Nothing was merged and nothing was removed.
  CHECK(world.entities().size() == 2);
  CHECK(world.find(far_id) != nullptr);
  CHECK(world.find(target_id)->vessel.parts.size() == tug_stack().size());
}

TEST_CASE("the dock command goes through the host", "[core][docking][host]") {
  // Two vessels in the same circular orbit, a couple of metres apart in true
  // anomaly. That is a real rendezvous, not a test backdoor: the host has no
  // "put this here" command and should not gain one, and an orbit is the only
  // way a scenario can say where a vessel is. The difference is small because
  // two metres at 6771 km is 1.7e-5 of a degree.
  core::Scenario scenario;
  scenario.name = "docking test";

  core::ScenarioEntity station;
  station.name = "Station";
  station.parent_body = "Earth";
  station.periapsis_altitude = 400e3;
  station.apoapsis_altitude = 400e3;
  station.true_anomaly_deg = 0.0;
  station.parts = tug_stack();

  core::ScenarioEntity probe = station;
  probe.name = "Probe";
  probe.true_anomaly_deg = 1.7e-5;
  probe.parts = probe_stack();

  scenario.entities = {station, probe};

  auto source = simhost::LocalSimSource::from_scenario(scenario);
  REQUIRE(source.snapshot().entity_count == 2);

  const std::uint64_t station_id = source.snapshot().entities[0].id;
  const std::uint64_t probe_id = source.snapshot().entities[1].id;
  REQUIRE(station_id != probe_id);

  // What the client would ask before offering the key at all.
  const proto::EntitySnapshot& a = source.snapshot().entities[0];
  const proto::EntitySnapshot& b = source.snapshot().entities[1];
  const core::DockingGeometry geometry = core::docking_geometry(
      core::StateVector{{a.position.x, a.position.y, a.position.z},
                        {a.velocity.x, a.velocity.y, a.velocity.z}},
      core::StateVector{{b.position.x, b.position.y, b.position.z},
                        {b.velocity.x, b.velocity.y, b.velocity.z}},
      a.radius, b.radius);
  REQUIRE(core::can_dock(geometry, core::kDefaultDockingLimits));

  source.send(proto::Command::select(station_id));
  source.send(proto::Command::dock(probe_id));
  source.pump(1e-6);  // a positive duration, so the frame is published

  CHECK(source.snapshot().entity_count == 1);
  CHECK(source.snapshot().entities[0].id == station_id);
  CHECK(source.snapshot().selected == station_id);
  // Six tug parts plus three probe parts, in three stages: the probe's single
  // stage was renumbered above the tug's two.
  CHECK(source.snapshot().entities[0].stage_count == 3);
  CHECK(source.snapshot().selected != probe_id);
}

TEST_CASE("a dock the host refuses leaves the world alone", "[core][docking][host]") {
  core::Scenario scenario;
  core::ScenarioEntity station;
  station.name = "Station";
  station.parent_body = "Earth";
  station.periapsis_altitude = 400e3;
  station.apoapsis_altitude = 400e3;
  station.parts = tug_stack();

  // The other side of the planet, so there is no manoeuvre that makes this a
  // docking and the host must decline.
  core::ScenarioEntity probe = station;
  probe.name = "Probe";
  probe.true_anomaly_deg = 180.0;
  probe.parts = probe_stack();
  scenario.entities = {station, probe};

  auto source = simhost::LocalSimSource::from_scenario(scenario);
  const std::uint64_t station_id = source.snapshot().entities[0].id;
  const std::uint64_t probe_id = source.snapshot().entities[1].id;

  source.send(proto::Command::select(station_id));
  source.send(proto::Command::dock(probe_id));
  source.pump(1e-6);

  CHECK(source.snapshot().entity_count == 2);
  CHECK(source.snapshot().selected == station_id);
}
