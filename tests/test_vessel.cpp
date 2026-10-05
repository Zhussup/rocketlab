// Tests for parts, staging and delta-v accounting.
//
// The interesting claim M2 makes is not "the arithmetic runs" but "the number
// the stage table promises is the number the simulation delivers". So the tests
// here compute the expectation the long way — sums written out by hand and the
// rocket equation applied to them — and then require the implementation and an
// actual burn to agree.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <string>
#include <vector>

#include "rocketlab/core/constants.hpp"
#include "rocketlab/core/part.hpp"
#include "rocketlab/core/vessel.hpp"
#include "rocketlab/core/world.hpp"
#include "rocketlab/hud/readout.hpp"
#include "rocketlab/scenario/json_io.hpp"
#include "rocketlab/simhost/local_sim_source.hpp"

using namespace rocketlab;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

namespace core = rocketlab::core;
namespace hud = rocketlab::hud;
namespace proto = rocketlab::proto;
namespace simhost = rocketlab::simhost;

/// A two-stage tug: a small upper stage on a large booster, which is the shape
/// every orbital rocket has and therefore the shape worth testing.
std::vector<core::StackPart> tug_stack() {
  return {
      {"probe.core", 1},      {"tank.fl100", 1},     {"engine.terrier", 1},
      {"decoupler.large", 0}, {"tank.fl400", 0},     {"engine.skipper", 0},
  };
}

/// One stage, small enough that a burn does not outlast the orbit it is in.
std::vector<core::StackPart> single_stage() {
  return {{"probe.core", 0}, {"tank.fl100", 0}, {"engine.terrier", 0}};
}

/// A vessel in a circular orbit about `body`, ready to be lit.
core::World world_with_vessel(const std::vector<core::StackPart>& stack, double altitude,
                              const std::string& body_name = "Earth") {
  core::Scenario scenario;
  scenario.name = "burn test";
  core::ScenarioEntity spec;
  spec.name = "Vessel";
  spec.parent_body = body_name;
  spec.periapsis_altitude = altitude;
  spec.apoapsis_altitude = altitude;
  spec.parts = stack;
  scenario.entities = {spec};
  return core::World::from_scenario(scenario);
}

}  // namespace

TEST_CASE("the catalogue holds the parts a scenario may name", "[vessel]") {
  const core::PartCatalogue catalogue = core::PartCatalogue::stock();
  CHECK(catalogue.size() > 8);

  // Every kind is represented, or a scenario could not build a complete vessel.
  for (const core::PartKind kind :
       {core::PartKind::CommandPod, core::PartKind::Tank, core::PartKind::Engine,
        core::PartKind::Decoupler, core::PartKind::Payload}) {
    bool found = false;
    for (const core::Part& part : catalogue.parts()) {
      found = found || part.kind == kind;
    }
    INFO("kind " << core::to_string(kind));
    CHECK(found);
  }

  const core::Part* tank = catalogue.find("tank.fl100");
  REQUIRE(tank != nullptr);
  CHECK(tank->kind == core::PartKind::Tank);
  CHECK_THAT(tank->propellant, WithinRel(4000.0, 1e-12));
  // A tank holds propellant and produces nothing. Getting this backwards is how
  // a station ends up with an engine it does not have.
  CHECK_THAT(tank->thrust, WithinAbs(0.0, 1e-12));

  const core::Part* engine = catalogue.find("engine.terrier");
  REQUIRE(engine != nullptr);
  CHECK(engine->kind == core::PartKind::Engine);
  CHECK(engine->isp > 0.0);
  CHECK(engine->thrust > 0.0);
  CHECK_THAT(engine->propellant, WithinAbs(0.0, 1e-12));

  CHECK(catalogue.find("engine.does.not.exist") == nullptr);
}

TEST_CASE("a stack resolves against the catalogue and its tanks fill", "[vessel]") {
  const core::Vessel vessel = core::build_vessel(single_stage(), core::PartCatalogue::stock());

  REQUIRE(vessel.parts.size() == 3);
  CHECK_FALSE(vessel.empty());
  CHECK(vessel.stage_count == 1);
  CHECK(vessel.current_stage == 0);

  // 90 kg of probe, 500 kg of tank holding 4000 kg, 500 kg of engine.
  CHECK_THAT(vessel.mass(), WithinAbs(90.0 + 500.0 + 4000.0 + 500.0, 1e-9));
  CHECK_THAT(vessel.propellant_capacity(), WithinAbs(4000.0, 1e-9));
  CHECK_THAT(vessel.propellant_left(), WithinAbs(4000.0, 1e-9));
  CHECK_THAT(vessel.radius(), WithinRel(1.25, 1e-12));
  CHECK_THAT(vessel.length(), WithinAbs(0.30 + 1.50 + 1.50, 1e-9));

  SECTION("a part the catalogue does not have is an error, not a light vessel") {
    std::vector<core::StackPart> broken = single_stage();
    broken.push_back({"engine.imaginary", 0});
    CHECK_THROWS_AS(core::build_vessel(broken, core::PartCatalogue::stock()),
                    std::invalid_argument);
  }

  SECTION("a negative stage is refused") {
    std::vector<core::StackPart> broken = single_stage();
    broken.push_back({"antenna.hg", -1});
    CHECK_THROWS_AS(core::build_vessel(broken, core::PartCatalogue::stock()),
                    std::invalid_argument);
  }
}

TEST_CASE("the stage table applies the rocket equation to the right masses", "[vessel]") {
  const core::Vessel vessel = core::build_vessel(tug_stack(), core::PartCatalogue::stock());
  const std::vector<core::StageReport> table = core::stage_table(vessel);
  REQUIRE(table.size() == 2);

  const double g0 = core::kStandardGravity;

  // Written out by hand rather than derived from the catalogue, so that a change
  // to a part's mass or to how stages are grouped has to be reconciled here.
  const double dry = 90.0 + 500.0 + 500.0 + 200.0 + 4000.0 + 3000.0;
  const double prop_booster = 32000.0;
  const double prop_upper = 4000.0;

  const double m0_booster = dry + prop_booster + prop_upper;
  const double m1_booster = m0_booster - prop_booster;
  const double expected_booster = 320.0 * g0 * std::log(m0_booster / m1_booster);

  const double m0_upper = 90.0 + 500.0 + prop_upper + 500.0;
  const double m1_upper = m0_upper - prop_upper;
  const double expected_upper = 345.0 * g0 * std::log(m0_upper / m1_upper);

  CHECK_THAT(table[0].ignition_mass, WithinAbs(m0_booster, 1e-6));
  CHECK_THAT(table[0].final_mass, WithinAbs(m1_booster, 1e-6));
  CHECK_THAT(table[0].propellant, WithinAbs(prop_booster, 1e-6));
  CHECK_THAT(table[0].delta_v, WithinRel(expected_booster, 1e-12));
  CHECK_THAT(table[0].isp, WithinRel(320.0, 1e-12));

  CHECK_THAT(table[1].ignition_mass, WithinAbs(m0_upper, 1e-6));
  CHECK_THAT(table[1].final_mass, WithinAbs(m1_upper, 1e-6));
  CHECK_THAT(table[1].delta_v, WithinRel(expected_upper, 1e-12));

  CHECK_THAT(core::total_delta_v(vessel), WithinRel(expected_booster + expected_upper, 1e-12));

  SECTION("the booster is the heavy one and the upper stage is the empty one") {
    CHECK(table[0].final_mass > table[1].ignition_mass);
    CHECK(table[0].delta_v > 3000.0);
    CHECK(table[1].delta_v > 4000.0);
  }

  SECTION("burn time follows from the propellant and the mass flow") {
    const double flow_booster = table[0].thrust / (table[0].isp * g0);
    CHECK_THAT(table[0].burn_time, WithinRel(prop_booster / flow_booster, 1e-12));
    CHECK(table[0].burn_time > 150.0);
    CHECK(table[0].burn_time < 160.0);
  }

  SECTION("a stage with an engine but no tank of its own is worth nothing") {
    const core::Vessel starved =
        core::build_vessel({{"engine.terrier", 0}, {"tank.fl100", 1}}, core::PartCatalogue::stock());
    const std::vector<core::StageReport> starved_table = core::stage_table(starved);
    REQUIRE(starved_table.size() == 2);
    // The engine is on stage 0, which carries no tank. It cannot burn the fuel
    // sitting in the stage above it, and the table must say so rather than
    // crediting the design with a delta-v it cannot produce.
    CHECK_THAT(starved_table[0].delta_v, WithinAbs(0.0, 1e-12));
    CHECK_THAT(starved_table[0].propellant, WithinAbs(0.0, 1e-12));
  }
}

TEST_CASE("staging drops the parts it says it drops", "[vessel]") {
  core::Vessel vessel = core::build_vessel(tug_stack(), core::PartCatalogue::stock());
  const double wet = vessel.mass();
  REQUIRE(vessel.stage_count == 2);

  const core::JettisonedStage dropped = vessel.jettison();
  REQUIRE(dropped.parts.size() == 3);
  // Nothing has burned, so the booster leaves with a full load. A stage dropped
  // before it fires is heavier debris than one that ran dry, and the world has to
  // be able to say by how much.
  CHECK_THAT(dropped.propellant, WithinAbs(32000.0, 1e-9));

  // The booster's own dry mass plus the fuel still in it.
  CHECK_THAT(dropped.mass(), WithinAbs(200.0 + 4000.0 + 32000.0 + 3000.0, 1e-9));
  CHECK_THAT(dropped.radius(), WithinRel(2.5, 1e-12));

  CHECK(vessel.current_stage == 1);
  CHECK(vessel.parts.size() == 3);
  CHECK_THAT(vessel.mass(), WithinAbs(wet - dropped.mass(), 1e-9));

  // What is left is exactly the upper stage's worth, which is the figure the
  // stage table gave before anything was flown.
  const std::vector<core::StageReport> table = core::stage_table(vessel);
  CHECK_THAT(core::remaining_delta_v(vessel), WithinRel(table[1].delta_v, 1e-12));

  SECTION("the last stage is the core and cannot be dropped") {
    CHECK(vessel.jettison().empty());
    CHECK(vessel.current_stage == 1);
    CHECK(vessel.parts.size() == 3);
  }
}

TEST_CASE("propellant is drawn from the stage that is burning", "[vessel]") {
  core::Vessel vessel = core::build_vessel(tug_stack(), core::PartCatalogue::stock());

  CHECK_THAT(vessel.stage_propellant(0), WithinAbs(32000.0, 1e-9));
  CHECK_THAT(vessel.stage_propellant(1), WithinAbs(4000.0, 1e-9));

  // Drawing more than the stage holds takes what there is and no more, so a
  // caller that asked for a step it cannot afford gets told how much it got.
  const double drawn = vessel.draw_propellant(99999.0);
  CHECK_THAT(drawn, WithinAbs(32000.0, 1e-9));
  CHECK_THAT(vessel.stage_propellant(0), WithinAbs(0.0, 1e-9));
  CHECK_THAT(vessel.stage_propellant(1), WithinAbs(4000.0, 1e-9));
  CHECK_THAT(vessel.propellant_left(), WithinAbs(4000.0, 1e-9));

  // A dry stage produces nothing, however much fuel is aboard the one above it.
  vessel.throttle = 1.0;
  CHECK_THAT(vessel.thrust(), WithinAbs(0.0, 1e-12));
  CHECK_FALSE(vessel.can_thrust());
}

TEST_CASE("a full burn spends exactly what the tanks held", "[vessel]") {
  core::World world = world_with_vessel(tug_stack(), 400e3);
  const core::EntityId id = world.entities().front().id;

  const std::vector<core::StageReport> table = core::stage_table(world.find(id)->vessel);
  world.find(id)->vessel.throttle = 1.0;

  core::Seconds elapsed = 0.0;
  while (elapsed < 4000.0 && world.find(id)->vessel.can_thrust()) {
    world.advance(1.0, 1.0);
    elapsed += 1.0;
  }

  const core::Entity* vessel = world.find(id);
  REQUIRE(vessel != nullptr);
  CHECK_FALSE(vessel->vessel.can_thrust());

  // The upper stage's tank is untouched: a stage burns its own tanks and only
  // its own, which is what the stage table assumed.
  CHECK_THAT(vessel->vessel.propellant_left(), WithinAbs(4000.0, 1e-6));
  CHECK_THAT(vessel->mass, WithinAbs(table[0].final_mass, 1e-6));

  // The burn stopped when the tank ran dry, so the elapsed time is the table's
  // burn time to within the step the loop took.
  CHECK_THAT(elapsed, WithinAbs(table[0].burn_time, 1.0));
}

TEST_CASE("a short prograde burn does what the rocket equation says", "[vessel]") {
  // A single light stage in a circular orbit: the burn is a few seconds, so the
  // velocity barely rotates while it happens and the ideal impulsive answer is
  // the right one to compare against.
  core::World world = world_with_vessel(single_stage(), 400e3);
  const core::EntityId id = world.entities().front().id;

  const double mu = world.bodies().body(world.find(id)->parent).mu;
  const double radius = core::norm(world.find(id)->state.r);
  const double speed_before = core::norm(world.find(id)->state.v);

  const core::Vessel& design = world.find(id)->vessel;
  const double exhaust = design.exhaust_velocity();
  const double m0 = design.mass();
  const double start_propellant = design.propellant_left();

  // Burn 120 kg of a 4000 kg load, which is about 80 m/s on this design.
  const double burn = 120.0;
  const double target = start_propellant - burn;

  world.find(id)->vessel.throttle = 1.0;  // a zero thrust_dir means prograde
  while (world.find(id)->vessel.propellant_left() > target) {
    world.advance(0.25, 0.25);
  }

  const core::Entity* vessel = world.find(id);
  REQUIRE(vessel != nullptr);
  // The loop overshoots by at most one step's worth of propellant, which is a
  // few kilograms out of four tonnes.
  CHECK_THAT(vessel->vessel.propellant_left(), WithinAbs(target, 10.0));

  // 1. The rocket equation holds: the speed gained is the exhaust velocity times
  //    the log of the mass ratio actually flown. This is the claim the whole
  //    stage table rests on, and it is checked against a burn that happened
  //    rather than against the loop's own bookkeeping.
  const double m1 = vessel->mass;
  const double predicted = exhaust * std::log(m0 / m1);
  const double achieved = core::norm(vessel->state.v) - speed_before;
  INFO("burned " << (start_propellant - vessel->vessel.propellant_left()) << " kg, m0 " << m0
                 << " m1 " << m1 << ", achieved " << achieved << " m/s, predicted " << predicted
                 << " m/s");
  CHECK_THAT(achieved, WithinRel(predicted, 0.01));

  // 2. The orbit is what vis-viva says it should be. A prograde burn at
  //    periapsis raises the far side and leaves the near side alone, so the burn
  //    point stays the periapsis and the new orbit is pinned by the new speed.
  const core::OrbitalElements after = core::rv_to_elements(vessel->state, mu);
  const double speed_after = core::norm(vessel->state.v);
  const double semi_major = 1.0 / (2.0 / radius - speed_after * speed_after / mu);
  const double expected_apoapsis = 2.0 * semi_major - radius;

  CHECK_THAT(after.periapsis(), WithinRel(radius, 1e-3));
  CHECK_THAT(after.apoapsis(), WithinRel(expected_apoapsis, 0.02));
  CHECK(after.apoapsis() > radius);
}

TEST_CASE("the throttle scales the burn and leaves the delta-v alone", "[vessel]") {
  // The rocket equation says the impulse depends on how much propellant is
  // burned, not on how fast it is burned. So the same propellant at half
  // throttle must buy the same speed over twice the time. It is a cheap check
  // and it catches a throttle wired into the wrong term.
  struct Burn {
    double achieved{0.0};   // speed gained, straight off the state vector
    double predicted{0.0};  // the rocket equation applied to the mass actually spent
    double burned{0.0};     // [kg]
    double elapsed{0.0};    // [s]
  };

  const auto fly = [](double throttle) {
    core::World world = world_with_vessel(single_stage(), 400e3);
    const core::EntityId id = world.entities().front().id;
    const core::Entity& before = *world.find(id);

    Burn burn;
    const double m0 = before.vessel.mass();
    const double speed_before = core::norm(before.state.v);
    const double target = before.vessel.propellant_left() - 100.0;

    world.find(id)->vessel.throttle = throttle;
    // Small steps, because the loop stops on a threshold: a coarse step would
    // burn a different amount on each side and the comparison would end up
    // measuring the step rather than the throttle.
    while (burn.elapsed < 20000.0 && world.find(id)->vessel.propellant_left() > target) {
      world.advance(0.05, 0.05);
      burn.elapsed += 0.05;
    }

    const core::Entity& after = *world.find(id);
    burn.achieved = core::norm(after.state.v) - speed_before;
    burn.burned = m0 - after.mass;
    burn.predicted = before.vessel.exhaust_velocity() * std::log(m0 / after.mass);
    return burn;
  };

  const Burn full = fly(1.0);
  const Burn half = fly(0.5);

  INFO("full: " << full.achieved << " of " << full.predicted << " m/s in " << full.elapsed
                << " s; half: " << half.achieved << " of " << half.predicted << " m/s in "
                << half.elapsed << " s");

  // Each burn matches its own rocket equation, against the mass it actually
  // spent rather than the mass the loop meant to spend.
  CHECK_THAT(full.achieved, WithinRel(full.predicted, 0.01));
  CHECK_THAT(half.achieved, WithinRel(half.predicted, 0.01));

  // And the two agree, which is the part the throttle could break.
  CHECK_THAT(half.burned, WithinRel(full.burned, 0.02));
  CHECK_THAT(half.achieved, WithinRel(full.achieved, 0.02));
  CHECK_THAT(half.elapsed, WithinRel(2.0 * full.elapsed, 0.05));

  SECTION("a throttle of zero produces neither thrust nor mass flow") {
    core::World world = world_with_vessel(single_stage(), 400e3);
    const core::EntityId id = world.entities().front().id;
    world.find(id)->vessel.throttle = 0.0;
    const core::StateVector before = world.find(id)->state;
    world.advance(600.0, 60.0);
    CHECK_THAT(world.find(id)->vessel.propellant_left(), WithinAbs(4000.0, 1e-9));

    // Coasting, the only force left is gravity — and, at 400 km, a trace of
    // air. The Keplerian propagation is what the state has to agree with to
    // within that trace, so this is the assertion that nothing *else* got in:
    // a stray force from the integrator, a stage that quietly lit, or a drag
    // term applied where there is no air would all miss by far more.
    const core::StateVector expected =
        core::propagate(before, world.bodies().body(world.find(id)->parent).mu, 600.0);
    const double drift = core::norm(world.find(id)->state.r - expected.r);
    INFO("drift from the Keplerian orbit over ten minutes: " << drift << " m");
    CHECK(drift < 1.0);
  }
}

TEST_CASE("a dropped stage becomes debris that the world still accounts for", "[vessel]") {
  core::World world = world_with_vessel(tug_stack(), 400e3);
  const core::EntityId id = world.entities().front().id;

  const double before = world.find(id)->mass;
  const core::StateVector state_at_separation = world.find(id)->state;
  REQUIRE(world.entities().size() == 1);

  const core::EntityId debris = world.jettison_stage(id);
  REQUIRE(debris != core::kInvalidEntity);
  REQUIRE(world.entities().size() == 2);

  const core::Entity* dropped = world.find(debris);
  REQUIRE(dropped != nullptr);
  CHECK(dropped->kind == core::EntityKind::Debris);
  CHECK_FALSE(dropped->controllable);
  CHECK(dropped->vessel.empty());
  CHECK(dropped->name.find("stage 0") != std::string::npos);

  // Mass is conserved: what left the vessel is exactly what the debris weighs.
  CHECK_THAT(world.find(id)->mass + dropped->mass, WithinAbs(before, 1e-9));

  // It leaves with the parent's state at the instant of separation, so it keeps
  // flying in company rather than snapping to the origin.
  CHECK_THAT(dropped->state.r.x, WithinAbs(state_at_separation.r.x, 1e-9));
  CHECK_THAT(dropped->state.v.y, WithinAbs(state_at_separation.v.y, 1e-9));
  CHECK(dropped->parent == world.find(id)->parent);

  SECTION("jettisoning an entity that has no stage left does nothing") {
    const core::EntityId other = world.entities().front().id;  // the tug itself
    // The outer scope already dropped stage 0, so what is left is the vessel's
    // core. There is nothing above it to jettison to, and refusing is what keeps
    // the vessel flyable rather than empty.
    CHECK(world.jettison_stage(other) == core::kInvalidEntity);
    CHECK(world.entities().size() == 2);
  }
}

TEST_CASE("a scenario carries parts and no masses of its own", "[vessel][scenario]") {
  core::Scenario scenario;
  scenario.name = "round trip";
  core::ScenarioEntity spec;
  spec.name = "Tug";
  spec.parent_body = "Earth";
  spec.parts = tug_stack();
  scenario.entities = {spec};

  const std::string text = scenario::write_scenario(scenario);
  // A vessel built from parts writes its parts and nothing else. Two sources for
  // one mass is how a file ends up disagreeing with itself.
  CHECK(text.find("parts") != std::string::npos);
  CHECK(text.find("\"mass\"") == std::string::npos);

  const core::Scenario back = scenario::parse_scenario(text);
  REQUIRE(back.entities.size() == 1);
  REQUIRE(back.entities[0].parts.size() == 6);
  CHECK(back.entities[0].parts[0].part == "probe.core");
  CHECK(back.entities[0].parts[0].stage == 1);
  CHECK(back.entities[0].parts[5].part == "engine.skipper");
  CHECK(back.entities[0].parts[5].stage == 0);

  // The stack survives the trip through the catalogue unchanged, which is the
  // part that matters: a round trip that loses a stage is a lost stage.
  const core::Vessel vessel = core::build_vessel(back.entities[0].parts, core::PartCatalogue::stock());
  CHECK_THAT(core::total_delta_v(vessel),
             WithinRel(core::total_delta_v(core::build_vessel(tug_stack(), core::PartCatalogue::stock())),
                       1e-12));
}

TEST_CASE("a point mass has no propulsion to report", "[vessel][scenario]") {
  core::Scenario scenario;
  core::ScenarioEntity spec;
  spec.name = "Probe";
  spec.parent_body = "Earth";
  spec.periapsis_altitude = 400e3;
  spec.apoapsis_altitude = 400e3;
  scenario.entities = {spec};

  auto source = simhost::LocalSimSource::from_scenario(scenario);
  const proto::EntitySnapshot* entity = hud::find_entity(source.snapshot(), source.snapshot().selected);
  REQUIRE(entity != nullptr);
  CHECK(entity->stage_count == 0);

  const std::vector<hud::Row> rows = hud::entity_readout(source.snapshot(), *entity);
  for (const hud::Row& row : rows) {
    CHECK(row.label != "delta-v");
    CHECK(row.label != "throttle");
  }
}

TEST_CASE("a vessel publishes its stage, its fuel and the delta-v that is left", "[vessel][simhost]") {
  core::Scenario scenario;
  scenario.name = "tug";
  core::ScenarioEntity spec;
  spec.name = "Tug";
  spec.parent_body = "Earth";
  spec.periapsis_altitude = 400e3;
  spec.apoapsis_altitude = 400e3;
  spec.parts = tug_stack();
  scenario.entities = {spec};

  auto source = simhost::LocalSimSource::from_scenario(scenario);
  const proto::EntitySnapshot* entity =
      hud::find_entity(source.snapshot(), source.snapshot().selected);
  REQUIRE(entity != nullptr);

  CHECK(entity->stage_count == 2);
  CHECK(entity->stage == 0);
  CHECK_THAT(entity->propellant_capacity, WithinAbs(36000.0, 1e-6));
  CHECK_THAT(entity->propellant, WithinAbs(36000.0, 1e-6));
  CHECK_THAT(entity->mass, WithinAbs(44290.0, 1e-6));
  CHECK_THAT(entity->delta_v, WithinRel(core::total_delta_v(core::build_vessel(tug_stack(), core::PartCatalogue::stock())), 1e-9));

  const std::vector<hud::Row> rows = hud::entity_readout(source.snapshot(), *entity);
  const auto row = [&rows](std::string_view label) -> const hud::Row* {
    for (const hud::Row& candidate : rows) {
      if (candidate.label == label) {
        return &candidate;
      }
    }
    return nullptr;
  };
  REQUIRE(row("stage") != nullptr);
  CHECK(row("stage")->value == "1 of 2");
  CHECK(row("throttle")->value == "off");
  CHECK(row("propellant")->value == "36.00 t of 36.00 t");
  CHECK(row("delta-v")->value == "9.24 km/s");

  SECTION("the stage and the throttle reach the snapshot through commands") {
    source.send(proto::Command::stage());
    source.pump(0.0);
    // The tug is built from parts, so staging it leaves debris behind: the world
    // does not lose the mass it is no longer flying.
    CHECK(source.snapshot().entity_count == 2);
    CHECK(source.snapshot().entities[0].stage == 1);
    CHECK(source.snapshot().entities[0].stage_count == 2);
    CHECK_THAT(source.snapshot().entities[0].mass, WithinAbs(5090.0, 1e-6));

    // The delta-v of the entity the selection points at is its own, not the
    // debris'. Staging must not leave the readout describing the wrong object.
    const proto::EntitySnapshot* after =
        hud::find_entity(source.snapshot(), source.snapshot().selected);
    REQUIRE(after != nullptr);
    CHECK(after->id == source.snapshot().entities[0].id);
    CHECK_THAT(after->delta_v, WithinAbs(5.215e3, 5.0));

    // Throttle is a separate command, sent afterwards because a pump advances
    // the clock by the real time that has passed and would otherwise eat a
    // fraction of a gram before the mass above was read.
    source.send(proto::Command::set_throttle(1.0));
    source.pump(0.0);
    CHECK_THAT(source.snapshot().entities[0].throttle, WithinAbs(1.0, 1e-12));
    CHECK(source.snapshot().entities[0].thrust > 0.0);
  }
}
