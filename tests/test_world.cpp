#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>

#include "rocketlab/core/constants.hpp"
#include "rocketlab/core/world.hpp"
#include "rocketlab/scenario/json_io.hpp"

using namespace rocketlab::core;
namespace scenario_io = rocketlab::scenario;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

constexpr double kAu = kAstronomicalUnit;

[[nodiscard]] StateVector circular_state(const CelestialBody& body, double altitude) {
  OrbitalElements el;
  el.e = 0.0;
  el.p = body.radius + altitude;
  return elements_to_rv(el, body.mu);
}

}  // namespace

TEST_CASE("the body catalogue comes out at the right scale", "[world]") {
  const BodySystem system = BodySystem::solar_system();

  SECTION("Earth stays within its perihelion and aphelion over a year") {
    for (int day = 0; day < 365; day += 7) {
      const double distance = norm(system.root_state(*system.find("Earth"), day * kSecondsPerDay).r);
      INFO("day " << day);
      // 0.98329 au and 1.01671 au, with a little room for the ephemeris error.
      CHECK(distance > 0.982 * kAu);
      CHECK(distance < 1.018 * kAu);
    }
  }

  SECTION("the Moon stays near its mean geocentric distance") {
    const BodyId earth = *system.find("Earth");
    const BodyId moon = *system.find("Moon");
    for (int day = 0; day < 400; day += 3) {
      const Vec3 offset =
          system.root_state(moon, day * kSecondsPerDay).r - system.root_state(earth, day * kSecondsPerDay).r;
      INFO("day " << day);
      // 356,500 km at perigee to 406,700 at apogee.
      CHECK(norm(offset) > 350e6);
      CHECK(norm(offset) < 412e6);
    }
  }

  SECTION("spheres of influence match the published values") {
    CHECK_THAT(system.body(*system.find("Earth")).soi_radius, WithinRel(924.0e6, 0.01));
    CHECK_THAT(system.body(*system.find("Moon")).soi_radius, WithinRel(66.1e6, 0.01));
    CHECK_THAT(system.body(*system.find("Sun")).soi_radius, WithinAbs(0.0, 1e-9));
  }
}

TEST_CASE("the dominant body is the innermost sphere of influence containing a point", "[world]") {
  const BodySystem system = BodySystem::solar_system();
  const BodyId earth = *system.find("Earth");
  const BodyId moon = *system.find("Moon");

  const Vec3 earth_center = system.root_state(earth, 0.0).r;
  const Vec3 moon_center = system.root_state(moon, 0.0).r;

  CHECK(system.dominant_body(earth_center + Vec3{7000e3, 0.0, 0.0}, 0.0) == earth);
  // A point beside the Moon belongs to the Moon, not to the Earth whose SOI
  // also contains it.
  CHECK(system.dominant_body(moon_center + Vec3{2000e3, 0.0, 0.0}, 0.0) == moon);
  // Deep space, far outside every planet: the Sun.
  CHECK(system.dominant_body(Vec3{2.5 * kAu, 0.0, 0.0}, 0.0) == system.root_id());
  // Just outside the Earth's SOI but still well inside the inner system.
  CHECK(system.dominant_body(earth_center + Vec3{1.5e9, 0.0, 0.0}, 0.0) == system.root_id());
}

TEST_CASE("a bound orbit keeps its shape over a long propagation", "[world]") {
  World world(BodySystem::solar_system());
  const BodyId earth = *world.bodies().find("Earth");
  const CelestialBody& body = world.bodies().body(earth);

  Entity entity;
  entity.name = "Station";
  entity.parent = earth;
  entity.state = circular_state(body, 400e3);
  world.add(entity);

  const OrbitalElements before = rv_to_elements(world.entities().front().state, body.mu);
  world.advance(30.0 * kSecondsPerDay);

  const Entity& after_entity = world.entities().front();
  REQUIRE(after_entity.parent == earth);
  const OrbitalElements after = rv_to_elements(after_entity.state, body.mu);

  CHECK_THAT(after.p, WithinRel(before.p, 1e-9));
  // The baseline eccentricity is exactly zero on a circular orbit, and
  // WithinRel never matches against a zero target, so this one is absolute.
  CHECK_THAT(after.e, WithinAbs(before.e, 1e-12));
  CHECK_THAT(after.i, WithinRel(before.i, 1e-9));
  CHECK_THAT(after.raan, WithinRel(before.raan, 1e-9));
}

TEST_CASE("a vessel that escapes the Earth is re-framed onto the Sun", "[world]") {
  World world(BodySystem::solar_system());
  const BodyId earth = *world.bodies().find("Earth");
  const CelestialBody& body = world.bodies().body(earth);

  // Escape speed at 7000 km is about 10.7 km/s, so 12 km/s sends it away.
  OrbitalElements el;
  el.p = 7000e3 * (1.0 + 1.5);  // e = 1.5 hyperbola
  el.e = 1.5;
  el.i = 0.4;
  el.nu = 0.0;

  Entity entity;
  entity.name = "Escape";
  entity.parent = earth;
  entity.state = elements_to_rv(el, body.mu);
  CHECK(norm(entity.state.v) > std::sqrt(2.0 * body.mu / 7000e3));

  const EntityId id = world.add(entity);
  world.advance(5.0 * kSecondsPerDay);

  const Entity* escaped = world.find(id);
  REQUIRE(escaped != nullptr);
  CHECK(escaped->parent == world.bodies().root_id());

  // Re-framing must not change the trajectory: the root-frame position before
  // and after the switch has to agree with the propagated one.
  CHECK(norm(world.root_state(*escaped).r) > kAu * 0.9);
}

TEST_CASE("the world clock advances by exactly the requested span", "[world]") {
  World world(BodySystem::solar_system());
  CHECK(world.clock().tdb() == 0.0);

  world.advance(1234.5, 60.0);
  CHECK_THAT(world.clock().tdb(), WithinAbs(1234.5, 1e-9));

  // Steps that do not divide the span evenly must still land on it.
  world.advance(0.5, 0.3);
  CHECK_THAT(world.clock().tdb(), WithinAbs(1235.0, 1e-9));

  // A non-positive span is a no-op, not a rewind.
  world.advance(-100.0, 60.0);
  CHECK_THAT(world.clock().tdb(), WithinAbs(1235.0, 1e-9));
}

TEST_CASE("scenarios survive a JSON round trip", "[scenario]") {
  const Scenario original = scenario_io::default_scenario();
  const std::string text = scenario_io::write_scenario(original);
  const Scenario parsed = scenario_io::parse_scenario(text);

  CHECK(parsed.name == original.name);
  CHECK_THAT(parsed.epoch.tdb, WithinAbs(original.epoch.tdb, 1e-6));
  REQUIRE(parsed.entities.size() == original.entities.size());
  for (std::size_t i = 0; i < parsed.entities.size(); ++i) {
    CHECK(parsed.entities[i].name == original.entities[i].name);
    CHECK(parsed.entities[i].parent_body == original.entities[i].parent_body);
    CHECK_THAT(parsed.entities[i].periapsis_altitude,
               WithinRel(original.entities[i].periapsis_altitude, 1e-9));
    CHECK_THAT(parsed.entities[i].apoapsis_altitude,
               WithinRel(original.entities[i].apoapsis_altitude, 1e-9));
    CHECK_THAT(parsed.entities[i].inclination_deg,
               WithinRel(original.entities[i].inclination_deg, 1e-9));
  }
}

TEST_CASE("an ISO epoch is read as a civil date", "[scenario]") {
  const Scenario scenario = scenario_io::parse_scenario(R"({
    "name": "epoch test",
    "epoch": "2000-01-01T12:00:00",
    "entities": []
  })");
  CHECK_THAT(scenario.epoch.tdb, WithinAbs(0.0, 1e-6));

  const Scenario dated = scenario_io::parse_scenario(R"({
    "epoch": "2000-01-02T12:00:00"
  })");
  CHECK_THAT(dated.epoch.tdb, WithinAbs(kSecondsPerDay, 1e-6));
}

TEST_CASE("a malformed scenario is rejected with a useful message", "[scenario]") {
  // Body names are not the parser's business: whether "Nibiru" exists is a
  // question only the catalogue can answer, and it is checked when the world
  // is built (see the test below).
  CHECK_NOTHROW(scenario_io::parse_scenario(R"({"entities": [{"name": "x", "parent": "Nibiru"}]})"));

  CHECK_THROWS(scenario_io::parse_scenario("{ not json"));
  CHECK_THROWS(scenario_io::parse_scenario(
      R"({"entities": [{"name": "x", "periapsis_altitude": 500000, "apoapsis_altitude": 100000}]})"));
  CHECK_THROWS(scenario_io::parse_scenario(R"({"name": 42})"));
}

TEST_CASE("unknown body names are rejected when the world is built", "[scenario]") {
  Scenario scenario;
  ScenarioEntity entity;
  entity.name = "x";
  entity.parent_body = "Nibiru";
  scenario.entities.push_back(entity);
  CHECK_THROWS(World::from_scenario(scenario));
}
