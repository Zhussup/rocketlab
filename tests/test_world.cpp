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

  SECTION("the moons are all there, at their published spheres of influence") {
    struct Published {
      const char* name;
      const char* parent;
      double soi_metres;
    };
    // a (m/M)^(2/5), rounded, as published for each satellite.
    const Published moons[] = {
        {"Phobos", "Mars", 7.2e3},       {"Deimos", "Mars", 8.1e3},
        {"Io", "Jupiter", 7.83e6},       {"Europa", "Jupiter", 9.72e6},
        {"Ganymede", "Jupiter", 24.3e6}, {"Callisto", "Jupiter", 37.7e6},
        {"Titan", "Saturn", 43.3e6},     {"Triton", "Neptune", 11.9e6},
    };
    for (const Published& moon : moons) {
      INFO(moon.name);
      const std::optional<BodyId> id = system.find(moon.name);
      REQUIRE(id.has_value());
      CHECK_THAT(system.body(*id).soi_radius, WithinRel(moon.soi_metres, 0.02));
      CHECK(system.body(system.body(*id).parent).name == moon.parent);
    }
  }

  SECTION("every moon sits at its published distance from its primary") {
    // Semi-major axis times one plus or minus the eccentricity brackets the
    // whole orbit, so a moon that came out at the wrong distance — or the
    // wrong scale, which is the easy mistake with a table in kilometres —
    // leaves the bracket whatever the epoch.
    struct Orbit {
      const char* name;
      const char* parent;
      double a_metres;
      double e;
    };
    const Orbit orbits[] = {
        {"Phobos", "Mars", 9376.0e3, 0.0151},      {"Deimos", "Mars", 23463.2e3, 0.00033},
        {"Io", "Jupiter", 421.8e6, 0.0041},        {"Europa", "Jupiter", 671.1e6, 0.0094},
        {"Ganymede", "Jupiter", 1070.4e6, 0.0013}, {"Callisto", "Jupiter", 1882.7e6, 0.0074},
        {"Titan", "Saturn", 1221.87e6, 0.0288},    {"Triton", "Neptune", 354.759e6, 0.000016},
    };
    for (const Orbit& orbit : orbits) {
      const BodyId moon = *system.find(orbit.name);
      const BodyId primary = *system.find(orbit.parent);
      for (int day = 0; day < 30; day += 2) {
        const Vec3 offset = system.root_state(moon, day * kSecondsPerDay).r -
                            system.root_state(primary, day * kSecondsPerDay).r;
        INFO(orbit.name << " on day " << day);
        CHECK(norm(offset) > orbit.a_metres * (1.0 - 1.01 * orbit.e));
        CHECK(norm(offset) < orbit.a_metres * (1.0 + 1.01 * orbit.e));
      }
    }
  }

  SECTION("the outer planets' orbits are the right size") {
    struct Giant {
      const char* name;
      double a_au;
      double e;
      double period_years;
    };
    const Giant giants[] = {
        {"Uranus", 19.18916464, 0.04725744, 84.011},
        {"Neptune", 30.06992276, 0.00859048, 164.79},
    };
    for (const Giant& giant : giants) {
      INFO(giant.name);
      const CelestialBody& planet = system.body(*system.find(giant.name));
      CHECK_THAT(planet.ephemeris.semi_major_axis.at(0.0), WithinRel(giant.a_au, 1e-6));
      CHECK_THAT(planet.ephemeris.eccentricity.at(0.0), WithinRel(giant.e, 1e-6));

      // The period follows from that distance and the Sun's mass, so this is a
      // check on both against the published value — and on the fact that no
      // separate mean motion was tabulated to drift away from them.
      const double a = planet.ephemeris.semi_major_axis.at(0.0) * kAu;
      const double period = kTwoPi * std::sqrt(a * a * a / kMuSun);
      CHECK_THAT(period / (365.25 * kSecondsPerDay), WithinRel(giant.period_years, 0.002));

      // Neither giant moves far in a year, so its distance stays between the
      // perihelion and the aphelion its own elements imply.
      for (int day = 0; day < 365; day += 30) {
        const double distance =
            norm(system.root_state(*system.find(giant.name), day * kSecondsPerDay).r);
        INFO("day " << day);
        CHECK(distance > giant.a_au * (1.0 - 1.01 * giant.e) * kAu);
        CHECK(distance < giant.a_au * (1.0 + 1.01 * giant.e) * kAu);
      }
    }
  }
}

TEST_CASE("every moon orbits in its primary's equatorial plane", "[world]") {
  const BodySystem system = BodySystem::solar_system();

  // The inclinations the catalogue was built from, to the primary's Laplace
  // plane — for a close moon, the primary's equator. These are the numbers
  // that pin the rotation out of that plane and into the ecliptic: get the
  // order of the two rotations wrong and Io reads 0.85 degrees off Jupiter's
  // pole instead of 0.036, and Titan 55 instead of 0.35.
  struct Published {
    const char* moon;
    const char* primary;
    double inclination_deg;
    bool retrograde;
  };
  const Published moons[] = {
      {"Phobos", "Mars", 1.075, false},      {"Deimos", "Mars", 1.788, false},
      {"Io", "Jupiter", 0.036, false},       {"Europa", "Jupiter", 0.466, false},
      {"Ganymede", "Jupiter", 0.177, false}, {"Callisto", "Jupiter", 0.192, false},
      {"Titan", "Saturn", 0.348, false},
      // Triton is the only one that goes the other way round, and the reason
      // the catalogue has to rotate a plane rather than add an obliquity.
      {"Triton", "Neptune", 156.885, true},
  };

  for (const Published& entry : moons) {
    INFO(entry.moon);
    const BodyId moon = *system.find(entry.moon);
    const BodyId primary = *system.find(entry.primary);
    const StateVector moon_state = system.root_state(moon, 0.0);
    const StateVector primary_state = system.root_state(primary, 0.0);
    const StateVector relative{moon_state.r - primary_state.r, moon_state.v - primary_state.v};

    const Vec3 normal = cross(relative.r, relative.v);
    const Vec3 pole = system.body(primary).spin_axis;
    const double measured =
        std::acos(std::clamp(dot(normalized(normal), pole), -1.0, 1.0)) * 180.0 / 3.141592653589793;

    CHECK_THAT(measured, WithinAbs(entry.inclination_deg, 1e-6));
    CHECK((dot(normalized(normal), pole) < 0.0) == entry.retrograde);
  }
}

TEST_CASE("an orbit is restated in another plane without changing its shape", "[world]") {
  const BodySystem system = BodySystem::solar_system();
  const Vec3 pole = system.body(*system.find("Earth")).spin_axis;

  // The normal of a plane given as an inclination and a node. Applied to a
  // reference plane it is that plane's own pole; applied to an orbit's
  // elements it is the orbit's normal, and the angle between the two is the
  // inclination of the orbit to the plane.
  const auto normal = [](double inclination, double node) {
    return Vec3{std::sin(inclination) * std::sin(node), -std::sin(inclination) * std::cos(node),
                std::cos(inclination)};
  };

  // Earth's equator reaches the ecliptic by being tilted by the obliquity: the
  // vector form of the same statement the catalogue makes.
  const ReferencePlane equator{std::acos(std::clamp(pole.z, -1.0, 1.0)),
                               wrap_angle(std::atan2(pole.x, -pole.y))};
  CHECK_THAT(equator.inclination * 180.0 / 3.141592653589793, WithinAbs(23.4392911, 1e-5));

  SECTION("a reference plane that is the ecliptic changes nothing") {
    const PlaneElements same = to_ecliptic(0.7, 1.1, 2.3, ReferencePlane{0.0, 0.0});
    CHECK_THAT(same.inclination, WithinRel(0.7, 1e-12));
    CHECK_THAT(same.ascending_node, WithinRel(1.1, 1e-12));
    CHECK_THAT(same.argument_periapsis, WithinRel(2.3, 1e-12));
  }

  SECTION("a prograde equatorial orbit comes out at the obliquity") {
    const PlaneElements restated = to_ecliptic(0.0, 0.0, 0.0, equator);
    CHECK_THAT(restated.inclination * 180.0 / 3.141592653589793, WithinAbs(23.4392911, 1e-5));
  }

  SECTION("the inclination to the reference plane is what survives") {
    // The restatement is a rigid rotation of a plane, so the angle between the
    // orbit's normal and the reference plane's pole has to be the same before
    // and after — measured once in the reference plane's own coordinates, and
    // once in the ecliptic. This is the property the moons depend on, and the
    // one that a rotation composed in the wrong order destroys: it would read
    // Io's normal as 0.85 degrees off Jupiter's pole instead of 0.036.
    const ReferencePlane neptune = [&system] {
      const Vec3 npole = system.body(*system.find("Neptune")).spin_axis;
      return ReferencePlane{std::acos(std::clamp(npole.z, -1.0, 1.0)),
                            wrap_angle(std::atan2(npole.x, -npole.y))};
    }();
    const Vec3 reference_pole = normal(neptune.inclination, neptune.ascending_node);

    constexpr double kDegrees = 3.141592653589793 / 180.0;
    const double inclinations[] = {0.0, 0.036 * kDegrees, 0.466 * kDegrees, 2.5, 156.885 * kDegrees};
    const double nodes[] = {0.0, 0.7, 2.3, 4.9};

    for (double inclination : inclinations) {
      for (double node : nodes) {
        const PlaneElements restated = to_ecliptic(inclination, node, 1.4, neptune);
        const Vec3 restated_normal = normal(restated.inclination, restated.ascending_node);
        INFO("i " << inclination << " node " << node);
        CHECK_THAT(std::acos(std::clamp(dot(restated_normal, reference_pole), -1.0, 1.0)),
                   WithinAbs(inclination, 1e-9));
        // And an orbit in the plane itself stays in the plane.
        CHECK(std::isfinite(restated.inclination));
        CHECK(std::isfinite(restated.ascending_node));
        CHECK(std::isfinite(restated.argument_periapsis));
      }
    }
  }
}

TEST_CASE("a vessel is captured by a moon and handed back to its primary", "[world]") {
  World world(BodySystem::solar_system());
  const BodySystem& system = world.bodies();
  const BodyId earth = *system.find("Earth");
  const BodyId moon = *system.find("Moon");

  const StateVector earth_state = system.root_state(earth, 0.0);
  const StateVector moon_state = system.root_state(moon, 0.0);
  const Vec3 to_moon = moon_state.r - earth_state.r;
  const Vec3 with_moon = moon_state.v - earth_state.v;

  SECTION("arriving from outside the sphere of influence") {
    // 70,000 km out — outside the Moon's 66,100 km sphere of influence — and
    // closing at 1.5 km/s, which is several times the lunar escape speed at
    // that range so the flyby is a hyperbola and the vessel leaves again. The
    // traverse is drawn slightly across the approach so the closest pass is
    // 22,000 km: inside the sphere of influence, well outside the Moon.
    Entity entity;
    entity.name = "Flyby";
    entity.parent = earth;
    entity.state.r = to_moon + Vec3{70e6, 0.0, 0.0};
    entity.state.v = with_moon + Vec3{-1500.0, 500.0, 0.0};
    const EntityId id = world.add(entity);

    // At the epoch this point is the Earth's: nothing has been captured yet.
    CHECK(system.dominant_body(world.root_state(*world.find(id)).r, 0.0) == earth);

    // Entry takes about 2,600 s, so a step of 3,000 s has to have re-framed it.
    world.advance(3000.0, 60.0);
    REQUIRE(world.find(id) != nullptr);
    CHECK(world.find(id)->parent == moon);

    // Exit is at about 81,000 s; from there the vessel is a satellite of the
    // Earth again rather than a satellite of the Moon, which is the capture's
    // mirror image and the case a "descend from the root" search gets right
    // where a "test the parent's own sphere" one would strand it.
    world.advance(2.0 * kSecondsPerDay, 60.0);
    REQUIRE(world.find(id) != nullptr);
    CHECK(world.find(id)->parent == earth);
    CHECK(system.dominant_body(world.root_state(*world.find(id)).r, world.clock().tdb()) == earth);
  }

  SECTION("already inside it when the epoch starts") {
    // The path that has to reframe at once, with no boundary to bisect for:
    // the vessel is inside the Moon's sphere of influence from the first step.
    const double radius = 40e6;
    Entity entity;
    entity.name = "Arrival";
    entity.parent = earth;
    entity.state.r = to_moon + Vec3{radius, 0.0, 0.0};
    // Across the radius, so the trajectory is not the rectilinear one the
    // analytic propagator declines to handle.
    entity.state.v = with_moon + Vec3{0.0, 1500.0, 0.0};
    const EntityId id = world.add(entity);

    world.advance(60.0, 60.0);
    REQUIRE(world.find(id) != nullptr);
    CHECK(world.find(id)->parent == moon);
    // Re-framing is exact, so the state is still the offset the vessel was
    // built with, to within a minute of the Moon's gravity bending it. The
    // radius holds to a kilometre and the speed to a metre; what is being
    // tested is that no part of the state was invented in the switch.
    CHECK_THAT(norm(world.find(id)->state.r), WithinAbs(radius, 1e3));
    CHECK_THAT(norm(world.find(id)->state.v), WithinAbs(1500.0, 1.0));
  }
}

TEST_CASE("the sphere-of-influence crossing lands in the same place at any step", "[world]") {
  // The value of solving for the crossing rather than noticing it at the end of
  // a step: at a 3,600 s step the old behaviour flew an hour past the boundary
  // in the wrong body's gravity, which is a velocity error of order
  // (a_earth - a_sun) * t and a position error that grows from there. Here the
  // step size is an event granularity and nothing more.
  const auto escape = [](double max_step) {
    World world(BodySystem::solar_system());
    const CelestialBody& earth = world.bodies().body(*world.bodies().find("Earth"));
    OrbitalElements el;
    el.p = 7000e3 * 2.5;
    el.e = 1.5;
    el.i = 0.4;
    Entity entity;
    entity.name = "Escape";
    entity.parent = earth.id;
    entity.state = elements_to_rv(el, earth.mu);
    const EntityId id = world.add(entity);
    world.advance(5.0 * kSecondsPerDay, max_step);
    return world.root_state(*world.find(id));
  };

  const StateVector fine = escape(60.0);
  const StateVector coarse = escape(3600.0);
  INFO("position difference " << norm(fine.r - coarse.r) << " m, velocity "
                              << norm(fine.v - coarse.v) << " m/s");
  CHECK(norm(fine.r - coarse.r) < 1.0);
  CHECK(norm(fine.v - coarse.v) < 1e-4);
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
