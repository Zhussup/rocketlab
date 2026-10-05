#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>

#include "rocketlab/core/constants.hpp"
#include "rocketlab/core/world.hpp"

using namespace rocketlab::core;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

/// Deliberately local. `kPi` is not in the catalogue's constants on purpose —
/// every angle in the core is handled through `wrap_angle` and the two-pi
/// family, and a bare pi sitting in a shared header is an invitation to write
/// the kind of trig that only works in the first quadrant.
constexpr double kPi = 3.141592653589793;
constexpr double kE = 2.718281828459045;

/// The drag the circular-orbit decay law predicts, worked out from the body's
/// own table rather than from the simulation's internals.
///
/// A circular orbit in a thin atmosphere loses radius according to
/// dr/dt = -2 r^2 a_t v / mu, where a_t = 1/2 rho Cd A v_rel^2 / m is the
/// tangential deceleration. It is the textbook result and it is independent of
/// how `World` happens to integrate, which is what makes it worth comparing
/// against: a missing factor of a half, an area that forgot the pi, or a mass
/// that never divided shows up as a ratio that is off by a clean factor.
[[nodiscard]] double circular_decay_per_period(const CelestialBody& body, double altitude,
                                               double mass, double radius) {
  const double r = body.radius + altitude;
  const double v = std::sqrt(body.mu / r);
  const double co_rotation = kTwoPi / body.rotation_period * r;
  const double relative = v - co_rotation;
  const double density = body.density_at(Vec3{r, 0.0, 0.0});
  constexpr double kCd = 2.2;
  const double area = kPi * radius * radius;
  const double deceleration = 0.5 * density * kCd * area * relative * relative / mass;
  const double rate = -2.0 * r * r * deceleration * v / body.mu;
  return rate * (kTwoPi * std::sqrt(r * r * r / body.mu));
}

[[nodiscard]] StateVector circular_state(const CelestialBody& body, double altitude,
                                         double inclination) {
  OrbitalElements el;
  el.p = body.radius + altitude;
  el.i = inclination;
  return elements_to_rv(el, body.mu);
}

/// A world with one point-like vessel on a circular orbit, and a handle to it.
struct Trial {
  World world{BodySystem::solar_system()};
  BodyId earth{*world.bodies().find("Earth")};
  EntityId id{kInvalidEntity};

  Trial(double altitude, double mass, double radius, double inclination) {
    Entity entity;
    entity.name = "Probe";
    entity.kind = EntityKind::Debris;
    entity.parent = earth;
    entity.mass = mass;
    entity.radius = radius;
    entity.state = circular_state(world.bodies().body(earth), altitude, inclination);
    id = world.add(entity);
  }

  [[nodiscard]] const Entity& entity() const { return *world.find(id); }
  [[nodiscard]] double radius() const { return norm(entity().state.r); }
  [[nodiscard]] OrbitalElements elements() const {
    return rv_to_elements(entity().state, world.bodies().body(earth).mu);
  }
  /// The semi-major axis as an altitude. This, and not the instantaneous
  /// radius, is the quantity a drag decay moves monotonically: a slightly
  /// eccentric orbit swings about its mean radius by more than a day's decay,
  /// so sampling the radius samples the phase as much as the drag.
  [[nodiscard]] double altitude() const {
    return elements().semi_major_axis() - world.bodies().body(earth).radius;
  }
};

}  // namespace

TEST_CASE("the density profile is exponential between its rows", "[atmosphere]") {
  const BodySystem system = BodySystem::solar_system();
  const CelestialBody& earth = system.body(*system.find("Earth"));

  SECTION("the tabulated rows come back exactly") {
    CHECK_THAT(earth.atmosphere.density_at(0.0), WithinRel(1.225, 1e-12));
    CHECK_THAT(earth.atmosphere.density_at(200e3), WithinRel(2.789e-10, 1e-12));
    CHECK_THAT(earth.atmosphere.density_at(400e3), WithinRel(3.725e-12, 1e-12));
    // The last row's altitude is the top of the atmosphere, so its density is
    // the limit approached from below and never a value returned.
    CHECK_THAT(earth.atmosphere.density_at(1000e3 - 1e-6), WithinRel(3.561e-15, 1e-5));
  }

  SECTION("between two rows the density is their geometric mean at the midpoint") {
    // Log-linear interpolation: the exponential through the rows at 400 km and
    // 500 km has its midpoint value at the midpoint altitude, so the geometric
    // mean is the value the model must produce — and the square of the
    // midpoint value must be the product of the endpoints.
    const double mid = earth.atmosphere.density_at(450e3);
    CHECK_THAT(mid * mid, WithinRel(3.725e-12 * 1.000e-12, 1e-12));
  }

  SECTION("below the surface row the value is held, above the top it is zero") {
    CHECK_THAT(earth.atmosphere.density_at(-10e3), WithinRel(1.225, 1e-12));
    CHECK(earth.atmosphere.density_at(1000e3 + 1.0) == 0.0);
    CHECK(earth.atmosphere.density_at(1.0e9) == 0.0);
  }

  SECTION("an airless body has no air at any altitude") {
    const BodySystem catalogue = BodySystem::solar_system();
    const CelestialBody& moon = catalogue.body(*catalogue.find("Moon"));
    CHECK(moon.atmosphere.empty());
    CHECK(catalogue.body(*catalogue.find("Mercury")).atmosphere.empty());
    CHECK(moon.density_at(Vec3{kRadiusMoon + 100.0, 0.0, 0.0}) == 0.0);
    // Mars has air, but nowhere near this high.
    CHECK(catalogue.body(*catalogue.find("Mars")).density_at(Vec3{kRadiusMars + 300e3, 0.0, 0.0}) == 0.0);
  }
}

TEST_CASE("a two-row atmosphere is exactly one scale height", "[atmosphere]") {
  const BodySystem system = BodySystem::solar_system();
  const CelestialBody& mars = system.body(*system.find("Mars"));

  const double surface = 0.020;
  const double top = 2.98e-10;
  const double scale_height = 200e3 / std::log(surface / top);
  CHECK_THAT(scale_height, WithinRel(11.1e3, 0.02));  // published: 11.1 km

  CHECK_THAT(mars.atmosphere.density_at(scale_height), WithinRel(surface / kE, 1e-9));
  CHECK_THAT(mars.atmosphere.density_at(2.0 * scale_height), WithinRel(surface / (kE * kE), 1e-9));
}

TEST_CASE("the air turns with its planet", "[atmosphere]") {
  const BodySystem system = BodySystem::solar_system();
  const CelestialBody& earth = system.body(*system.find("Earth"));

  const Vec3 equator{kRadiusEarth, 0.0, 0.0};
  const Vec3 wind = earth.surface_velocity(equator);

  // 465 m/s at the equator. It is the number the whole co-rotating atmosphere
  // exists to produce: a launch to the east gets it for free.
  CHECK_THAT(norm(wind), WithinRel(464.6, 0.002));
  // Along the spin axis it is nothing at all, which is what makes a polar
  // launch cheaper to compute and no cheaper to fly.
  CHECK_THAT(norm(earth.surface_velocity(earth.spin_axis * kRadiusEarth)), WithinAbs(0.0, 1e-9));
  // omega x r is perpendicular to both, so the wind neither climbs nor turns
  // inwards.
  CHECK_THAT(dot(wind, earth.spin_axis), WithinAbs(0.0, 1e-9));
  CHECK_THAT(dot(wind, equator), WithinAbs(0.0, 1e-6));

  SECTION("a body with no modelled spin is still air") {
    CelestialBody still = earth;
    still.rotation_period = 0.0;
    CHECK_THAT(norm(still.surface_velocity(equator)), WithinAbs(0.0, 1e-12));
  }
}

TEST_CASE("drag lowers a low orbit and leaves a high one alone", "[atmosphere][drag]") {
  SECTION("a circular orbit at 400 km decays at the textbook rate") {
    Trial trial(400e3, 1000.0, 1.0, 0.0);
    const CelestialBody& earth = trial.world.bodies().body(trial.earth);
    const double before = earth.radius + 400e3;
    const double period = kTwoPi * std::sqrt(std::pow(before, 3.0) / earth.mu);

    trial.world.advance(period, 60.0);
    const double dropped = 400e3 - trial.altitude();
    const double expected = -circular_decay_per_period(earth, 400e3, 1000.0, 1.0);

    INFO("dropped " << dropped << " m, the decay law predicts " << expected << " m");
    // The orbit descends as it goes, so it ends up in thicker air than it
    // started in and loses slightly more than the linear estimate. A few per
    // cent is the size of that; a factor of two would be a bug.
    CHECK(dropped > 0.97 * expected);
    CHECK(dropped < 1.10 * expected);
  }

  SECTION("the descent accelerates day on day") {
    Trial trial(400e3, 1000.0, 1.0, 0.0);
    double previous = trial.altitude();
    double first_day = 0.0;
    double last_day = 0.0;
    for (int day = 1; day <= 10; ++day) {
      trial.world.advance(kSecondsPerDay, 60.0);
      const double current = trial.altitude();
      const double dropped = previous - current;
      previous = current;
      INFO("day " << day << " dropped " << dropped << " m");
      CHECK(dropped > 50.0);
      CHECK(dropped < 500.0);
      if (day == 1) {
        first_day = dropped;
      }
      last_day = dropped;
    }
    // Ten days lower is ten days deeper into the profile, and the density
    // climbs by about a fifth of a per cent per day over that range, so the
    // last day has to cost more than the first. It is a small margin on
    // purpose: it is the sign that is being tested, not the magnitude.
    CHECK(last_day > first_day);
    CHECK_THAT(trial.elements().e, WithinAbs(0.0, 1e-5));
    CHECK(trial.altitude() > 300e3);
  }

  SECTION("a point mass has no cross-section and feels nothing") {
    Trial trial(400e3, 1000.0, 0.0, 0.0);
    const CelestialBody& earth = trial.world.bodies().body(trial.earth);
    const StateVector expected =
        propagate(trial.entity().state, earth.mu, kSecondsPerDay);
    trial.world.advance(kSecondsPerDay, 60.0);
    // Not bit-identical: the day is flown as 1,440 analytic steps rather than
    // one, and composing them rounds. What matters is that the difference is a
    // rounding error — a drag that leaked in would be a hundred metres.
    CHECK(norm(trial.entity().state.r - expected.r) < 1e-3);
  }

  SECTION("above the top of the profile the propagation is exactly Keplerian") {
    // Deliberately past the 1000 km top row: a body whose periapsis never
    // reaches the air must take the analytic path, or time warp stops being
    // free for everything in high orbit.
    Trial trial(1100e3, 1000.0, 1.0, 0.0);
    const CelestialBody& earth = trial.world.bodies().body(trial.earth);
    const StateVector expected = propagate(trial.entity().state, earth.mu, kSecondsPerDay);
    trial.world.advance(kSecondsPerDay, 60.0);
    CHECK(norm(trial.entity().state.r - expected.r) < 1e-3);

    // At 1000 km exactly — the top of the table — the orbit survives a day
    // with only centimetres lost. The profile is finite there, but barely.
    Trial grazing(1000e3, 1000.0, 1.0, 0.0);
    const double before = grazing.altitude();
    grazing.world.advance(kSecondsPerDay, 60.0);
    const double dropped = before - grazing.altitude();
    INFO("dropped " << dropped << " m in a day at the top of the profile");
    CHECK(dropped >= 0.0);
    CHECK(dropped < 1.0);
  }
}

TEST_CASE("drag acts along the relative wind, and only in the orbital plane", "[atmosphere][drag]") {
  SECTION("a retrograde orbit meets faster air and decays sooner") {
    Trial prograde(400e3, 1000.0, 1.0, 0.0);
    Trial retrograde(400e3, 1000.0, 1.0, kPi);
    const double pro_before = prograde.radius();
    const double retro_before = retrograde.radius();

    prograde.world.advance(kSecondsPerDay, 60.0);
    retrograde.world.advance(kSecondsPerDay, 60.0);
    const double pro_drop = pro_before - prograde.radius();
    const double retro_drop = retro_before - retrograde.radius();

    // Relative to the air, the retrograde orbit moves at v + omega*r instead of
    // v - omega*r, so it loses (8166/7179)^2 = 1.29 times as much radius.
    INFO("prograde dropped " << pro_drop << " m, retrograde " << retro_drop << " m");
    CHECK(retro_drop / pro_drop > 1.15);
    CHECK(retro_drop / pro_drop < 1.45);
  }

  SECTION("an equatorial orbit keeps its plane") {
    // The wind runs along the orbit, so there is no out-of-plane force to tilt
    // it. An inclined orbit would slowly be dragged into the equatorial plane,
    // which is a real effect and a different test.
    Trial trial(400e3, 1000.0, 1.0, 0.35);
    const double before = trial.elements().i;
    trial.world.advance(kSecondsPerDay, 60.0);
    CHECK_THAT(trial.elements().i, WithinRel(before, 1e-6));
  }
}

TEST_CASE("an airless body drags nothing", "[atmosphere][drag]") {
  World world(BodySystem::solar_system());
  const BodyId moon = *world.bodies().find("Moon");
  const CelestialBody& body = world.bodies().body(moon);

  Entity entity;
  entity.name = "Low pass";
  entity.kind = EntityKind::Debris;
  entity.parent = moon;
  entity.mass = 1000.0;
  entity.radius = 1.0;
  entity.state = circular_state(body, 100e3, 0.0);

  const StateVector expected = propagate(entity.state, body.mu, kSecondsPerDay);
  const EntityId id = world.add(entity);
  world.advance(kSecondsPerDay, 60.0);

  // 100 km above the Moon is hard vacuum, so the path is the analytic one to
  // within the rounding of composing it a step at a time.
  CHECK(norm(world.find(id)->state.r - expected.r) < 1e-3);
}
