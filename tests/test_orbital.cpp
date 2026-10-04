#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <cmath>

#include "rocketlab/core/constants.hpp"
#include "rocketlab/core/orbital.hpp"

using namespace rocketlab::core;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

constexpr double kMu = kMuEarth;
constexpr double kPi = 3.14159265358979323846;

/// Relative distance between two states, scaled by the larger of the two so
/// the tolerance means the same thing whether we are in LEO or on a
/// heliocentric transfer.
[[nodiscard]] double state_error(const StateVector& a, const StateVector& b) {
  const double r_scale = std::max(norm(a.r), norm(b.r));
  const double v_scale = std::max(norm(a.v), norm(b.v));
  const double r_err = norm(a.r - b.r) / (r_scale > 0.0 ? r_scale : 1.0);
  const double v_err = norm(a.v - b.v) / (v_scale > 0.0 ? v_scale : 1.0);
  return std::max(r_err, v_err);
}

[[nodiscard]] StateVector state_from(double p, double e, double i, double raan,
                                     double argp, double nu) {
  OrbitalElements el;
  el.p = p;
  el.e = e;
  el.i = i;
  el.raan = raan;
  el.argp = argp;
  el.nu = nu;
  return elements_to_rv(el, kMu);
}

}  // namespace

TEST_CASE("a state survives a round trip through orbital elements", "[orbital]") {
  // A spread of orbit families that exercise every degenerate branch in
  // rv_to_elements: near-circular, exactly circular, equatorial, retrograde
  // equatorial, polar, hyperbolic and parabolic.
  struct Case {
    const char* name;
    double p;
    double e;
    double i;
    double raan;
    double argp;
    double nu;
  };

  const Case cases[] = {
      {"LEO inclined", 7000e3, 0.001, 0.9006, 0.5, 0.3, 1.2},
      {"Molniya-like", 26554e3 * (1.0 - 0.74 * 0.74), 0.74, 1.106, 2.0, 4.0, 0.7},
      {"equatorial eccentric", 9000e3, 0.2, 0.0, 0.0, 1.1, 2.2},
      {"retrograde equatorial", 8000e3, 0.05, kPi, 0.0, 0.7, 2.9},
      {"polar", 12000e3, 0.3, kPi / 2.0, 3.0, 5.0, 0.2},
      {"circular inclined", 7000e3, 0.0, 0.9, 0.5, 0.0, 1.2},
      {"circular equatorial", 7000e3, 0.0, 0.0, 0.0, 0.0, 2.5},
      {"hyperbolic", 7000e3 * 2.5, 1.5, 0.5, 1.0, 0.4, 0.5},
      {"parabolic", 2.0 * 7000e3, 1.0, 0.6, 2.0, 1.0, 0.4},
  };

  for (const Case& c : cases) {
    INFO("orbit: " << c.name);
    const StateVector original =
        state_from(c.p, c.e, c.i, c.raan, c.argp, c.nu);
    const OrbitalElements elements = rv_to_elements(original, kMu);
    REQUIRE_FALSE(elements.degenerate);
    const StateVector recovered = elements_to_rv(elements, kMu);
    CHECK(state_error(original, recovered) < 1e-9);
  }
}

TEST_CASE("elements are recovered exactly when the orbit is not degenerate", "[orbital]") {
  struct Case {
    double p, e, i, raan, argp, nu;
  };
  const Case cases[] = {
      {7000e3, 0.001, 0.9006, 0.5, 0.3, 1.2},
      {26554e3 * (1.0 - 0.74 * 0.74), 0.74, 1.106, 2.0, 4.0, 0.7},
      {12000e3, 0.3, kPi / 2.0, 3.0, 5.0, 0.2},
  };

  for (const Case& c : cases) {
    const StateVector state = state_from(c.p, c.e, c.i, c.raan, c.argp, c.nu);
    const OrbitalElements el = rv_to_elements(state, kMu);
    CHECK_THAT(el.p, WithinRel(c.p, 1e-9));
    CHECK_THAT(el.e, WithinRel(c.e, 1e-9));
    CHECK_THAT(el.i, WithinRel(c.i, 1e-9));
    CHECK_THAT(el.raan, WithinRel(c.raan, 1e-9));
    CHECK_THAT(el.argp, WithinRel(c.argp, 1e-9));
    CHECK_THAT(el.nu, WithinRel(c.nu, 1e-9));
  }
}

TEST_CASE("a circular orbit returns to its starting point after one period", "[orbital]") {
  const StateVector start = state_from(7000e3, 0.0, 0.9, 0.5, 0.0, 0.0);
  const OrbitalElements el = rv_to_elements(start, kMu);
  const double period = el.period(kMu);

  REQUIRE(period > 0.0);
  const StateVector after = propagate(start, kMu, period);
  CHECK(state_error(start, after) < 1e-9);

  // Half a period lands on the antipode, which catches a sign error that a
  // full revolution would hide.
  const StateVector half = propagate(start, kMu, 0.5 * period);
  CHECK_THAT(norm(half.r + start.r), WithinAbs(0.0, 1.0));  // metres
}

TEST_CASE("propagation preserves the shape of the conic", "[orbital]") {
  const StateVector start = state_from(26554e3 * (1.0 - 0.74 * 0.74), 0.74, 1.106, 2.0, 4.0, 0.7);
  const OrbitalElements before = rv_to_elements(start, kMu);

  for (double dt : {1.0, 60.0, 3600.0, 86400.0, 7.0 * 86400.0}) {
    INFO("dt = " << dt);
    const OrbitalElements after = rv_to_elements(propagate(start, kMu, dt), kMu);
    CHECK_THAT(after.p, WithinRel(before.p, 1e-9));
    CHECK_THAT(after.e, WithinRel(before.e, 1e-9));
    CHECK_THAT(after.i, WithinRel(before.i, 1e-9));
    CHECK_THAT(after.raan, WithinRel(before.raan, 1e-9));
  }
}

TEST_CASE("propagation composes and reverses", "[orbital]") {
  const StateVector start = state_from(12000e3, 0.3, kPi / 2.0, 3.0, 5.0, 0.2);
  const double step = 137.0;
  const int count = 500;

  // One big step must agree with many small ones: the propagation is analytic,
  // so any discrepancy means the Kepler solver is losing accuracy.
  StateVector stepped = start;
  for (int i = 0; i < count; ++i) {
    stepped = propagate(stepped, kMu, step);
  }
  const StateVector one_shot = propagate(start, kMu, step * count);
  CHECK(state_error(one_shot, stepped) < 1e-9);

  // Running time backwards retraces the same path.
  const StateVector back = propagate(one_shot, kMu, -step * count);
  CHECK(state_error(start, back) < 1e-9);
}

TEST_CASE("hyperbolic and parabolic orbits propagate both ways", "[orbital]") {
  SECTION("hyperbolic") {
    const StateVector start = state_from(7000e3 * 2.5, 1.5, 0.5, 1.0, 0.4, 0.5);
    REQUIRE(classify(rv_to_elements(start, kMu)) == ConicKind::Hyperbolic);
    const StateVector after = propagate(start, kMu, 3000.0);
    // An escaping orbit must gain radius and lose speed.
    CHECK(norm(after.r) > norm(start.r));
    CHECK(norm(after.v) < norm(start.v));
    // Energy is the invariant that proves the conic was not silently changed.
    const double energy_before = specific_energy(start, kMu);
    const double energy_after = specific_energy(after, kMu);
    CHECK_THAT(energy_after, WithinRel(energy_before, 1e-12));
  }

  SECTION("parabolic") {
    const StateVector start = state_from(2.0 * 7000e3, 1.0, 0.6, 2.0, 1.0, 0.4);
    REQUIRE(classify(rv_to_elements(start, kMu)) == ConicKind::Parabolic);
    const StateVector after = propagate(start, kMu, 3000.0);
    CHECK(norm(after.r) > norm(start.r));
  }
}

TEST_CASE("known orbits come out at the right scale", "[orbital]") {
  SECTION("geostationary") {
    const StateVector geo = state_from(42164e3, 0.0, 0.0, 0.0, 0.0, 0.0);
    const OrbitalElements el = rv_to_elements(geo, kMu);
    // A sidereal day, to within a part in a thousand.
    CHECK_THAT(el.period(kMu), WithinRel(86164.1, 1e-3));
  }

  SECTION("ISS-like") {
    const StateVector iss = state_from(6771e3, 0.0005, 0.9006, 0.0, 0.0, 0.0);
    const OrbitalElements el = rv_to_elements(iss, kMu);
    // Roughly 92 minutes.
    CHECK_THAT(el.period(kMu), WithinAbs(5545.0, 60.0));
  }

  SECTION("escape speed at the surface is about 11.2 km/s") {
    OrbitalElements el;
    el.p = 2.0 * kRadiusEarth;  // parabola from the surface
    el.e = 1.0;
    el.i = 0.0;
    el.raan = 0.0;
    el.argp = 0.0;
    el.nu = 0.0;
    const StateVector surface = elements_to_rv(el, kMu);
    CHECK_THAT(norm(surface.v), WithinAbs(11186.0, 50.0));
  }
}

TEST_CASE("a radial trajectory is reported as degenerate rather than as NaN", "[orbital]") {
  StateVector radial;
  radial.r = Vec3{-7.0e6, 0.0, 0.0};
  radial.v = Vec3{-100.0, 0.0, 0.0};

  const OrbitalElements el = rv_to_elements(radial, kMu);
  CHECK(el.degenerate);

  // Propagation must pass it through untouched instead of emitting garbage.
  const StateVector after = propagate(radial, kMu, 10.0);
  CHECK(after.r.x == radial.r.x);
  CHECK(after.r.y == radial.r.y);
  CHECK(after.r.z == radial.r.z);
}
