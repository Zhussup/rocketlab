#include "rocketlab/core/body.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "rocketlab/core/constants.hpp"

namespace rocketlab::core {
namespace {

constexpr double kDegreesToRadians = 0.017453292519943295;
constexpr double kDaysPerCentury = 36525.0;

/// Bodies are identified by their index in the catalogue, one-based, so that
/// zero can keep meaning "nothing".
[[nodiscard]] constexpr BodyId id_of(std::size_t index) noexcept {
  return static_cast<BodyId>(index + 1);
}

}  // namespace

OrbitalElements BodyEphemeris::elements_at(Seconds tdb) const noexcept {
  const double centuries = tdb / (kSecondsPerDay * kDaysPerCentury);

  const double a = semi_major_axis.at(centuries) * kAstronomicalUnit;
  const double e = eccentricity.at(centuries);
  const double mean_lon = mean_longitude.at(centuries) * kDegreesToRadians;
  const double periapsis_lon = longitude_of_periapsis.at(centuries) * kDegreesToRadians;
  const double node_lon = longitude_of_ascending_node.at(centuries) * kDegreesToRadians;

  OrbitalElements el;
  el.e = e;
  el.i = inclination.at(centuries) * kDegreesToRadians;
  el.raan = wrap_angle(node_lon);
  el.argp = wrap_angle(periapsis_lon - node_lon);
  el.p = a * (1.0 - e * e);
  // JPL publishes a mean longitude, so the anomaly has to be solved for.
  el.nu = wrap_angle(true_anomaly_from_mean(wrap_angle(mean_lon - periapsis_lon), e));
  return el;
}

StateVector CelestialBody::state_relative_to_parent(Seconds tdb, double parent_mu) const noexcept {
  if (is_root()) {
    return StateVector{};
  }
  return state_from_mean_elements(ephemeris.elements_at(tdb), parent_mu);
}

namespace {

/// Assembles one catalogue row and derives its sphere of influence from the
/// two gravitational parameters, so editing the table cannot leave the SOI
/// radii stale.
struct BodySpec {
  const char* name;
  double mu;
  double radius;
  double rotation_period;
  double a_au;
  double a_rate;
  double e;
  double e_rate;
  double i_deg;
  double i_rate;
  double mean_longitude_deg;
  double mean_longitude_rate;
  double periapsis_deg;
  double periapsis_rate;
  double node_deg;
  double node_rate;
};

/// JPL, "Approximate Positions of the Major Planets", table for 1800-2050 AD.
/// Elements are heliocentric and referred to the J2000 ecliptic; the row for
/// the Earth is really the Earth-Moon barycentre, which is a 4700 km error
/// that no part of this simulator can notice.
const BodySpec kPlanets[] = {
    {"Mercury", kMuMercury, 2439.7e3, 58.6462 * kSecondsPerDay,
     0.38709927, 0.00000037, 0.20563593, 0.00001906, 7.00497902, -0.00594749,
     252.25032350, 149472.67411175, 77.45779628, 0.16047689, 48.33076593, -0.12534081},
    {"Venus", kMuVenus, 6051.8e3, -243.025 * kSecondsPerDay,
     0.72333566, 0.00000390, 0.00677672, -0.00004107, 3.39467605, -0.00078890,
     181.97909950, 58517.81538729, 131.60246718, 0.00268329, 76.67984255, -0.27769418},
    {"Earth", kMuEarth, kRadiusEarth, 86164.0905,
     1.00000261, 0.00000562, 0.01671123, -0.00004392, -0.00001531, -0.01294668,
     100.46457166, 35999.37244981, 102.93768193, 0.32327364, 0.0, 0.0},
    {"Mars", kMuMars, kRadiusMars, 88642.66,
     1.52371034, 0.00001847, 0.09339410, 0.00007882, 1.84969142, -0.00813131,
     -4.55343205, 19140.30268499, -23.94362959, 0.44441088, 49.55953891, -0.29257343},
    {"Jupiter", kMuJupiter, 69911e3, 35730.0,
     5.20288700, -0.00011607, 0.04838624, -0.00013253, 1.30439695, -0.00183714,
     34.39644051, 3034.74612775, 14.72847983, 0.21252668, 100.47390909, 0.20469106},
    {"Saturn", kMuSaturn, 58232e3, 38362.0,
     9.53667594, -0.00125060, 0.05386179, -0.00050991, 2.48599187, 0.00193609,
     49.95424423, 1222.49362201, 92.59887831, -0.41897216, 113.66242448, -0.28867794},
};

/// The Moon, geocentric, from the mean elements given in Meeus, "Astronomical
/// Algorithms", chapter 47. Only the largest secular terms are kept: the node
/// regresses in 18.6 years and the perigee advances in 8.85, both of which the
/// rates below reproduce.
const BodySpec kMoon = {
    "Moon", kMuMoon, kRadiusMoon, 2360591.0,
    0.00256955, 0.0, 0.0549006, 0.0, 5.1453964, 0.0,
    218.3164477, 481267.88123421, 83.3532465, 4069.0137287, 125.0445479, -1934.13626197};

}  // namespace

BodySystem BodySystem::solar_system() {
  BodySystem system;

  CelestialBody sun;
  sun.id = id_of(system.bodies_.size());
  sun.name = "Sun";
  sun.mu = kMuSun;
  sun.radius = kRadiusSun;
  sun.rotation_period = 25.38 * kSecondsPerDay;
  system.bodies_.push_back(sun);

  const auto add = [&system](const BodySpec& spec, BodyId parent, double parent_mu) {
    CelestialBody body;
    body.id = id_of(system.bodies_.size());
    body.name = spec.name;
    body.parent = parent;
    body.mu = spec.mu;
    body.radius = spec.radius;
    body.rotation_period = spec.rotation_period;
    body.soi_radius = sphere_of_influence(spec.a_au * kAstronomicalUnit, spec.mu, parent_mu);
    body.ephemeris.semi_major_axis = {spec.a_au, spec.a_rate};
    body.ephemeris.eccentricity = {spec.e, spec.e_rate};
    body.ephemeris.inclination = {spec.i_deg, spec.i_rate};
    body.ephemeris.mean_longitude = {spec.mean_longitude_deg, spec.mean_longitude_rate};
    body.ephemeris.longitude_of_periapsis = {spec.periapsis_deg, spec.periapsis_rate};
    body.ephemeris.longitude_of_ascending_node = {spec.node_deg, spec.node_rate};
    system.bodies_.push_back(body);
    return body.id;
  };

  const BodyId sun_id = system.bodies_.front().id;
  BodyId earth_id = kInvalidBody;
  for (const BodySpec& spec : kPlanets) {
    const BodyId id = add(spec, sun_id, kMuSun);
    if (spec.name == std::string_view{"Earth"}) {
      earth_id = id;
    }
  }
  add(kMoon, earth_id, kMuEarth);

  return system;
}

const CelestialBody& BodySystem::body(BodyId id) const {
  if (id == kInvalidBody || id > bodies_.size()) {
    throw std::out_of_range("rocketlab: no such body id");
  }
  return bodies_[id - 1];
}

std::optional<BodyId> BodySystem::find(std::string_view name) const noexcept {
  const auto it = std::find_if(bodies_.begin(), bodies_.end(),
                               [name](const CelestialBody& b) { return b.name == name; });
  if (it == bodies_.end()) {
    return std::nullopt;
  }
  return it->id;
}

BodyId BodySystem::root_id() const noexcept {
  const auto it = std::find_if(bodies_.begin(), bodies_.end(),
                               [](const CelestialBody& b) { return b.is_root(); });
  return it == bodies_.end() ? kInvalidBody : it->id;
}

StateVector BodySystem::root_state(BodyId id, Seconds tdb) const {
  StateVector accumulated{};
  BodyId current = id;
  // Depth is at most two in the shipped catalogue, so the recursion is written
  // as a loop and needs no memoisation.
  while (current != kInvalidBody) {
    const CelestialBody& node = body(current);
    if (node.is_root()) {
      break;
    }
    const StateVector relative = node.state_relative_to_parent(tdb, body(node.parent).mu);
    accumulated.r += relative.r;
    accumulated.v += relative.v;
    current = node.parent;
  }
  return accumulated;
}

BodyId BodySystem::dominant_body(const Vec3& root_position, Seconds tdb) const {
  BodyId current = root_id();
  bool descended = true;
  while (descended) {
    descended = false;
    for (const CelestialBody& candidate : bodies_) {
      if (candidate.parent != current || candidate.is_root()) {
        continue;
      }
      const Vec3 offset = root_position - root_state(candidate.id, tdb).r;
      if (norm(offset) <= candidate.soi_radius) {
        current = candidate.id;
        descended = true;
        break;
      }
    }
  }
  return current;
}

}  // namespace rocketlab::core
