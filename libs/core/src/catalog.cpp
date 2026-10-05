#include "rocketlab/core/body.hpp"

#include <algorithm>
#include <cmath>
#include <initializer_list>
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
  /// The north pole at J2000, as right ascension and declination in the
  /// equatorial frame. Used twice: it is the spin axis the atmosphere turns
  /// about, and it is the pole of the Laplace plane a moon's elements are
  /// published in.
  double pole_ra_deg;
  double pole_dec_deg;
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
///
/// Poles are the IAU/WGCCRE J2000 values, which the catalogue holds fixed: the
/// 26 000-year precession of the equinoxes would move a pole by half a degree
/// over the span this simulator is usable for.
const BodySpec kPlanets[] = {
    {"Mercury", kMuMercury, 2439.7e3, 58.6462 * kSecondsPerDay, 281.0103, 61.4155,
     0.38709927, 0.00000037, 0.20563593, 0.00001906, 7.00497902, -0.00594749,
     252.25032350, 149472.67411175, 77.45779628, 0.16047689, 48.33076593, -0.12534081},
    {"Venus", kMuVenus, 6051.8e3, -243.025 * kSecondsPerDay, 272.76, 67.16,
     0.72333566, 0.00000390, 0.00677672, -0.00004107, 3.39467605, -0.00078890,
     181.97909950, 58517.81538729, 131.60246718, 0.00268329, 76.67984255, -0.27769418},
    {"Earth", kMuEarth, kRadiusEarth, 86164.0905, 0.0, 90.0,
     1.00000261, 0.00000562, 0.01671123, -0.00004392, -0.00001531, -0.01294668,
     100.46457166, 35999.37244981, 102.93768193, 0.32327364, 0.0, 0.0},
    {"Mars", kMuMars, kRadiusMars, 88642.66, 317.68143, 52.88650,
     1.52371034, 0.00001847, 0.09339410, 0.00007882, 1.84969142, -0.00813131,
     -4.55343205, 19140.30268499, -23.94362959, 0.44441088, 49.55953891, -0.29257343},
    {"Jupiter", kMuJupiter, 69911e3, 35730.0, 268.056595, 64.495303,
     5.20288700, -0.00011607, 0.04838624, -0.00013253, 1.30439695, -0.00183714,
     34.39644051, 3034.74612775, 14.72847983, 0.21252668, 100.47390909, 0.20469106},
    {"Saturn", kMuSaturn, 58232e3, 38362.0, 40.589, 83.537,
     9.53667594, -0.00125060, 0.05386179, -0.00050991, 2.48599187, 0.00193609,
     49.95424423, 1222.49362201, 92.59887831, -0.41897216, 113.66242448, -0.28867794},
    {"Uranus", kMuUranus, 25.362e6, -0.71833 * kSecondsPerDay, 257.311, -15.175,
     19.18916464, -0.00196176, 0.04725744, -0.00004397, 0.77263783, -0.00242939,
     313.23810451, 428.48202785, 170.95427630, 0.40805281, 74.01692503, 0.04240589},
    {"Neptune", kMuNeptune, 24.622e6, 0.6713 * kSecondsPerDay, 299.36, 43.46,
     30.06992276, 0.00026291, 0.00859048, 0.00005105, 1.77004347, 0.00035372,
     -55.12002969, 218.45945325, 44.96476227, -0.32241464, 131.78422574, -0.00508664},
};

/// The Moon, geocentric, from the mean elements given in Meeus, "Astronomical
/// Algorithms", chapter 47. Only the largest secular terms are kept: the node
/// regresses in 18.6 years and the perigee advances in 8.85, both of which the
/// rates below reproduce.
///
/// This one is the exception to the rule below: the Moon's orbit precesses
/// about the *ecliptic* pole rather than the Earth's equator, so its elements
/// were already published ecliptic-referred and need no rotation.
const BodySpec kMoon = {
    "Moon", kMuMoon, kRadiusMoon, 2360591.0, 0.0, 90.0,
    0.00256955, 0.0, 0.0549006, 0.0, 5.1453964, 0.0,
    218.3164477, 481267.88123421, 83.3532465, 4069.0137287, 125.0445479, -1934.13626197};

/// A moon, published the way the satellite tables publish one: referred to the
/// primary's Laplace plane, which for a body close enough to be worth flying
/// to is the primary's equator.
///
/// The mean motion is not tabulated. It follows from the semi-major axis and
/// the primary's mass by Kepler's third law, so deriving it removes a column
/// of data that could only ever disagree with the two columns next to it.
///
/// The node and the argument of periapsis are taken as zero. Every moon here
/// is on a nearly circular orbit in its primary's equatorial plane, where both
/// angles are genuinely ill-determined — they move by degrees over a decade —
/// so a tabulated value would be false precision rather than information. It
/// puts each moon at the ascending node of its Laplace plane on the ecliptic
/// at the epoch, which is the one place on that orbit worth naming.
struct MoonSpec {
  const char* name;
  const char* parent;
  double mu;
  double radius;
  double a_metres;
  double e;
  double i_deg;  // to the primary's Laplace plane
};

const MoonSpec kMoons[] = {
    {"Phobos", "Mars", kMuPhobos, kRadiusPhobos, 9376.0e3, 0.0151, 1.075},
    {"Deimos", "Mars", kMuDeimos, kRadiusDeimos, 23463.2e3, 0.00033, 1.788},
    {"Io", "Jupiter", kMuIo, kRadiusIo, 421.8e6, 0.0041, 0.036},
    {"Europa", "Jupiter", kMuEuropa, kRadiusEuropa, 671.1e6, 0.0094, 0.466},
    {"Ganymede", "Jupiter", kMuGanymede, kRadiusGanymede, 1070.4e6, 0.0013, 0.177},
    {"Callisto", "Jupiter", kMuCallisto, kRadiusCallisto, 1882.7e6, 0.0074, 0.192},
    {"Titan", "Saturn", kMuTitan, kRadiusTitan, 1221.87e6, 0.0288, 0.348},
    // The one retrograde moon in the catalogue, and the reason the rotation
    // into the ecliptic is written as a rotation rather than as "add the
    // primary's obliquity".
    {"Triton", "Neptune", kMuTriton, kRadiusTriton, 354.759e6, 0.000016, 156.885},
};

/// The plane a moon's elements are published in, given the primary's pole.
///
/// A pole at ecliptic longitude `l` puts the ascending node of its equator on
/// the ecliptic at `l - 90`, which is the same statement as the vector one
/// below: the node direction is the ecliptic pole crossed into the body pole.
[[nodiscard]] ReferencePlane laplace_plane(const Vec3& pole) noexcept {
  return ReferencePlane{std::acos(std::clamp(pole.z, -1.0, 1.0)),
                        wrap_angle(std::atan2(pole.x, -pole.y))};
}

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
    body.spin_axis = pole_to_ecliptic(spec.pole_ra_deg, spec.pole_dec_deg);
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

  for (const MoonSpec& spec : kMoons) {
    const std::optional<BodyId> parent_id = system.find(spec.parent);
    if (!parent_id.has_value()) {
      throw std::logic_error("rocketlab: catalogue moon names an unknown primary");
    }
    const CelestialBody primary = system.body(*parent_id);
    const double a = spec.a_metres;
    const ReferencePlane plane = laplace_plane(primary.spin_axis);
    const PlaneElements ecliptic = to_ecliptic(spec.i_deg * kDegreesToRadians, 0.0, 0.0, plane);

    // Kepler's third law, in the units the ephemeris wants: degrees per
    // Julian century.
    const double period = kTwoPi * std::sqrt(a * a * a / primary.mu);
    const double rate = 360.0 / period * (kSecondsPerDay * kDaysPerCentury);

    CelestialBody body;
    body.id = id_of(system.bodies_.size());
    body.name = spec.name;
    body.parent = *parent_id;
    body.mu = spec.mu;
    body.radius = spec.radius;
    // Tidally locked, so its spin and its orbit are the same period. There is
    // no separate figure to look up and no way for the two to disagree.
    body.rotation_period = period;
    body.spin_axis = primary.spin_axis;
    body.soi_radius = sphere_of_influence(a, spec.mu, primary.mu);
    body.ephemeris.semi_major_axis = {a / kAstronomicalUnit, 0.0};
    body.ephemeris.eccentricity = {spec.e, 0.0};
    body.ephemeris.inclination = {ecliptic.inclination / kDegreesToRadians, 0.0};
    body.ephemeris.longitude_of_ascending_node = {ecliptic.ascending_node / kDegreesToRadians, 0.0};
    body.ephemeris.longitude_of_periapsis = {
        (ecliptic.ascending_node + ecliptic.argument_periapsis) / kDegreesToRadians, 0.0};
    body.ephemeris.mean_longitude = {body.ephemeris.longitude_of_periapsis.value, rate};
    system.bodies_.push_back(body);
  }

  // Air. The Earth gets the full table because its scale height changes by a
  // factor of ten between the ground and the thermosphere, and it is the one
  // atmosphere in the catalogue that a mission actually spends time in. The
  // rest are single exponentials, which the same interpolator reproduces
  // exactly from two rows.
  const auto set_air = [&system](std::string_view name, std::initializer_list<AtmosphereLayer> rows) {
    const std::optional<BodyId> id = system.find(name);
    if (!id.has_value()) {
      throw std::logic_error("rocketlab: catalogue gives air to a body it does not have");
    }
    system.bodies_[*id - 1].atmosphere.layers.assign(rows);
  };
  set_air("Earth", {{0.0, 1.225},       {25.0e3, 3.899e-2},  {50.0e3, 1.057e-3},
                    {75.0e3, 3.206e-5}, {100.0e3, 5.297e-7}, {150.0e3, 2.070e-9},
                    {200.0e3, 2.789e-10}, {300.0e3, 1.916e-11}, {400.0e3, 3.725e-12},
                    {500.0e3, 1.000e-12}, {700.0e3, 3.614e-13}, {1000.0e3, 3.561e-15}});
  set_air("Venus", {{0.0, 65.0}, {200.0e3, 2.24e-4}});
  set_air("Mars", {{0.0, 0.020}, {200.0e3, 2.98e-10}});
  set_air("Titan", {{0.0, 5.3}, {600.0e3, 2.1e-12}});

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
