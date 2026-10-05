#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "rocketlab/core/orbital.hpp"
#include "rocketlab/core/time.hpp"

namespace rocketlab::core {

using BodyId = std::uint32_t;

/// Zero is never a valid body: it marks "no parent", which is what the root of
/// the body tree carries.
inline constexpr BodyId kInvalidBody = 0;

/// One orbital element that drifts linearly with time.
///
/// This is the form JPL publishes for the major planets: a value at J2000 and
/// a rate per Julian century. Over 1800-2050 it lands within a few arcminutes,
/// which is far finer than a patched-conic simulator can use, and evaluating it
/// costs one multiply per element.
struct KeplerianRate {
  double value{0.0};
  double rate{0.0};

  [[nodiscard]] constexpr double at(double centuries_since_j2000) const noexcept {
    return value + rate * centuries_since_j2000;
  }
};

struct BodyEphemeris {
  KeplerianRate semi_major_axis;               // au
  KeplerianRate eccentricity;
  KeplerianRate inclination;                   // degrees
  KeplerianRate mean_longitude;                // degrees
  KeplerianRate longitude_of_periapsis;        // degrees
  KeplerianRate longitude_of_ascending_node;   // degrees

  /// Heliocentric-ish elements at a simulation instant. The angles are
  /// converted out of JPL's longitude form into the anomaly form the rest of
  /// the core uses.
  [[nodiscard]] OrbitalElements elements_at(Seconds tdb) const noexcept;
};

/// One row of a density profile.
struct AtmosphereLayer {
  double altitude{0.0};  // [m] above the body's mean radius
  double density{0.0};   // [kg/m^3]
};

/// An exponential atmosphere, given as a table of (altitude, density) rows and
/// interpolated log-linearly between them.
///
/// Log-linear is the right interpolation because density is exponential in
/// altitude: between two rows the model is exactly a single exponential whose
/// scale height is the one implied by the two densities. A body with one
/// constant scale height is therefore a two-row table, and the Earth — whose
/// scale height runs from 6 km at the surface to 60 km in the thermosphere —
/// is a twelve-row table. Same code, same cost, no special cases.
///
/// The rows are the usual US Standard Atmosphere figures, rounded to the
/// precision that matters for drag. Above the top row the density is zero,
/// which is what lets a caller ask for the density anywhere without also
/// carrying a range test around with it.
struct Atmosphere {
  std::vector<AtmosphereLayer> layers;  // ascending altitude

  [[nodiscard]] bool empty() const noexcept { return layers.empty(); }

  /// [kg/m^3] at `altitude` above the body's mean radius. Below the first row
  /// the surface value is held, above the last it is zero.
  [[nodiscard]] double density_at(double altitude) const noexcept;
};

struct CelestialBody {
  BodyId id{kInvalidBody};
  std::string name;
  BodyId parent{kInvalidBody};  // kInvalidBody marks the root of the tree
  double mu{0.0};               // gravitational parameter [m^3/s^2]
  double radius{0.0};          // mean radius [m]
  double soi_radius{0.0};      // sphere of influence [m]; zero for the root
  double rotation_period{0.0}; // sidereal spin [s]; zero if not modelled
  /// Direction of the spin axis in the parent frame. Only the co-rotating
  /// atmosphere reads it, so a body that has none may leave it at the default.
  Vec3 spin_axis{0.0, 0.0, 1.0};
  Atmosphere atmosphere;       // empty for an airless body
  BodyEphemeris ephemeris;     // orbit around `parent`; unused for the root

  [[nodiscard]] bool is_root() const noexcept { return parent == kInvalidBody; }

  /// Velocity of the body's rotating atmosphere at `offset` from its centre,
  /// expressed in the parent frame [m/s]: omega x r. Zero when the body has no
  /// modelled spin, so an airless or slowly rotating body costs nothing.
  [[nodiscard]] Vec3 surface_velocity(const Vec3& offset) const noexcept;

  /// Density [kg/m^3] at `offset` from the body's centre, zero for an airless
  /// body or a point above the top of the profile.
  [[nodiscard]] double density_at(const Vec3& offset) const noexcept {
    return atmosphere.density_at(norm(offset) - radius);
  }

  /// State relative to this body's parent at simulation time `tdb`. Returns a
  /// zero state for the root. `parent_mu` is passed in rather than stored so
  /// that the body itself stays a plain data record.
  [[nodiscard]] StateVector state_relative_to_parent(Seconds tdb, double parent_mu) const noexcept;
};

/// The tree of bodies a world is built on, plus the lookups everything else
/// needs. Bodies are addressed by a small integer id, never by name, so that
/// hot paths never touch a string.
class BodySystem {
 public:
  /// A body tree with the Sun at the root, the planets, and the Moon.
  ///
  /// Elements are JPL's approximate ephemerides for 1800-2050. Spheres of
  /// influence are derived from mu rather than hardcoded, so they stay
  /// consistent if the table is ever edited.
  [[nodiscard]] static BodySystem solar_system();

  [[nodiscard]] const std::vector<CelestialBody>& bodies() const noexcept { return bodies_; }

  /// Throws `std::out_of_range` if the id is not in this system.
  [[nodiscard]] const CelestialBody& body(BodyId id) const;

  [[nodiscard]] std::optional<BodyId> find(std::string_view name) const noexcept;

  /// State in the root frame (heliocentric ecliptic), obtained by walking the
  /// parent chain. This is what the renderer consumes.
  [[nodiscard]] StateVector root_state(BodyId id, Seconds tdb) const;

  /// The innermost body whose sphere of influence contains `root_position`.
  /// Descends from the root, so a vessel that has climbed out of a planet's
  /// SOI is correctly handed back to the parent.
  [[nodiscard]] BodyId dominant_body(const Vec3& root_position, Seconds tdb) const;

  [[nodiscard]] BodyId root_id() const noexcept;

 private:
  std::vector<CelestialBody> bodies_;
};

/// An orbit whose elements were published in one plane, restated in another.
///
/// The satellite tables give a moon's inclination, node and argument of
/// periapsis referred to the primary's Laplace plane — which for a close moon
/// is the primary's equator — together with the inclination and node of that
/// plane on the ecliptic. Everything else in the catalogue is ecliptic-
/// referred, so the two conventions have to be reconciled exactly once, when
/// the catalogue is built. Reconciling them per call would put a rotation into
/// every ephemeris evaluation.
///
/// The mean anomaly is untouched: the orbit is the same orbit, and rotating
/// the plane it is described in does not change where the moon is along it.
struct PlaneElements {
  double inclination{0.0};        // [rad], measured from the ecliptic
  double ascending_node{0.0};     // [rad], on the ecliptic
  double argument_periapsis{0.0}; // [rad], in the ecliptic plane
};

/// The reference plane itself: its inclination to the ecliptic and the
/// ecliptic longitude of its ascending node.
struct ReferencePlane {
  double inclination{0.0};     // [rad]
  double ascending_node{0.0};  // [rad]
};

[[nodiscard]] PlaneElements to_ecliptic(double inclination, double ascending_node,
                                        double argument_periapsis,
                                        const ReferencePlane& from) noexcept;

}  // namespace rocketlab::core
