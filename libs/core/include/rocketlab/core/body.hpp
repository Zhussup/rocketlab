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

struct CelestialBody {
  BodyId id{kInvalidBody};
  std::string name;
  BodyId parent{kInvalidBody};  // kInvalidBody marks the root of the tree
  double mu{0.0};               // gravitational parameter [m^3/s^2]
  double radius{0.0};          // mean radius [m]
  double soi_radius{0.0};      // sphere of influence [m]; zero for the root
  double rotation_period{0.0}; // sidereal spin [s]; zero if not modelled
  BodyEphemeris ephemeris;     // orbit around `parent`; unused for the root

  [[nodiscard]] bool is_root() const noexcept { return parent == kInvalidBody; }

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

}  // namespace rocketlab::core
