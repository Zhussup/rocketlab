#pragma once

#include <limits>

#include "rocketlab/core/vec3.hpp"

namespace rocketlab::core {

/// Cartesian state relative to a central body.
struct StateVector {
  Vec3 r;  // position [m]
  Vec3 v;  // velocity [m/s]
};

/// Classical orbital elements.
///
/// The shape is carried as the semi-latus rectum `p` rather than the
/// semi-major axis `a`. They are related by p = a(1 - e^2), but `p` stays
/// finite for every conic — including the parabolic case, where `a` diverges
/// — so storing `p` keeps one code path for ellipses, parabolas and
/// hyperbolas. Use `semi_major_axis()` when the orbital period is what you
/// want.
///
/// Angles are radians. For a circular orbit `argp` is zero and `nu` is
/// measured from the ascending node (the argument of latitude); for an
/// equatorial orbit `raan` is zero. Both degeneracies fold their lost degree
/// of freedom into `nu`, which makes `elements_to_rv` round-trip exactly.
struct OrbitalElements {
  double p{0.0};     // semi-latus rectum [m]
  double e{0.0};     // eccentricity
  double i{0.0};     // inclination [rad]
  double raan{0.0};  // right ascension of the ascending node [rad]
  double argp{0.0};  // argument of periapsis [rad]
  double nu{0.0};    // true anomaly [rad]

  /// True when the state had no well-defined orbital plane (a radial
  /// trajectory, to within a relative tolerance). Every other field is
  /// meaningless in that case, and `propagate` leaves such a state alone: no
  /// conic fits it, and a caller that needs it to move has to integrate it
  /// numerically.
  bool degenerate{false};

  /// a = p / (1 - e^2). Infinite for a parabola, negative for a hyperbola.
  [[nodiscard]] double semi_major_axis() const noexcept;

  /// Radius at periapsis, p / (1 + e).
  [[nodiscard]] double periapsis() const noexcept { return p / (1.0 + e); }

  /// Radius at apoapsis; infinite for parabolic and hyperbolic orbits.
  [[nodiscard]] double apoapsis() const noexcept;

  [[nodiscard]] double period(double mu) const noexcept;
};

enum class ConicKind { Elliptic, Parabolic, Hyperbolic };

/// Eccentricity band around 1.0 treated as parabolic.
inline constexpr double kParabolicTolerance = 1e-9;
/// Relative threshold on |n| / |h| below which an orbit counts as equatorial.
inline constexpr double kNodeTolerance = 1e-12;
/// Eccentricity below which an orbit counts as circular.
inline constexpr double kCircularTolerance = 1e-10;

/// Brings an angle into [0, 2pi), the conventional range for the three angular
/// orbital elements. atan2 alone hands back (-pi, pi], which names the same
/// direction but reads badly in telemetry and makes element-wise comparison
/// depend on which revolution you happen to be on.
[[nodiscard]] double wrap_angle(double radians) noexcept;

[[nodiscard]] ConicKind classify(const OrbitalElements& elements) noexcept;

/// Converts a Cartesian state into orbital elements about a body whose
/// gravitational parameter is `mu`.
[[nodiscard]] OrbitalElements rv_to_elements(const StateVector& state, double mu) noexcept;

/// Inverse of `rv_to_elements`.
[[nodiscard]] StateVector elements_to_rv(const OrbitalElements& elements, double mu) noexcept;

/// Analytic two-body propagation: moves the state forward by `dt` seconds
/// along its osculating conic.
///
/// This is exact for any `dt`, which is what makes time warp cheap — a warp
/// of 100000x costs exactly the same as 1x. The trade-off is that it cannot
/// represent forces that are not an inverse square, so anything under thrust,
/// drag or third-body perturbation must fall back to a numerical integrator.
///
/// A degenerate state — a rectilinear trajectory, which has no osculating
/// conic — is returned unchanged, and the caller that needs one to move has to
/// integrate it instead. That is a deliberate freeze rather than a NaN: the
/// analytic machinery has nothing to say about a radial orbit, and inventing a
/// number for it would put it in the snapshot.
[[nodiscard]] StateVector propagate(const StateVector& state, double mu, double dt) noexcept;

/// Specific orbital energy, v^2/2 - mu/r [J/kg]. Negative for a bound orbit.
[[nodiscard]] double specific_energy(const StateVector& state, double mu) noexcept;

/// sqrt(mu / |a|^3), the mean motion for elliptic and hyperbolic orbits alike.
[[nodiscard]] double mean_motion(double semi_major_axis, double mu) noexcept;

/// Mean anomaly (or Barker's parameter, on a parabola) at the elements' true
/// anomaly. Purely geometric: it depends on the conic's shape, not on mu.
[[nodiscard]] double mean_anomaly(const OrbitalElements& elements) noexcept;

/// Inverse of `mean_anomaly`: solves Kepler's equation and converts the
/// resulting anomaly into a true anomaly. Handles all three conic families, so
/// an ephemeris published as mean elements can be turned into a state.
[[nodiscard]] double true_anomaly_from_mean(double mean_anomaly, double eccentricity) noexcept;

/// Convenience for building a state from mean elements, which is how
/// ephemerides are published.
[[nodiscard]] StateVector state_from_mean_elements(const OrbitalElements& elements, double mu) noexcept;

/// Time since the last periapsis passage [s], in `[0, period)`.
///
/// Zero for a degenerate or unbound orbit, which has no period to be a fraction
/// of. Callers that care should check `degenerate` and the conic kind first.
[[nodiscard]] double time_since_periapsis(const OrbitalElements& elements, double mu) noexcept;

/// Time to the next apoapsis [s], in `[0, period)`.
///
/// Part of the same toolkit as `period`: an apsis is a property of the conic the
/// vessel is on, so when it will be reached follows from the elements rather
/// than from stepping the orbit forward. A flight computer that has to burn at
/// apoapsis needs this and should not be re-deriving it, because a Kepler solve
/// written twice is a Kepler solve that will disagree with itself.
[[nodiscard]] double time_to_apoapsis(const OrbitalElements& elements, double mu) noexcept;

/// Time to the next periapsis [s], in `[0, period)`.
[[nodiscard]] double time_to_periapsis(const OrbitalElements& elements, double mu) noexcept;

}  // namespace rocketlab::core
