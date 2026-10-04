#pragma once

#include <cmath>

namespace rocketlab::core {

/// Physical and astronomical constants, all in SI.
///
/// Values are drawn from IAU 2015 nominal values and JPL/NASA fact sheets so
/// that a propagated orbit can be compared against published ephemerides
/// without unit gymnastics.

inline constexpr double kGravitationalConstant = 6.67430e-11;   // m^3 kg^-1 s^-2

inline constexpr double kAstronomicalUnit = 1.495978707e11;     // m

// Gravitational parameters mu = G*M [m^3 s^-2].
inline constexpr double kMuSun = 1.32712440018e20;
inline constexpr double kMuMercury = 2.2032e13;
inline constexpr double kMuVenus = 3.24859e14;
inline constexpr double kMuEarth = 3.986004418e14;
inline constexpr double kMuMoon = 4.9048695e12;
inline constexpr double kMuMars = 4.282837e13;
inline constexpr double kMuJupiter = 1.26686534e17;
inline constexpr double kMuSaturn = 3.7931187e16;

// Mean equatorial radii [m].
inline constexpr double kRadiusSun = 6.957e8;
inline constexpr double kRadiusEarth = 6.371e6;
inline constexpr double kRadiusMoon = 1.7374e6;
inline constexpr double kRadiusMars = 3.3895e6;

/// Sphere of influence radius of a body orbiting a primary:
/// r_soi = a * (m / M)^(2/5), where `a` is the body's orbital semi-major axis
/// around the primary. Working with mu = G*M keeps the ratio mass-based, so
/// the gravitational constant cancels and never has to be known.
[[nodiscard]] inline double sphere_of_influence(double a, double mu_body, double mu_primary) noexcept {
  return a * std::pow(mu_body / mu_primary, 0.4);
}

}  // namespace rocketlab::core
