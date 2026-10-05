#pragma once

#include <cmath>

#include "rocketlab/core/vec3.hpp"

namespace rocketlab::core {

/// Physical and astronomical constants, all in SI.
///
/// Values are drawn from IAU 2015 nominal values and JPL/NASA fact sheets so
/// that a propagated orbit can be compared against published ephemerides
/// without unit gymnastics.

inline constexpr double kGravitationalConstant = 6.67430e-11;   // m^3 kg^-1 s^-2

inline constexpr double kTwoPi = 6.283185307179586;             // rad

/// Standard gravity, the constant that converts a specific impulse in seconds
/// into an exhaust velocity: v_e = Isp * g0. It is a definition, not a local
/// measurement, so it does not depend on which body the vessel is near.
inline constexpr double kStandardGravity = 9.80665;              // m/s^2

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
inline constexpr double kMuUranus = 5.793939e15;
inline constexpr double kMuNeptune = 6.836529e15;

// The moons that are big enough to be worth navigating to rather than merely
// worth naming. Small irregular satellites are not modelled: nothing flies to
// a rock whose sphere of influence is smaller than its own orbit.
inline constexpr double kMuPhobos = 7.087e5;
inline constexpr double kMuDeimos = 9.615e4;
inline constexpr double kMuIo = 5.959916e12;
inline constexpr double kMuEuropa = 3.202739e12;
inline constexpr double kMuGanymede = 9.887834e12;
inline constexpr double kMuCallisto = 7.179289e12;
inline constexpr double kMuTitan = 8.978138e12;
inline constexpr double kMuTriton = 1.427e12;

// Mean equatorial radii [m].
inline constexpr double kRadiusSun = 6.957e8;
inline constexpr double kRadiusEarth = 6.371e6;
inline constexpr double kRadiusMoon = 1.7374e6;
inline constexpr double kRadiusMars = 3.3895e6;
inline constexpr double kRadiusPhobos = 11.267e3;
inline constexpr double kRadiusDeimos = 6.2e3;
inline constexpr double kRadiusIo = 1.8216e6;
inline constexpr double kRadiusEuropa = 1.5608e6;
inline constexpr double kRadiusGanymede = 2.6341e6;
inline constexpr double kRadiusCallisto = 2.4103e6;
inline constexpr double kRadiusTitan = 2.57473e6;
inline constexpr double kRadiusTriton = 1.3534e6;

/// Obliquity of the J2000 ecliptic to the J2000 equator [rad]. Needed only
/// where a body's pole is published in equatorial coordinates and the orbit
/// that has to be built from it lives in the ecliptic.
inline constexpr double kEclipticObliquity = 0.4090928042223289;  // 23.4392911°

/// Turns a pole published as right ascension and declination in the J2000
/// equatorial frame into the direction it points in the ecliptic frame, which
/// is the frame every orbit in this catalogue is expressed in.
[[nodiscard]] inline Vec3 pole_to_ecliptic(double right_ascension_deg,
                                           double declination_deg) noexcept {
  constexpr double kDeg = 0.017453292519943295;
  const double ra = right_ascension_deg * kDeg;
  const double dec = declination_deg * kDeg;
  const double x = std::cos(dec) * std::cos(ra);
  const double y = std::cos(dec) * std::sin(ra);
  const double z = std::sin(dec);
  const double c = std::cos(kEclipticObliquity);
  const double s = std::sin(kEclipticObliquity);
  return Vec3{x, y * c + z * s, -y * s + z * c};
}

/// Sphere of influence radius of a body orbiting a primary:
/// r_soi = a * (m / M)^(2/5), where `a` is the body's orbital semi-major axis
/// around the primary. Working with mu = G*M keeps the ratio mass-based, so
/// the gravitational constant cancels and never has to be known.
[[nodiscard]] inline double sphere_of_influence(double a, double mu_body, double mu_primary) noexcept {
  return a * std::pow(mu_body / mu_primary, 0.4);
}

}  // namespace rocketlab::core
