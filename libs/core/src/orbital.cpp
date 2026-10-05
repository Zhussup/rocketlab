#include "rocketlab/core/orbital.hpp"

#include <algorithm>
#include <cmath>

namespace rocketlab::core {

double wrap_angle(double radians) noexcept {
  constexpr double kFullTurn = 6.283185307179586476925286766559;
  double wrapped = std::fmod(radians, kFullTurn);
  if (wrapped < 0.0) {
    wrapped += kFullTurn;
  }
  return wrapped;
}

namespace {

constexpr double kTwoPi = 6.283185307179586476925286766559;
constexpr double kPi = 3.141592653589793238462643383279;

/// Beyond this |H| the hyperbolic functions overflow a double, and the
/// corresponding orbit is far outside anything the simulator cares about.
constexpr double kMaxHyperbolicAnomaly = 700.0;

/// Solves Kepler's equation M = E - e sin E for the eccentric anomaly.
///
/// Newton's method alone is not safe here: for eccentricities near 1 the
/// derivative 1 - e cos E collapses towards zero and an unguarded step shoots
/// off to a wrong revolution. But the root is always confined to
/// [M - e, M + e] intersected with [-pi, pi] — because |E - M| = e|sin E| <= e
/// — and E - e·sin E - M increases strictly across it. So we keep that bracket
/// and bisect whenever a Newton step would leave it, which converges globally
/// while staying quadratic in the common case.
[[nodiscard]] double solve_kepler_elliptic(double mean_anomaly, double e) noexcept {
  const double m = std::remainder(mean_anomaly, kTwoPi);

  double lo = std::max(-kPi, m - e);
  double hi = std::min(kPi, m + e);
  if (!(lo < hi)) {
    return m;  // e == 0: the root is M itself.
  }

  double E = std::clamp(m + e * std::sin(m), lo, hi);
  for (int iter = 0; iter < 100; ++iter) {
    const double f = E - e * std::sin(E) - m;
    if (f > 0.0) {
      hi = E;
    } else {
      lo = E;
    }
    double next = E - f / (1.0 - e * std::cos(E));
    if (!(next > lo && next < hi)) {
      next = 0.5 * (lo + hi);
    }
    if (std::fabs(next - E) <= 1e-15) {
      return next;
    }
    E = next;
  }
  return E;
}

/// Solves the hyperbolic Kepler equation M = e sinh H - H. Both sides are odd
/// in their argument, so we solve for |M| and restore the sign.
[[nodiscard]] double solve_kepler_hyperbolic(double mean_anomaly, double e) noexcept {
  if (mean_anomaly == 0.0) {
    return 0.0;
  }
  const double sign = mean_anomaly < 0.0 ? -1.0 : 1.0;
  const double m = std::fabs(mean_anomaly);

  // Inverting sinh H = (M + H)/e gives a starter that always sits just below
  // the root, so Newton climbs to it monotonically.
  double H = std::asinh(m / e);
  for (int iter = 0; iter < 100; ++iter) {
    const double f = e * std::sinh(H) - H - m;
    double next = H - f / (e * std::cosh(H) - 1.0);
    if (!std::isfinite(next) || next < 0.0 || next > kMaxHyperbolicAnomaly) {
      next = 0.5 * H;
    }
    if (std::fabs(next - H) <= 1e-15 * (1.0 + H)) {
      H = next;
      break;
    }
    H = next;
  }
  return sign * H;
}

/// Barker's parameter B = D + D^3/3 with D = tan(nu/2). It advances linearly
/// in time along a parabola, playing the role the mean anomaly plays on an
/// ellipse.
[[nodiscard]] double barker_parameter(double true_anomaly) noexcept {
  const double d = std::tan(0.5 * true_anomaly);
  return d + d * d * d / 3.0;
}

/// Inverts Barker's equation. The cubic D^3 + 3D - 3B = 0 has the single real
/// root below (its discriminant is 9B^2/4 + 1, always positive), so Cardano's
/// formula applies directly and no iteration is needed.
[[nodiscard]] double barker_inverse(double b) noexcept {
  const double root = std::sqrt(9.0 * b * b / 4.0 + 1.0);
  return std::cbrt(1.5 * b + root) + std::cbrt(1.5 * b - root);
}

/// True anomaly -> eccentric anomaly, for the elliptic branch.
[[nodiscard]] double true_to_eccentric(double nu, double e) noexcept {
  return 2.0 * std::atan2(std::sqrt(1.0 - e) * std::sin(0.5 * nu),
                         std::sqrt(1.0 + e) * std::cos(0.5 * nu));
}

[[nodiscard]] double eccentric_to_true(double E, double e) noexcept {
  return 2.0 * std::atan2(std::sqrt(1.0 + e) * std::sin(0.5 * E),
                         std::sqrt(1.0 - e) * std::cos(0.5 * E));
}

/// True anomaly -> hyperbolic anomaly, for the hyperbolic branch.
[[nodiscard]] double true_to_hyperbolic(double nu, double e) noexcept {
  return 2.0 * std::atanh(std::sqrt((e - 1.0) / (e + 1.0)) * std::tan(0.5 * nu));
}

[[nodiscard]] double hyperbolic_to_true(double H, double e) noexcept {
  return 2.0 * std::atan2(std::sqrt(e + 1.0) * std::sinh(0.5 * H),
                         std::sqrt(e - 1.0) * std::cosh(0.5 * H));
}

/// Perifocal -> inertial rotation, built from the three Euler angles. The two
/// basis vectors are the first two columns of Rz(raan)·Rx(i)·Rz(argp); the
/// third is unused because perifocal states have zero z.
struct PerifocalBasis {
  Vec3 P;
  Vec3 Q;
};

[[nodiscard]] PerifocalBasis perifocal_basis(const OrbitalElements& el) noexcept {
  const double cO = std::cos(el.raan);
  const double sO = std::sin(el.raan);
  const double ci = std::cos(el.i);
  const double si = std::sin(el.i);
  const double cw = std::cos(el.argp);
  const double sw = std::sin(el.argp);

  PerifocalBasis basis;
  basis.P = Vec3{cO * cw - sO * ci * sw, sO * cw + cO * ci * sw, si * sw};
  basis.Q = Vec3{-cO * sw - sO * ci * cw, -sO * sw + cO * ci * cw, si * cw};
  return basis;
}

}  // namespace

double OrbitalElements::semi_major_axis() const noexcept {
  const double one_minus_e_squared = 1.0 - e * e;
  if (std::fabs(one_minus_e_squared) < 1e-15) {
    return e > 1.0 ? -std::numeric_limits<double>::infinity()
                   : std::numeric_limits<double>::infinity();
  }
  return p / one_minus_e_squared;
}

double OrbitalElements::apoapsis() const noexcept {
  if (e >= 1.0) {
    return std::numeric_limits<double>::infinity();
  }
  return p / (1.0 - e);
}

double OrbitalElements::period(double mu) const noexcept {
  const double a = semi_major_axis();
  if (!(a > 0.0) || !std::isfinite(a)) {
    return std::numeric_limits<double>::infinity();
  }
  return kTwoPi * std::sqrt(a * a * a / mu);
}

ConicKind classify(const OrbitalElements& elements) noexcept {
  if (elements.e < 1.0 - kParabolicTolerance) {
    return ConicKind::Elliptic;
  }
  if (elements.e > 1.0 + kParabolicTolerance) {
    return ConicKind::Hyperbolic;
  }
  return ConicKind::Parabolic;
}

OrbitalElements rv_to_elements(const StateVector& state, double mu) noexcept {
  OrbitalElements el;

  const double r = norm(state.r);
  const double v = norm(state.v);
  const Vec3 h_vec = cross(state.r, state.v);
  const double h = norm(h_vec);

  // A radial trajectory sweeps no area, so no orbital plane exists and no set
  // of elements can describe it. Flag it rather than emit NaNs.
  //
  // The test is relative — h is compared against r*v, which makes it the sine
  // of the angle between the radius and the velocity — because exact zero is
  // not the only value that has to be caught. A trajectory that is radial to
  // within a part in 1e12 is radial for every purpose a simulation has, and
  // the failure it produces is worse than being wrong: `propagate` classifies
  // it as parabolic, and the parabolic branch divides by p = h^2/mu, so a
  // state that missed exact radiality by a rounding error propagates into
  // infinities and takes the snapshot with it.
  if (!(h > kNodeTolerance * r * v) || !std::isfinite(h) || r == 0.0) {
    el.degenerate = true;
    return el;
  }

  const Vec3 n_vec{-h_vec.y, h_vec.x, 0.0};  // node line, k_hat x h
  const double n = norm(n_vec);

  const Vec3 e_vec =
      ((v * v - mu / r) * state.r - dot(state.r, state.v) * state.v) / mu;
  const double e = norm(e_vec);

  el.p = h * h / mu;
  el.e = e;
  el.i = std::acos(std::clamp(h_vec.z / h, -1.0, 1.0));

  const bool equatorial = n <= kNodeTolerance * h;
  const bool circular = e <= kCircularTolerance;
  // For a retrograde orbit the longitude runs the other way; the anomaly
  // formulas below are written for the prograde sense and get mirrored here.
  const double sense = h_vec.z < 0.0 ? -1.0 : 1.0;

  el.raan = equatorial ? 0.0 : wrap_angle(std::atan2(h_vec.x, -h_vec.y));

  if (circular) {
    // argp is undefined; fold it into nu so that nu becomes the argument of
    // latitude (equatorial case: the true longitude).
    el.argp = 0.0;
    el.nu = wrap_angle(equatorial
                            ? std::atan2(sense * state.r.y, state.r.x)
                            : std::atan2(dot(cross(n_vec, state.r), h_vec) / h,
                                         dot(n_vec, state.r)));
  } else {
    el.argp = wrap_angle(
        equatorial ? std::atan2(sense * e_vec.y, e_vec.x)
                   : std::atan2(dot(cross(n_vec, e_vec), h_vec) / h, dot(n_vec, e_vec)));
    // sin(nu) = (r . v) sqrt(p/mu) / (e r) and cos(nu) = (e_vec . r) / (e r)
    // share the positive factor 1/(e r), so atan2 of the numerators is exact
    // and, unlike acos, keeps full precision near nu = 0 and nu = pi.
    el.nu = wrap_angle(
        std::atan2(dot(state.r, state.v) * std::sqrt(el.p / mu), dot(e_vec, state.r)));
  }
  return el;
}

StateVector elements_to_rv(const OrbitalElements& elements, double mu) noexcept {
  const double cos_nu = std::cos(elements.nu);
  const double sin_nu = std::sin(elements.nu);

  const double radius = elements.p / (1.0 + elements.e * cos_nu);
  const double speed_scale = std::sqrt(mu / elements.p);

  const Vec3 r_peri{radius * cos_nu, radius * sin_nu, 0.0};
  const Vec3 v_peri{-speed_scale * sin_nu, speed_scale * (elements.e + cos_nu), 0.0};

  const PerifocalBasis basis = perifocal_basis(elements);
  return StateVector{r_peri.x * basis.P + r_peri.y * basis.Q,
                      v_peri.x * basis.P + v_peri.y * basis.Q};
}

StateVector propagate(const StateVector& state, double mu, double dt) noexcept {
  if (dt == 0.0) {
    return state;
  }

  OrbitalElements el = rv_to_elements(state, mu);
  if (el.degenerate) {
    return state;
  }

  switch (classify(el)) {
    case ConicKind::Elliptic: {
      const double a = el.semi_major_axis();
      const double n = std::sqrt(mu / (a * a * a));
      const double E0 = true_to_eccentric(el.nu, el.e);
      const double mean = E0 - el.e * std::sin(E0) + n * dt;
      el.nu = eccentric_to_true(solve_kepler_elliptic(mean, el.e), el.e);
      break;
    }
    case ConicKind::Hyperbolic: {
      const double abs_a = -el.semi_major_axis();  // a is negative here
      const double n = std::sqrt(mu / (abs_a * abs_a * abs_a));
      const double H0 = true_to_hyperbolic(el.nu, el.e);
      const double mean = el.e * std::sinh(H0) - H0 + n * dt;
      el.nu = hyperbolic_to_true(solve_kepler_hyperbolic(mean, el.e), el.e);
      break;
    }
    case ConicKind::Parabolic: {
      // e == 1 exactly, so a diverges; time enters through Barker's parameter
      // scaled by the periapsis distance, p/2.
      const double periapsis = 0.5 * el.p;
      const double rate = std::sqrt(mu / (2.0 * periapsis * periapsis * periapsis));
      const double b = barker_parameter(el.nu) + rate * dt;
      el.nu = 2.0 * std::atan(barker_inverse(b));
      break;
    }
  }
  return elements_to_rv(el, mu);
}

double specific_energy(const StateVector& state, double mu) noexcept {
  const double r = norm(state.r);
  if (r == 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  return 0.5 * norm_squared(state.v) - mu / r;
}

double mean_motion(double semi_major_axis, double mu) noexcept {
  const double a = std::fabs(semi_major_axis);
  if (a == 0.0 || !std::isfinite(a)) {
    return 0.0;
  }
  return std::sqrt(mu / (a * a * a));
}

double true_anomaly_from_mean(double mean, double eccentricity) noexcept {
  if (eccentricity < 1.0 - kParabolicTolerance) {
    return eccentric_to_true(solve_kepler_elliptic(mean, eccentricity), eccentricity);
  }
  if (eccentricity > 1.0 + kParabolicTolerance) {
    return hyperbolic_to_true(solve_kepler_hyperbolic(mean, eccentricity), eccentricity);
  }
  return 2.0 * std::atan(barker_inverse(mean));
}

StateVector state_from_mean_elements(const OrbitalElements& elements, double mu) noexcept {
  OrbitalElements resolved = elements;
  resolved.nu = true_anomaly_from_mean(mean_anomaly(elements), elements.e);
  return elements_to_rv(resolved, mu);
}

double time_since_periapsis(const OrbitalElements& elements, double mu) noexcept {
  if (elements.degenerate) {
    return 0.0;
  }
  const double n = mean_motion(elements.semi_major_axis(), mu);
  const double period = elements.period(mu);
  if (!(n > 0.0) || !(period > 0.0)) {
    return 0.0;
  }
  // `mean_anomaly` comes back in (-pi, pi], so this is in (-period/2, period/2].
  // Wrapping it into a whole period is what makes "time to apoapsis" an answer
  // about the next apoapsis rather than about the one just gone.
  double since = std::fmod(mean_anomaly(elements) / n, period);
  if (since < 0.0) {
    since += period;
  }
  return since;
}

double time_to_apoapsis(const OrbitalElements& elements, double mu) noexcept {
  const double period = elements.period(mu);
  if (elements.degenerate || elements.e >= 1.0 || !(period > 0.0)) {
    return 0.0;
  }
  const double half = 0.5 * period;
  const double since = time_since_periapsis(elements, mu);
  // Apoapsis is half a period after periapsis, so the answer is either later
  // this revolution or later than the periapsis that is still to come.
  return since <= half ? half - since : period - since + half;
}

double time_to_periapsis(const OrbitalElements& elements, double mu) noexcept {
  const double period = elements.period(mu);
  if (elements.degenerate || elements.e >= 1.0 || !(period > 0.0)) {
    return 0.0;
  }
  const double since = time_since_periapsis(elements, mu);
  return since <= 0.0 ? -since : period - since;
}

double mean_anomaly(const OrbitalElements& elements) noexcept {
  switch (classify(elements)) {
    case ConicKind::Elliptic: {
      const double E = true_to_eccentric(elements.nu, elements.e);
      return E - elements.e * std::sin(E);
    }
    case ConicKind::Hyperbolic: {
      const double H = true_to_hyperbolic(elements.nu, elements.e);
      return elements.e * std::sinh(H) - H;
    }
    case ConicKind::Parabolic:
      return barker_parameter(elements.nu);
  }
  return 0.0;
}

}  // namespace rocketlab::core
