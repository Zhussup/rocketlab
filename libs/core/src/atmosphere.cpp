#include <algorithm>
#include <cmath>

#include "rocketlab/core/body.hpp"
#include "rocketlab/core/constants.hpp"

namespace rocketlab::core {

double Atmosphere::density_at(double altitude) const noexcept {
  if (layers.empty()) {
    return 0.0;
  }
  if (altitude <= layers.front().altitude) {
    // Below the first row the body has a surface, and the row *is* that
    // surface. Extrapolating downwards would hand a vessel that has flown into
    // the ground a thinner atmosphere than one sitting on it.
    return layers.front().density;
  }
  const AtmosphereLayer& top = layers.back();
  if (altitude >= top.altitude) {
    return 0.0;
  }

  // The rows are few and ascending, so a linear scan is faster than a binary
  // search for any plausible table and has no branch misprediction to speak
  // of. If a body ever needs a hundred rows, that is the moment to revisit it.
  for (std::size_t i = 1; i < layers.size(); ++i) {
    const AtmosphereLayer& upper = layers[i];
    if (altitude > upper.altitude) {
      continue;
    }
    const AtmosphereLayer& lower = layers[i - 1];
    const double span = upper.altitude - lower.altitude;
    if (!(span > 0.0) || !(lower.density > 0.0) || !(upper.density > 0.0)) {
      return upper.density;
    }
    // Linear in log(density): between two rows this is exactly the exponential
    // whose scale height the two densities imply.
    const double t = (altitude - lower.altitude) / span;
    return lower.density * std::exp(t * std::log(upper.density / lower.density));
  }
  return 0.0;
}

Vec3 CelestialBody::surface_velocity(const Vec3& offset) const noexcept {
  if (!(rotation_period > 0.0)) {
    return Vec3{};
  }
  const double spin = kTwoPi / rotation_period;
  return cross(spin_axis, offset) * spin;
}

PlaneElements to_ecliptic(double inclination, double ascending_node, double argument_periapsis,
                          const ReferencePlane& from) noexcept {
  const double si = std::sin(inclination);
  const double ci = std::cos(inclination);
  const double sn = std::sin(ascending_node);
  const double cn = std::cos(ascending_node);
  const double sw = std::sin(argument_periapsis);
  const double cw = std::cos(argument_periapsis);

  // The orbit's normal and the direction of its periapsis, both expressed in
  // the reference plane's own coordinates.
  const Vec3 normal{si * sn, -si * cn, ci};
  const Vec3 periapsis{cw * cn - sw * ci * sn, cw * sn + sw * ci * cn, sw * si};

  // The reference plane reaches the ecliptic by being tilted about the line of
  // nodes by its own inclination, and then swung round so that line sits at its
  // ascending node: R_z(node) * R_x(inclination), applied to the vector in that
  // order. Applying the two the other way round is the mistake that looks right
  // and puts every moon in the wrong plane.
  const auto rotate = [&from](const Vec3& v) noexcept {
    const double tilted_y = v.y * std::cos(from.inclination) - v.z * std::sin(from.inclination);
    const double tilted_z = v.y * std::sin(from.inclination) + v.z * std::cos(from.inclination);
    return Vec3{v.x * std::cos(from.ascending_node) - tilted_y * std::sin(from.ascending_node),
                v.x * std::sin(from.ascending_node) + tilted_y * std::cos(from.ascending_node),
                tilted_z};
  };

  const Vec3 h = rotate(normal);
  const Vec3 e = rotate(periapsis);

  PlaneElements out;
  out.inclination = std::acos(std::clamp(h.z, -1.0, 1.0));
  // h = (sin i sin N, -sin i cos N, cos i), so the node falls straight out.
  out.ascending_node = wrap_angle(std::atan2(h.x, -h.y));

  // The ascending node of the *rotated* orbit is the ecliptic's own line of
  // nodes with it, not the image of the reference plane's — which is why it is
  // derived from `h` above and only the periapsis direction is carried across.
  const Vec3 node{-h.y, h.x, 0.0};
  const double node_length = norm(node);
  if (node_length > 0.0) {
    const Vec3 axis = node / node_length;
    out.argument_periapsis =
        wrap_angle(std::atan2(dot(cross(axis, e), h), dot(axis, e)));
  } else {
    // An equatorial orbit has no node to measure from and folds the angle into
    // the anomaly, exactly as `elements_to_rv` expects.
    out.argument_periapsis = wrap_angle(std::atan2(e.y, e.x));
  }
  return out;
}

}  // namespace rocketlab::core
