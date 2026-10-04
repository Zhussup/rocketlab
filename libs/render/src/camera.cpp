#include "rocketlab/render/camera.hpp"

#include <algorithm>
#include <cmath>

namespace rocketlab::render {

ScreenPoint Camera2D::project(const proto::Vec3d& root) const noexcept {
  const double dx = root.x - center_x;
  const double dy = root.y - center_y;

  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double rx = dx * c + dy * s;
  const double ry = -dx * s + dy * c;

  const double half_w = 0.5 * static_cast<double>(width);
  const double half_h = 0.5 * static_cast<double>(height);

  // Screen Y grows downward, the root frame's Y grows up, so the second term
  // is subtracted rather than added.
  return ScreenPoint{half_w + rx / metres_per_pixel, half_h - ry / metres_per_pixel};
}

double Camera2D::to_pixels(double metres) const noexcept {
  return metres / metres_per_pixel;
}

proto::Vec3d Camera2D::unproject(double x, double y) const noexcept {
  const double rx = (x - 0.5 * static_cast<double>(width)) * metres_per_pixel;
  const double ry = (0.5 * static_cast<double>(height) - y) * metres_per_pixel;

  // Inverse rotation: the transpose, since the matrix is orthonormal.
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return proto::Vec3d{center_x + rx * c - ry * s, center_y + rx * s + ry * c, 0.0};
}

void Camera2D::center_on(const proto::Vec3d& root) noexcept {
  center_x = root.x;
  center_y = root.y;
}

void Camera2D::zoom_by(double factor) noexcept {
  if (!(factor > 0.0) || !std::isfinite(factor)) {
    return;
  }
  metres_per_pixel = std::clamp(metres_per_pixel * factor, kMinMetresPerPixel, kMaxMetresPerPixel);
}

void Camera2D::frame(const proto::Vec3d& root, double radius, double fraction) noexcept {
  center_on(root);
  if (!(radius > 0.0) || !(fraction > 0.0)) {
    return;
  }
  // The shorter axis decides, otherwise a wide terminal would crop the object
  // vertically.
  const int shorter = std::max(1, std::min(width, height));
  const double wanted = radius / (fraction * static_cast<double>(shorter));
  metres_per_pixel = std::clamp(wanted, kMinMetresPerPixel, kMaxMetresPerPixel);
}

}  // namespace rocketlab::render
