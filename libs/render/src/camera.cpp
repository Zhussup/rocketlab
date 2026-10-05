#include "rocketlab/render/camera.hpp"

#include <algorithm>
#include <cmath>

namespace rocketlab::render {

ScreenPoint View::project(const proto::Vec3d& root) const noexcept {
  const proto::Vec3d d{root.x - center.x, root.y - center.y, root.z - center.z};
  const double rx = d.x * right.x + d.y * right.y + d.z * right.z;
  const double ry = d.x * up.x + d.y * up.y + d.z * up.z;

  // Screen coordinates are cell indices, so a viewport of `width` columns spans
  // [0, width - 1] and its centre is half a cell inside the naive midpoint.
  // Getting this wrong puts the tracked object half a cell off centre and, at
  // high zoom, makes the marker and the reticle disagree about where it is.
  const double half_w = 0.5 * static_cast<double>(width - 1);
  const double half_h = 0.5 * static_cast<double>(height - 1);
  const double aspect = cell_aspect > 0.0 ? cell_aspect : 1.0;

  // Screen Y grows downward, the view's up axis grows up, so the second term is
  // subtracted rather than added. The x term carries the aspect because
  // `metres_per_pixel` is defined vertically.
  return ScreenPoint{half_w + aspect * rx / metres_per_pixel, half_h - ry / metres_per_pixel};
}

double View::depth(const proto::Vec3d& root) const noexcept {
  return (root.x - center.x) * forward.x + (root.y - center.y) * forward.y +
         (root.z - center.z) * forward.z;
}

double View::to_pixels(double metres) const noexcept {
  return metres / metres_per_pixel;
}

proto::Vec3d View::unproject(double x, double y) const noexcept {
  const double aspect = cell_aspect > 0.0 ? cell_aspect : 1.0;
  const double rx = (x - 0.5 * static_cast<double>(width - 1)) * metres_per_pixel / aspect;
  const double ry = (0.5 * static_cast<double>(height - 1) - y) * metres_per_pixel;
  return proto::Vec3d{center.x + rx * right.x + ry * up.x,
                      center.y + rx * right.y + ry * up.y,
                      center.z + rx * right.z + ry * up.z};
}

View Camera2D::view() const noexcept {
  View out;
  out.center = proto::Vec3d{center_x, center_y, 0.0};
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  // The plane's own axes. `forward` is the ecliptic pole, which is what makes
  // this the top-down view of a Camera3D rather than a special case.
  out.right = proto::Vec3d{c, s, 0.0};
  out.up = proto::Vec3d{-s, c, 0.0};
  out.forward = proto::Vec3d{0.0, 0.0, -1.0};
  out.metres_per_pixel = metres_per_pixel;
  out.width = width;
  out.height = height;
  out.cell_aspect = cell_aspect;
  out.depth_sort = false;
  return out;
}

ScreenPoint Camera2D::project(const proto::Vec3d& root) const noexcept {
  return view().project(root);
}

double Camera2D::to_pixels(double metres) const noexcept {
  return metres / metres_per_pixel;
}

proto::Vec3d Camera2D::unproject(double x, double y) const noexcept {
  const double aspect = cell_aspect > 0.0 ? cell_aspect : 1.0;
  const double rx =
      (x - 0.5 * static_cast<double>(width - 1)) * metres_per_pixel / aspect;
  const double ry =
      (0.5 * static_cast<double>(height - 1) - y) * metres_per_pixel;

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

void CameraEase::snap(const proto::Vec3d& to) noexcept {
  center = to;
  fresh = false;
}

proto::Vec3d CameraEase::step(const proto::Vec3d& to, double dt) noexcept {
  if (fresh || !(rate > 0.0) || !(dt > 0.0) || !std::isfinite(dt)) {
    center = to;
    fresh = false;
    return center;
  }

  // 1 - exp(-rate*dt): the fraction of the remaining gap to close this step.
  // Written as an exponential rather than as `gap * rate * dt` so that the
  // result does not depend on the step size — a linear form overshoots as soon
  // as `rate * dt` approaches one, which a client stalled for a quarter of a
  // second would reach.
  const double fraction = 1.0 - std::exp(-rate * dt);
  center.x += (to.x - center.x) * fraction;
  center.y += (to.y - center.y) * fraction;
  center.z += (to.z - center.z) * fraction;
  return center;
}

}  // namespace rocketlab::render
