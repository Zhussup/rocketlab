#include "rocketlab/render/camera3d.hpp"

#include <algorithm>
#include <cmath>

namespace rocketlab::render {

namespace {

inline constexpr double kTwoPi = 6.283185307179586476925286766559;

/// Rotates a root-frame point by the view's two angles and returns its
/// components along the three axes. One place for the trig, so the basis and
/// the projection cannot disagree about the convention.
struct Basis {
  double cy;
  double sy;
  double cp;
  double sp;
};

[[nodiscard]] Basis basis_of(const Camera3D& camera) noexcept {
  return Basis{std::cos(camera.yaw), std::sin(camera.yaw), std::cos(camera.pitch),
               std::sin(camera.pitch)};
}

}  // namespace

proto::Vec3d Camera3D::right_axis() const noexcept {
  // Built in the reference plane and never tilted, which is the whole reason
  // there is no degenerate orientation: the other two axes are derived from it,
  // so they can only fail together, and they cannot.
  const Basis b = basis_of(*this);
  return proto::Vec3d{b.cy, b.sy, 0.0};
}

proto::Vec3d Camera3D::forward_axis() const noexcept {
  // The ecliptic pole rotated out of the plane by `pitch`, about the right axis.
  const Basis b = basis_of(*this);
  return proto::Vec3d{-b.sy * b.sp, b.cy * b.sp, -b.cp};
}

proto::Vec3d Camera3D::up_axis() const noexcept {
  const Basis b = basis_of(*this);
  return proto::Vec3d{-b.sy * b.cp, b.cy * b.cp, b.sp};
}

View Camera3D::view() const noexcept {
  View out;
  out.center = center;
  out.right = right_axis();
  out.up = up_axis();
  out.forward = forward_axis();
  out.metres_per_pixel = metres_per_pixel;
  out.width = width;
  out.height = height;
  out.cell_aspect = cell_aspect;
  // Only the tilted view has anything behind anything. A top-down camera keeps
  // the storage order, so its picture is identical to the 2D map's.
  out.depth_sort = !is_top_down();
  return out;
}

ScreenPoint Camera3D::project(const proto::Vec3d& root) const noexcept {
  const View resolved = view();
  return resolved.project(root);
}

double Camera3D::to_pixels(double metres) const noexcept { return metres / metres_per_pixel; }

proto::Vec3d Camera3D::unproject(double x, double y) const noexcept {
  const View resolved = view();
  return resolved.unproject(x, y);
}

void Camera3D::center_on(const proto::Vec3d& root) noexcept { center = root; }

void Camera3D::zoom_by(double factor) noexcept {
  if (!(factor > 0.0) || !std::isfinite(factor)) {
    return;
  }
  metres_per_pixel = std::clamp(metres_per_pixel * factor, kMinMetresPerPixel, kMaxMetresPerPixel);
}

void Camera3D::orbit(double d_yaw, double d_pitch) noexcept {
  if (!std::isfinite(d_yaw) || !std::isfinite(d_pitch)) {
    return;
  }
  // Wrapped into [0, 2pi), which is the rule angles are kept under everywhere
  // else in the project. `fmod` alone would let a drag to the left leave the
  // yaw negative, and two cameras that are the same rotation would then not
  // compare equal.
  yaw = std::fmod(yaw + d_yaw, kTwoPi);
  if (yaw < 0.0) {
    yaw += kTwoPi;
  }
  pitch = std::clamp(pitch + d_pitch, -kMaxPitch, kMaxPitch);
}

void Camera3D::frame(const proto::Vec3d& root, double radius, double fraction) noexcept {
  center_on(root);
  if (!(radius > 0.0) || !(fraction > 0.0)) {
    return;
  }
  // The shorter screen axis decides, or a wide window would crop the object.
  const int shorter = std::max(1, std::min(width, height));
  const double wanted = radius / (fraction * static_cast<double>(shorter));
  metres_per_pixel = std::clamp(wanted, kMinMetresPerPixel, kMaxMetresPerPixel);
}

bool Camera3D::is_top_down(double tolerance) const noexcept {
  return std::abs(std::sin(pitch)) <= tolerance;
}

void follow_target(const proto::Snapshot& snapshot, Camera3D& camera) noexcept {
  if (!camera.following || camera.target == 0) {
    return;
  }
  for (std::uint32_t i = 0; i < snapshot.entity_count && i < proto::kMaxEntities; ++i) {
    if (snapshot.entities[i].id == camera.target) {
      const proto::EntitySnapshot& entity = snapshot.entities[i];
      camera.center_on(proto::Vec3d{entity.parent_position.x + entity.position.x,
                                    entity.parent_position.y + entity.position.y,
                                    entity.parent_position.z + entity.position.z});
      return;
    }
  }
}

}  // namespace rocketlab::render
