// A 2D map camera.
//
// The view is a plane through the root frame. For now the plane normal is the
// ecliptic pole, so what you see is the projection onto the ecliptic — which
// is the useful default for anything in the inner solar system, where most
// motion really is near that plane. `yaw` rotates the view within the plane,
// so a polar orbit can be laid out horizontally.
//
// The distinction that matters: the camera's *centre* tracks the selected
// object, the camera itself does not sit on it. That keeps the object's motion
// visible against the bodies around it, which is the whole point of the map.

#pragma once

#include <cstdint>

#include "rocketlab/proto/snapshot.hpp"

namespace rocketlab::render {

/// Screen position in pixels. Fractional on purpose: sub-pixel precision is
/// what keeps a slowly drifting object from jittering between two pixels, and
/// a vector backend (SVG) can use the extra bits directly.
struct ScreenPoint {
  double x{0.0};
  double y{0.0};
};

/// Metres per pixel. Below this a view is meaningless, above it the inner
/// solar system no longer fits on a terminal.
inline constexpr double kMinMetresPerPixel = 0.1;
inline constexpr double kMaxMetresPerPixel = 4.0e12;

/// The distance from the Sun to Neptune, used as the widest useful framing.
struct Camera2D {
  int width{80};
  int height{24};

  /// Centre of the view, in the root frame, projected onto the view plane.
  double center_x{0.0};
  double center_y{0.0};

  double metres_per_pixel{1.0e3};

  /// Rotation of the view within the plane, radians. Zero puts root +X to the
  /// right and +Y up.
  double yaw{0.0};

  /// When true, a client re-centres on `target` before every frame. This is
  /// the "camera centre follows the launched object" behaviour; turning it off
  /// gives a free-floating map you can pan around.
  bool following{true};
  std::uint64_t target{0};

  /// Projects a root-frame position into screen pixels. Depth is dropped.
  [[nodiscard]] ScreenPoint project(const proto::Vec3d& root) const noexcept;

  /// Converts a length in metres to a length in pixels.
  [[nodiscard]] double to_pixels(double metres) const noexcept;

  /// Screen pixels back to a root-frame position on the view plane. The
  /// component along the plane normal comes back as zero.
  [[nodiscard]] proto::Vec3d unproject(double x, double y) const noexcept;

  /// Re-centres on a root-frame position. Called every frame while `following`.
  void center_on(const proto::Vec3d& root) noexcept;

  /// Multiplies the scale. `factor` below one zooms in. Clamped to
  /// [kMinMetresPerPixel, kMaxMetresPerPixel].
  void zoom_by(double factor) noexcept;

  /// Frames a sphere of `radius` metres so it occupies `fraction` of the
  /// shorter screen axis, centred on `root`.
  void frame(const proto::Vec3d& root, double radius, double fraction = 0.25) noexcept;
};

}  // namespace rocketlab::render
