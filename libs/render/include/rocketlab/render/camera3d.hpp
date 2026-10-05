// An orbitable 3D view of the root frame.
//
// The 2D map is not replaced by this and is not a mode beside it: it is this
// camera at `pitch == 0`, and the arithmetic below reduces to `Camera2D`'s
// exactly when the pitch is zero. That is worth the constraint it puts on the
// angle convention, because it means the flat map and the tilted view cannot
// disagree about where anything is — there is one projection, not two, and a
// switch between them moves the camera without moving the world.
//
// Angles, both in radians:
//
//   `yaw`   rotates the view within the reference plane, exactly as in 2D. It
//           has no effect only when the pitch is zero, where it is degenerate
//           for the same reason it is in 2D: the view axis is the plane normal.
//   `pitch` tilts the camera out of that plane. Zero is straight down the pole
//           — the flat map. Positive tilts toward the plane, so that at +pi/2
//           the camera looks along the ecliptic with the pole up the screen.
//
// There is no degenerate orientation anywhere in that range: the right axis is
// built in the plane and the other two follow from it, so the basis is
// orthonormal for every pitch without a special case at the poles.
//
// The camera's *centre* tracks the selected object; the camera itself sits away
// from it, as in the map. `metres_per_pixel` is an orthographic scale, so there
// is no perspective divide and no near plane to clip against, which is what
// keeps a heliocentric view with a nine-order-of-magnitude span drawable at all.

#pragma once

#include <cstdint>

#include "rocketlab/render/camera.hpp"

namespace rocketlab::render {

/// How far the pitch may be tilted. Exactly the pole-ward limit, where the
/// camera looks along the reference plane with the pole up the screen; stopping
/// here is what keeps an orbit drag from rolling the view upside down, which a
/// basis built this way would happily do past it.
inline constexpr double kMaxPitch = 1.5707963267948966;  // pi/2

struct Camera3D {
  int width{80};
  int height{24};
  double cell_aspect{1.0};

  /// Centre of the view, in the root frame. This is the point the camera looks
  /// at, and the one a following camera is moved onto.
  proto::Vec3d center{};

  double metres_per_pixel{1.0e3};

  /// See the note at the top of the file. Zero pitch is the flat map.
  double yaw{0.0};
  double pitch{0.0};

  /// When true, a client re-centres on `target` before every frame. The same
  /// behaviour as the 2D camera's, and turned off the same way: clearing
  /// `following` gives a view you can orbit and pan freely.
  bool following{true};
  std::uint64_t target{0};

  /// The right axis, which lies in the reference plane and is never parallel to
  /// the view direction, so the basis never collapses.
  [[nodiscard]] proto::Vec3d right_axis() const noexcept;
  [[nodiscard]] proto::Vec3d up_axis() const noexcept;
  /// From the camera toward `center`.
  [[nodiscard]] proto::Vec3d forward_axis() const noexcept;

  [[nodiscard]] View view() const noexcept;

  [[nodiscard]] ScreenPoint project(const proto::Vec3d& root) const noexcept;
  [[nodiscard]] double to_pixels(double metres) const noexcept;
  [[nodiscard]] proto::Vec3d unproject(double x, double y) const noexcept;

  /// Re-centres on a root-frame position. Called every frame while following.
  void center_on(const proto::Vec3d& root) noexcept;

  /// Multiplies the scale. `factor` below one zooms in.
  void zoom_by(double factor) noexcept;

  /// Rotates the camera about the view centre. `d_pitch` is clamped so the
  /// basis stays the right way up.
  void orbit(double d_yaw, double d_pitch) noexcept;

  /// Frames a sphere of `radius` metres so it occupies `fraction` of the
  /// shorter screen axis, centred on `root`.
  void frame(const proto::Vec3d& root, double radius, double fraction = 0.25) noexcept;

  /// True while the view is close enough to the flat map to be called one.
  [[nodiscard]] bool is_top_down(double tolerance = 1.0e-3) const noexcept;
};

/// Re-centres a following 3D camera on the snapshot's selected entity.
///
/// The 3D counterpart of the 2D overload in `scene.hpp`, and separate for the
/// same reason: building a frame stays a pure function of its inputs, so a
/// camera a client wants to keep still simply does not call it.
void follow_target(const proto::Snapshot& snapshot, Camera3D& camera) noexcept;

}  // namespace rocketlab::render
