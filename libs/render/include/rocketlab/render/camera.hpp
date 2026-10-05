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

/// Metres per output unit. Below this a view is meaningless, above it the
/// inner solar system no longer fits on a terminal.
inline constexpr double kMinMetresPerPixel = 0.1;
inline constexpr double kMaxMetresPerPixel = 4.0e12;

/// A camera resolved into the three axes a scene builder projects onto.
///
/// `Camera2D` and `Camera3D` both produce one, which is what lets a single
/// scene builder serve the flat map and the orbitable view without knowing
/// which it was handed. The axes are unit vectors in the root frame, and
/// `forward` points from the camera toward `center`, so a point's `depth` grows
/// as it moves away from the viewer.
///
/// It is a plain value on purpose: a builder takes it by const reference, so
/// resolving a camera costs three normalisations per frame and nothing else.
struct View {
  proto::Vec3d center{};
  proto::Vec3d right{1.0, 0.0, 0.0};
  proto::Vec3d up{0.0, 1.0, 0.0};
  proto::Vec3d forward{0.0, 0.0, -1.0};

  double metres_per_pixel{1.0e3};
  int width{80};
  int height{24};
  /// Horizontal extent of one vertical output unit; see `Camera2D::cell_aspect`.
  double cell_aspect{1.0};

  /// True when markers should be painted far to near. A flat map has nothing
  /// behind anything, so it keeps the order the entities are stored in and its
  /// output does not depend on a comparison that is a tie for every pair.
  bool depth_sort{false};

  /// Root-frame position to screen output units. Depth is dropped; ask for it
  /// separately with `depth`.
  [[nodiscard]] ScreenPoint project(const proto::Vec3d& root) const noexcept;

  /// Metres along `forward` from the view centre. Positive is away from the
  /// viewer, which is the order painter's algorithm wants reversed.
  [[nodiscard]] double depth(const proto::Vec3d& root) const noexcept;

  /// Converts a length in metres to a length in output units.
  [[nodiscard]] double to_pixels(double metres) const noexcept;

  /// Screen output units back to a root-frame position on the view plane. The
  /// component along `forward` comes back as zero.
  [[nodiscard]] proto::Vec3d unproject(double x, double y) const noexcept;
};

struct Camera2D {
  int width{80};
  int height{24};

  /// Horizontal extent of one vertical output unit, i.e. the aspect ratio of
  /// a cell. A character terminal cell is roughly twice as tall as it is wide,
  /// so the TUI sets this to 2.0 and the map comes out round instead of
  /// stretched; a pixel backend leaves it at 1.0. It is a property of the
  /// medium, which is why the camera carries it rather than each backend
  /// rediscovering it.
  double cell_aspect{1.0};

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

  /// The same camera as a `View`, for a scene builder. `pitch` is zero by
  /// construction: this camera looks straight down the ecliptic pole, which is
  /// `Camera3D`'s top-down view.
  [[nodiscard]] View view() const noexcept;
};

/// Eases the camera's centre onto the followed target instead of snapping to it.
///
/// Changing the selection is the one camera move a client makes that a person
/// watches happen, and a hard cut throws the geometry away: a view of one vessel
/// becomes a view of another and the distance between them, which is the thing
/// worth seeing, is exactly what the cut hides. The centre therefore approaches
/// the target exponentially rather than jumping to it.
///
/// The rate is a compromise, and it is worth saying why four per second is the
/// answer. A tracking camera must not *lag*: an object at the speed of a low
/// Earth orbit — 7.7 km/s — with a rate of 4/s trails the centre by r/rate, under
/// two kilometres, which is a thousandth of a pixel at any zoom worth looking
/// at. Slow enough to watch, fast enough that following is still following.
struct CameraEase {
  /// Approach rate, in 1/s. Zero, or negative, snaps.
  double rate{4.0};
  /// Where the centre is now, in the root frame.
  proto::Vec3d center{};
  /// True until the first step, when there is nothing to glide away from.
  bool fresh{true};

  /// Puts the centre where it belongs immediately, cancelling any glide.
  void snap(const proto::Vec3d& to) noexcept;

  /// Advances the centre toward `to` by `dt` seconds and returns where it is.
  ///
  /// Exponential, and therefore frame-rate independent: one step of `dt` lands
  /// in the same place as two steps of `dt/2`. A glide whose speed depended on
  /// how fast the client happened to be drawing would be a different glide on
  /// every machine, and unrepeatable in a test.
  [[nodiscard]] proto::Vec3d step(const proto::Vec3d& to, double dt) noexcept;
};

}  // namespace rocketlab::render
