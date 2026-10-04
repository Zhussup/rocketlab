// Turns a snapshot plus a camera into a flat list of screen-space primitives.
//
// Splitting building from drawing is what lets one scene feed several
// backends: a character canvas in the TUI, ImDrawList in the GUI, and an SVG
// dump the tests can assert on. Nothing here knows about any of them, and the
// arithmetic is pure, so a scene can be checked without a terminal.
//
// The builder never propagates anything: a predicted path arrives as already
// sampled points that the caller obtained from the simulation host.

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "rocketlab/proto/snapshot.hpp"
#include "rocketlab/render/camera.hpp"
#include "rocketlab/render/color.hpp"

namespace rocketlab::render {

enum class PrimitiveKind : std::uint8_t {
  Polyline,  // open path through `points`
  Disc,      // ellipse at points[0], radii radius_x / radius_y
  Cross,     // reticle at points[0]
  Text,      // label anchored at points[0]
};

/// One drawable. Coordinates are in output units (terminal cells or pixels)
/// and have already been clipped to the viewport, so a backend only has to
/// round and plot.
struct Primitive {
  PrimitiveKind kind{PrimitiveKind::Polyline};
  Color color{};
  /// Stroke width in output units. Sub-unit widths round to one, the thinnest
  /// a character canvas can draw.
  double width{1.0};
  double radius_x{0.0};
  double radius_y{0.0};
  std::vector<ScreenPoint> points;
  std::string text;
};

/// An ordered draw list. Order is the paint order: grid, then bodies, then
/// trajectories, then entities, then labels on top.
struct Scene {
  std::vector<Primitive> primitives;
  int width{0};
  int height{0};
  /// Metres represented by the scale bar, or zero when none was drawn.
  double scale_bar_metres{0.0};

  void clear() noexcept;
  /// Appends a primitive and returns it, so a caller can keep filling it in.
  Primitive& add(PrimitiveKind kind, Color color);
};

struct SceneOptions {
  bool show_grid{true};
  bool show_bodies{true};
  bool show_trajectory{true};
  bool show_labels{true};
  bool show_scale_bar{true};

  /// A body is drawn at least this many output units across, whatever the
  /// zoom. A planet a hundredth of a cell across would otherwise be invisible
  /// exactly when it is most useful as a landmark. The map is not to scale at
  /// the small end, and the size floor is the honest admission of that.
  double min_body_radius{1.0};
  double min_entity_radius{1.5};
  /// Gap between a marker and its label, in output units.
  double label_offset{1.5};
};

/// Composes an entity's position in the root frame from the two halves the
/// snapshot carries.
[[nodiscard]] inline proto::Vec3d root_position(const proto::EntitySnapshot& entity) noexcept {
  return proto::Vec3d{entity.parent_position.x + entity.position.x,
                      entity.parent_position.y + entity.position.y,
                      entity.parent_position.z + entity.position.z};
}

/// Re-centres a following camera on the snapshot's selected entity.
///
/// The camera's centre tracks the object while the camera itself stays away
/// from it, which is the whole reason the view is useful: you see the object's
/// motion against the bodies around it, not a stationary dot with the world
/// sliding past. Kept out of `build_scene` so that building a frame stays a
/// pure function of its inputs — a caller that wants a fixed map simply does
/// not call this, and one that wants to pan clears `following`.
///
/// A selection that is missing or unset leaves the camera where it is.
void follow_target(const proto::Snapshot& snapshot, Camera2D& camera) noexcept;

/// Fills `out` with the frame.
///
/// `trajectory` is the predicted path of `snapshot.selected` in the root
/// frame, as returned by `SimSource::query_trajectory`; pass an empty vector
/// for none. It is the caller's job to have asked for the right entity — the
/// builder has no way to predict anything itself.
void build_scene(const proto::Snapshot& snapshot, const Camera2D& camera, const SceneOptions& options,
                 const std::vector<proto::Vec3d>& trajectory, Scene& out);

/// Chooses a round grid spacing in metres such that neighbouring lines are at
/// least `min_spacing` output units apart. Exposed because the scale bar and
/// the readout want the same answer.
[[nodiscard]] double choose_grid_spacing(double metres_per_pixel, double min_spacing) noexcept;

}  // namespace rocketlab::render
