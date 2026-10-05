#include "rocketlab/render/scene.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <string_view>
#include <utility>

namespace rocketlab::render {

namespace {

/// Liang-Barsky clip of a segment against [0,max_x] x [0,max_y]. Returns false
/// when the segment misses the viewport entirely.
///
/// Clipping here rather than in each backend matters: at any interesting zoom
/// the root-frame coordinates of off-screen geometry are enormous, and a
/// backend that blindly rounded them would overflow a 16-bit canvas index or
/// spend its time filling millions of cells.
[[nodiscard]] bool clip_segment(ScreenPoint& a, ScreenPoint& b, double max_x, double max_y) noexcept {
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double p[4] = {-dx, dx, -dy, dy};
  const double q[4] = {a.x, max_x - a.x, a.y, max_y - a.y};

  double t0 = 0.0;
  double t1 = 1.0;
  for (int i = 0; i < 4; ++i) {
    if (p[i] == 0.0) {
      if (q[i] < 0.0) {
        return false;  // parallel to this edge and outside it
      }
      continue;
    }
    const double r = q[i] / p[i];
    if (p[i] < 0.0) {
      if (r > t1) {
        return false;
      }
      t0 = std::max(t0, r);
    } else {
      if (r < t0) {
        return false;
      }
      t1 = std::min(t1, r);
    }
  }

  const ScreenPoint start = a;
  a = ScreenPoint{start.x + t0 * dx, start.y + t0 * dy};
  b = ScreenPoint{start.x + t1 * dx, start.y + t1 * dy};
  return true;
}

void append_path(Scene& out, Color color, double width, const std::vector<ScreenPoint>& path,
                 double max_x, double max_y) {
  if (path.size() < 2) {
    return;
  }

  // Fast path: if every vertex is inside, the whole path is one primitive and
  // the segments do not have to be split.
  bool all_inside = true;
  for (const ScreenPoint& point : path) {
    if (!(point.x >= 0.0 && point.x <= max_x && point.y >= 0.0 && point.y <= max_y)) {
      all_inside = false;
      break;
    }
  }
  if (all_inside) {
    Primitive& primitive = out.add(PrimitiveKind::Polyline, color);
    primitive.width = width;
    primitive.points = path;
    return;
  }

  // Otherwise each segment is clipped and emitted on its own. Splitting loses
  // the joins between segments, which at a one-unit stroke is not visible, and
  // it keeps every coordinate that reaches a backend inside the viewport.
  for (std::size_t i = 1; i < path.size(); ++i) {
    ScreenPoint a = path[i - 1];
    ScreenPoint b = path[i];
    if (!std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(b.x) || !std::isfinite(b.y)) {
      continue;
    }
    if (!clip_segment(a, b, max_x, max_y)) {
      continue;
    }
    Primitive& primitive = out.add(PrimitiveKind::Polyline, color);
    primitive.width = width;
    primitive.points = {a, b};
  }
}

[[nodiscard]] bool on_screen(const ScreenPoint& point, double max_x, double max_y,
                             double margin) noexcept {
  return point.x >= -margin && point.x <= max_x + margin && point.y >= -margin &&
         point.y <= max_y + margin;
}

/// Formats a length the way a scale bar wants it: a short round number with a
/// readable unit, never scientific notation.
[[nodiscard]] std::string format_length(double metres) {
  if (metres >= 1.0e9) {
    return std::format("{:g} Gm", metres / 1.0e9);
  }
  if (metres >= 1.0e6) {
    return std::format("{:g} Mm", metres / 1.0e6);
  }
  if (metres >= 1.0e3) {
    return std::format("{:g} km", metres / 1.0e3);
  }
  return std::format("{:g} m", metres);
}

struct PendingLabel {
  ScreenPoint at;  // already offset clear of its marker
  std::string text;
  Color color;
  /// Lower wins when two labels want the same space. The object being flown
  /// keeps its name in preference to the scenery it is flying past.
  int priority{0};
};

/// A label already committed to the screen, as the row it sits on and the
/// half-open column span it covers.
struct PlacedLabel {
  int row;
  int x0;
  int x1;
};

/// One body or entity's drawables, held back until the paint order is known.
///
/// A flat map emits these in snapshot order, and the picture is the one it has
/// always been. A tilted view has to sort them, and sorting is the only
/// difference between the two: everything that decides *what* is drawn — the
/// projection, the size floor, the colour, the reticle — happens once, here,
/// before either order is chosen.
struct Marker {
  Primitive disc;
  Primitive reticle;
  bool has_reticle{false};
  double depth{0.0};
};

void emit_marker(Scene& out, const Marker& marker) {
  out.primitives.push_back(marker.disc);
  if (marker.has_reticle) {
    out.primitives.push_back(marker.reticle);
  }
}

[[nodiscard]] double dot(const proto::Vec3d& a, const proto::Vec3d& b) noexcept {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

}  // namespace

void Scene::clear() noexcept {
  primitives.clear();
  width = 0;
  height = 0;
  scale_bar_metres = 0.0;
}

Primitive& Scene::add(PrimitiveKind kind, Color color) {
  primitives.push_back(Primitive{});
  Primitive& primitive = primitives.back();
  primitive.kind = kind;
  primitive.color = color;
  return primitive;
}

void follow_target(const proto::Snapshot& snapshot, Camera2D& camera) noexcept {
  if (!camera.following || camera.target == 0) {
    return;
  }
  for (std::uint32_t i = 0; i < snapshot.entity_count && i < proto::kMaxEntities; ++i) {
    if (snapshot.entities[i].id == camera.target) {
      camera.center_on(root_position(snapshot.entities[i]));
      return;
    }
  }
}

double choose_grid_spacing(double metres_per_pixel, double min_spacing) noexcept {
  if (!(metres_per_pixel > 0.0) || !(min_spacing > 0.0)) {
    return 1.0;
  }
  const double raw = metres_per_pixel * min_spacing;
  const double exponent = std::floor(std::log10(raw));
  const double base = std::pow(10.0, exponent);
  const double normalised = raw / base;  // in [1, 10)

  // A 1-2-5 sequence, the same one map makers use: consecutive steps are never
  // more than a factor of two and a half apart, so the grid never looks bare.
  double step = 10.0;
  if (normalised <= 1.0) {
    step = 1.0;
  } else if (normalised <= 2.0) {
    step = 2.0;
  } else if (normalised <= 5.0) {
    step = 5.0;
  }
  return step * base;
}

void build_scene(const proto::Snapshot& snapshot, const Camera2D& camera,
                 const SceneOptions& options, const std::vector<proto::Vec3d>& trajectory,
                 Scene& out) {
  build_scene(snapshot, camera.view(), options, trajectory, out);
}

void build_scene(const proto::Snapshot& snapshot, const View& view, const SceneOptions& options,
                 const std::vector<proto::Vec3d>& trajectory, Scene& out) {
  out.clear();
  out.width = view.width;
  out.height = view.height;

  const double max_x = static_cast<double>(std::max(0, view.width - 1));
  const double max_y = static_cast<double>(std::max(0, view.height - 1));
  const double aspect = view.cell_aspect > 0.0 ? view.cell_aspect : 1.0;
  const double mpp = view.metres_per_pixel;

  // The view-plane components of the centre. These are what the grid is laid
  // out against, because a grid line is a line of constant position *in the
  // view plane*: measured against root X and Y it would slide sideways as the
  // view turned, which is the one thing a reference grid must not do. With no
  // yaw the two agree exactly — `right` and `up` are then the root axes — so
  // the flat map's picture is unchanged.
  const double origin_x = dot(view.center, view.right);
  const double origin_y = dot(view.center, view.up);

  std::vector<PendingLabel> labels;
  std::vector<Marker> bodies;
  std::vector<Marker> entities;

  // --- grid -----------------------------------------------------------------
  //
  // Drawn first so everything else paints over it. Lines are placed on round
  // root-frame coordinates rather than on screen multiples, so they stay put
  // while the camera pans.
  if (options.show_grid && mpp > 0.0) {
    const double spacing = choose_grid_spacing(mpp, 8.0 * aspect);
    const double half_span_x = 0.5 * max_x * mpp / aspect;
    const double half_span_y = 0.5 * max_y * mpp;

    const double x_lo = origin_x - half_span_x;
    const double x_hi = origin_x + half_span_x;
    const double y_lo = origin_y - half_span_y;
    const double y_hi = origin_y + half_span_y;

    const auto first_multiple = [](double lo, double step) {
      return std::ceil(lo / step) * step;
    };
    const double first_x = first_multiple(x_lo, spacing);
    const double first_y = first_multiple(y_lo, spacing);

    // A guard rather than a promise: if the arithmetic ever produces a spacing
    // that would draw thousands of lines, drawing none is better than hanging.
    constexpr int kMaxGridLines = 512;
    const int columns = static_cast<int>((x_hi - first_x) / spacing) + 1;
    const int rows = static_cast<int>((y_hi - first_y) / spacing) + 1;

    if (columns > 0 && columns < kMaxGridLines) {
      for (int i = 0; i < columns; ++i) {
        const double x = first_x + i * spacing;
        // Built straight in screen space: a vertical line spans the whole
        // height, so projecting it twice would be wasted work.
        const double sx = 0.5 * max_x + aspect * (x - origin_x) / mpp;
        Primitive& line = out.add(PrimitiveKind::Polyline,
                                  std::abs(x) < spacing * 0.5 ? colors::kAxis : colors::kGrid);
        line.width = 1.0;
        line.points = {ScreenPoint{sx, 0.0}, ScreenPoint{sx, max_y}};
      }
    }
    if (rows > 0 && rows < kMaxGridLines) {
      for (int i = 0; i < rows; ++i) {
        const double y = first_y + i * spacing;
        const double sy = 0.5 * max_y - (y - origin_y) / mpp;
        Primitive& line = out.add(PrimitiveKind::Polyline,
                                  std::abs(y) < spacing * 0.5 ? colors::kAxis : colors::kGrid);
        line.width = 1.0;
        line.points = {ScreenPoint{0.0, sy}, ScreenPoint{max_x, sy}};
      }
    }
  }

  // --- bodies ---------------------------------------------------------------
  if (options.show_bodies) {
    for (std::uint32_t i = 0; i < snapshot.body_count && i < proto::kMaxBodies; ++i) {
      const proto::BodySnapshot& body = snapshot.bodies[i];
      const ScreenPoint at = view.project(body.position);
      if (!on_screen(at, max_x, max_y, 0.0)) {
        continue;
      }

      const double radius = std::max(view.to_pixels(body.radius), options.min_body_radius);
      Marker marker;
      marker.disc.kind = PrimitiveKind::Disc;
      marker.disc.color = colors::kBody;
      marker.disc.points = {at};
      marker.disc.radius_y = radius;
      marker.disc.radius_x = radius * aspect;
      marker.depth = view.depth(body.position);
      bodies.push_back(std::move(marker));

      if (options.show_labels) {
        labels.push_back(PendingLabel{
            ScreenPoint{at.x + radius * aspect + options.label_offset, at.y},
            std::string(body.name.view()), colors::kBodyDim, 2});
      }
    }
  }

  // --- predicted path -------------------------------------------------------
  //
  // Anchored on the selected entity's parent, which is what makes an orbit
  // look like an orbit. The parent position comes straight out of the snapshot
  // rather than being asked for separately, so a path and the object it
  // belongs to can never be drawn against two different anchors.
  const bool depth_sort = view.depth_sort;

  if (!depth_sort) {
    for (const Marker& marker : bodies) {
      emit_marker(out, marker);
    }
  }

  if (options.show_trajectory && !trajectory.empty()) {
    proto::Vec3d anchor;
    for (std::uint32_t i = 0; i < snapshot.entity_count && i < proto::kMaxEntities; ++i) {
      if (snapshot.entities[i].id == snapshot.selected) {
        anchor = snapshot.entities[i].parent_position;
        break;
      }
    }

    std::vector<ScreenPoint> path;
    path.reserve(trajectory.size());
    for (const proto::Vec3d& point : trajectory) {
      path.push_back(view.project(
          proto::Vec3d{anchor.x + point.x, anchor.y + point.y, anchor.z + point.z}));
    }
    append_path(out, colors::kTrajectory, 1.0, path, max_x, max_y);
  }

  // --- entities -------------------------------------------------------------
  for (std::uint32_t i = 0; i < snapshot.entity_count && i < proto::kMaxEntities; ++i) {
    const proto::EntitySnapshot& entity = snapshot.entities[i];
    const proto::Vec3d root = root_position(entity);
    const ScreenPoint at = view.project(root);
    const bool selected = entity.id == snapshot.selected;
    if (!on_screen(at, max_x, max_y, 2.0)) {
      continue;
    }

    Color color = entity.kind == proto::Kind::Debris ? colors::kDebris : colors::kVessel;
    if (has_flag(entity.flags, proto::Flags::Escaping)) {
      color = colors::kWarning;
    }

    const double radius = std::max(view.to_pixels(entity.radius), options.min_entity_radius);

    Marker marker;
    marker.disc.kind = PrimitiveKind::Disc;
    marker.disc.color = selected ? colors::kSelection : color;
    marker.disc.points = {at};
    marker.disc.radius_y = radius;
    marker.disc.radius_x = radius * aspect;
    marker.depth = view.depth(root);

    if (selected) {
      // A reticle around the selected object. The camera centre is on the
      // object, so without one a zoomed-in view gives no clue which of several
      // markers is the one being flown.
      marker.has_reticle = true;
      marker.reticle.kind = PrimitiveKind::Cross;
      marker.reticle.color = colors::kSelection;
      marker.reticle.points = {at};
      marker.reticle.radius_y = radius + 1.0;
      marker.reticle.radius_x = (radius + 1.0) * aspect;
    }
    entities.push_back(std::move(marker));

    if (options.show_labels) {
      labels.push_back(
          PendingLabel{ScreenPoint{at.x + radius * aspect + options.label_offset,
                                   at.y},
                       std::string(entity.name.view()), colors::kLabel, selected ? 0 : 1});
    }
  }

  // --- marker order ---------------------------------------------------------
  //
  // The flat map keeps the snapshot's order, which is also the order it drew
  // them in before there was any choice. A tilted view sorts far to near, so a
  // nearer marker paints over a farther one; ties keep bodies ahead of entities
  // because that is the order they were collected in and the sort is stable.
  if (depth_sort) {
    std::vector<Marker> merged;
    merged.reserve(bodies.size() + entities.size());
    merged.insert(merged.end(), bodies.begin(), bodies.end());
    merged.insert(merged.end(), entities.begin(), entities.end());
    std::stable_sort(merged.begin(), merged.end(),
                     [](const Marker& a, const Marker& b) { return a.depth > b.depth; });
    for (const Marker& marker : merged) {
      emit_marker(out, marker);
    }
  } else {
    for (const Marker& marker : entities) {
      emit_marker(out, marker);
    }
  }

  // --- labels ---------------------------------------------------------------
  //
  // Emitted last so they sit on top of every marker. A label is dropped when it
  // would run off the right edge or when every row near its marker is already
  // taken: two markers almost on top of each other would otherwise interleave
  // their names into one unreadable word. Before dropping one, the nearby rows
  // are tried, so a crowded view loses a name only when it is genuinely out of
  // room. Priority decides who gets first pick, so the tracked object is never
  // the one that loses its name.
  if (options.show_labels) {
    std::stable_sort(labels.begin(), labels.end(),
                     [](const PendingLabel& a, const PendingLabel& b) {
                       return a.priority < b.priority;
                     });

    // Rows to try, in order: straight out from the marker first, then up and
    // down. Two rows either way still reads as belonging to the marker.
    constexpr int kRowOffsets[] = {0, -1, 1, -2, 2};

    std::vector<PlacedLabel> placed;
    placed.reserve(labels.size());
    for (const PendingLabel& label : labels) {
      const ScreenPoint at = label.at;
      const double length = static_cast<double>(label.text.size());
      if (at.x < 0.0 || at.x + length > max_x || at.y < 0.0 || at.y > max_y) {
        continue;
      }

      const int base_row = static_cast<int>(std::floor(at.y + 0.5));
      const int x0 = static_cast<int>(std::floor(at.x));
      const int x1 = x0 + static_cast<int>(label.text.size());

      int row = base_row;
      bool room = false;
      for (const int dy : kRowOffsets) {
        const int candidate = base_row + dy;
        if (candidate < 0 || candidate > static_cast<int>(max_y)) {
          continue;
        }
        bool clash = false;
        for (const PlacedLabel& other : placed) {
          // A blank column between neighbours keeps adjacent labels legible.
          if (other.row == candidate && x0 <= other.x1 && other.x0 <= x1) {
            clash = true;
            break;
          }
        }
        if (!clash) {
          row = candidate;
          room = true;
          break;
        }
      }
      if (!room) {
        continue;
      }
      placed.push_back(PlacedLabel{row, x0, x1});

      Primitive& text = out.add(PrimitiveKind::Text, label.color);
      text.points = {ScreenPoint{at.x, static_cast<double>(row)}};
      text.text = label.text;
    }
  }

  // --- scale bar ------------------------------------------------------------
  if (options.show_scale_bar && mpp > 0.0 && max_x > 8.0 && max_y >= 2.0) {
    // A bar covering roughly a quarter of the width, rounded down to a 1-2-5
    // value so the number next to it reads as a round quantity of metres.
    const double wanted_metres = 0.25 * max_x * mpp / aspect;
    const double spacing = choose_grid_spacing(wanted_metres, 1.0);
    const double bar_metres = spacing <= wanted_metres ? spacing : spacing / 2.0;
    const double bar_units = bar_metres * aspect / mpp;

    const double y = max_y - 1.0;
    Primitive& bar = out.add(PrimitiveKind::Polyline, colors::kLabel);
    bar.width = 1.0;
    bar.points = {ScreenPoint{2.0, y}, ScreenPoint{2.0 + bar_units, y}};

    Primitive& cap_left = out.add(PrimitiveKind::Polyline, colors::kLabel);
    cap_left.points = {ScreenPoint{2.0, y - 1.0}, ScreenPoint{2.0, y}};
    Primitive& cap_right = out.add(PrimitiveKind::Polyline, colors::kLabel);
    cap_right.points = {ScreenPoint{2.0 + bar_units, y - 1.0}, ScreenPoint{2.0 + bar_units, y}};

    Primitive& caption = out.add(PrimitiveKind::Text, colors::kLabel);
    caption.points = {ScreenPoint{2.0 + bar_units + 1.0, y}};
    caption.text = format_length(bar_metres);

    out.scale_bar_metres = bar_metres;
  }
}

}  // namespace rocketlab::render
