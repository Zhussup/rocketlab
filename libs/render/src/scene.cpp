#include "rocketlab/render/scene.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <string_view>

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
};

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
  out.clear();
  out.width = camera.width;
  out.height = camera.height;

  const double max_x = static_cast<double>(std::max(0, camera.width - 1));
  const double max_y = static_cast<double>(std::max(0, camera.height - 1));
  const double aspect = camera.cell_aspect > 0.0 ? camera.cell_aspect : 1.0;
  const double mpp = camera.metres_per_pixel;

  std::vector<PendingLabel> labels;

  // --- grid -----------------------------------------------------------------
  //
  // Drawn first so everything else paints over it. Lines are placed on round
  // root-frame coordinates rather than on screen multiples, so they stay put
  // while the camera pans.
  if (options.show_grid && mpp > 0.0) {
    const double spacing = choose_grid_spacing(mpp, 8.0 * aspect);
    const double half_span_x = 0.5 * max_x * mpp / aspect;
    const double half_span_y = 0.5 * max_y * mpp;

    const double x_lo = camera.center_x - half_span_x;
    const double x_hi = camera.center_x + half_span_x;
    const double y_lo = camera.center_y - half_span_y;
    const double y_hi = camera.center_y + half_span_y;

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
        const double sx = 0.5 * max_x + aspect * (x - camera.center_x) / mpp;
        Primitive& line = out.add(PrimitiveKind::Polyline,
                                  std::abs(x) < spacing * 0.5 ? colors::kAxis : colors::kGrid);
        line.width = 1.0;
        line.points = {ScreenPoint{sx, 0.0}, ScreenPoint{sx, max_y}};
      }
    }
    if (rows > 0 && rows < kMaxGridLines) {
      for (int i = 0; i < rows; ++i) {
        const double y = first_y + i * spacing;
        const double sy = 0.5 * max_y - (y - camera.center_y) / mpp;
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
      const ScreenPoint at = camera.project(body.position);
      if (!on_screen(at, max_x, max_y, 0.0)) {
        continue;
      }

      const double radius = std::max(camera.to_pixels(body.radius), options.min_body_radius);
      Primitive& disc = out.add(PrimitiveKind::Disc, colors::kBody);
      disc.points = {at};
      disc.radius_y = radius;
      disc.radius_x = radius * aspect;

      if (options.show_labels) {
        labels.push_back(PendingLabel{
            ScreenPoint{at.x + disc.radius_x + options.label_offset, at.y},
            std::string(body.name.view()), colors::kBodyDim});
      }
    }
  }

  // --- predicted path -------------------------------------------------------
  if (options.show_trajectory && !trajectory.empty()) {
    std::vector<ScreenPoint> path;
    path.reserve(trajectory.size());
    for (const proto::Vec3d& point : trajectory) {
      path.push_back(camera.project(point));
    }
    append_path(out, colors::kTrajectory, 1.0, path, max_x, max_y);
  }

  // --- entities -------------------------------------------------------------
  for (std::uint32_t i = 0; i < snapshot.entity_count && i < proto::kMaxEntities; ++i) {
    const proto::EntitySnapshot& entity = snapshot.entities[i];
    const ScreenPoint at = camera.project(root_position(entity));
    const bool selected = entity.id == snapshot.selected;
    if (!on_screen(at, max_x, max_y, 2.0)) {
      continue;
    }

    Color color = entity.kind == proto::Kind::Debris ? colors::kDebris : colors::kVessel;
    if (has_flag(entity.flags, proto::Flags::Escaping)) {
      color = colors::kWarning;
    }

    const double radius = std::max(camera.to_pixels(entity.radius), options.min_entity_radius);

    Primitive& disc = out.add(PrimitiveKind::Disc, selected ? colors::kSelection : color);
    disc.points = {at};
    disc.radius_y = radius;
    disc.radius_x = radius * aspect;

    if (selected) {
      // A reticle around the selected object. The camera centre is on the
      // object, so without one a zoomed-in view gives no clue which of several
      // markers is the one being flown.
      Primitive& reticle = out.add(PrimitiveKind::Cross, colors::kSelection);
      reticle.points = {at};
      reticle.radius_y = radius + 1.0;
      reticle.radius_x = (radius + 1.0) * aspect;
    }

    if (options.show_labels) {
      labels.push_back(PendingLabel{ScreenPoint{at.x + disc.radius_x + options.label_offset, at.y},
                                    std::string(entity.name.view()), colors::kLabel});
    }
  }

  // --- labels ---------------------------------------------------------------
  //
  // Emitted last so they sit on top of every marker, and skipped when they
  // would run off the right edge rather than being truncated into nonsense.
  if (options.show_labels) {
    for (const PendingLabel& label : labels) {
      const ScreenPoint at = label.at;
      if (at.x < 0.0 ||
          at.x + static_cast<double>(label.text.size()) > max_x ||
          at.y < 0.0 || at.y > max_y) {
        continue;
      }
      Primitive& text = out.add(PrimitiveKind::Text, label.color);
      text.points = {at};
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
