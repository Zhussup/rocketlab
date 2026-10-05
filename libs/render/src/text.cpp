#include "rocketlab/render/text.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace rocketlab::render {

namespace {

[[nodiscard]] char glyph_for(const Primitive& primitive) {
  switch (primitive.kind) {
    case PrimitiveKind::Disc:
      if (primitive.color == colors::kBody || primitive.color == colors::kBodyDim) {
        return '#';
      }
      if (primitive.color == colors::kDebris) {
        return 'x';
      }
      return 'O';
    case PrimitiveKind::Cross:
      return '+';
    case PrimitiveKind::Polyline:
      if (primitive.color == colors::kTrajectory) {
        return '*';
      }
      if (primitive.color == colors::kAxis) {
        return ':';
      }
      if (primitive.color == colors::kGrid) {
        return '.';
      }
      return '-';
    case PrimitiveKind::Text:
      return ' ';
  }
  return '?';
}

/// An integer cell index, with the same overflow guard the canvas backend uses.
[[nodiscard]] int cell_of(double value, int limit) noexcept {
  const double clamped = std::clamp(value, -1.0e6, 1.0e6);
  return std::clamp(static_cast<int>(std::lround(clamped)), 0, std::max(0, limit - 1));
}

void plot(std::vector<std::string>& rows, int x, int y, char glyph) {
  if (y < 0 || y >= static_cast<int>(rows.size())) {
    return;
  }
  std::string& row = rows[static_cast<std::size_t>(y)];
  if (x < 0 || x >= static_cast<int>(row.size())) {
    return;
  }
  row[static_cast<std::size_t>(x)] = glyph;
}

/// A line of single cells. Not Bresenham: stepping along the longer axis and
/// rounding keeps the loop short and is indistinguishable at this resolution.
void draw_line(std::vector<std::string>& rows, const ScreenPoint& from, const ScreenPoint& to,
               int width, int height, char glyph) {
  if (!std::isfinite(from.x) || !std::isfinite(from.y) || !std::isfinite(to.x) ||
      !std::isfinite(to.y)) {
    return;
  }
  const int x0 = cell_of(from.x, width);
  const int y0 = cell_of(from.y, height);
  const int x1 = cell_of(to.x, width);
  const int y1 = cell_of(to.y, height);

  const int dx = std::abs(x1 - x0);
  const int dy = std::abs(y1 - y0);
  const int steps = std::max(dx, dy);
  if (steps == 0) {
    plot(rows, x0, y0, glyph);
    return;
  }
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    plot(rows, static_cast<int>(std::lround(x0 + t * (x1 - x0))),
         static_cast<int>(std::lround(y0 + t * (y1 - y0))), glyph);
  }
}

}  // namespace

std::string to_text(const Scene& scene) {
  if (scene.width <= 0 || scene.height <= 0) {
    return {};
  }

  std::vector<std::string> rows(static_cast<std::size_t>(scene.height),
                               std::string(static_cast<std::size_t>(scene.width), ' '));

  for (const Primitive& primitive : scene.primitives) {
    switch (primitive.kind) {
      case PrimitiveKind::Polyline:
        for (std::size_t i = 1; i < primitive.points.size(); ++i) {
          draw_line(rows, primitive.points[i - 1], primitive.points[i], scene.width, scene.height,
                    glyph_for(primitive));
        }
        break;

      case PrimitiveKind::Disc: {
        if (primitive.points.empty()) {
          break;
        }
        const char glyph = glyph_for(primitive);
        const ScreenPoint at = primitive.points[0];
        const double rx = std::max(primitive.radius_x, 0.5);
        const double ry = std::max(primitive.radius_y, 0.5);
        const int y0 = cell_of(at.y - ry, scene.height);
        const int y1 = cell_of(at.y + ry, scene.height);
        const int x0 = cell_of(at.x - rx, scene.width);
        const int x1 = cell_of(at.x + rx, scene.width);
        for (int y = y0; y <= y1; ++y) {
          const double ny = (static_cast<double>(y) + 0.5 - at.y) / ry;
          for (int x = x0; x <= x1; ++x) {
            const double nx = (static_cast<double>(x) + 0.5 - at.x) / rx;
            if (nx * nx + ny * ny <= 1.0) {
              plot(rows, x, y, glyph);
            }
          }
        }
        break;
      }

      case PrimitiveKind::Cross: {
        if (primitive.points.empty()) {
          break;
        }
        const ScreenPoint at = primitive.points[0];
        draw_line(rows, ScreenPoint{at.x - primitive.radius_x, at.y},
                  ScreenPoint{at.x + primitive.radius_x, at.y}, scene.width, scene.height, '+');
        draw_line(rows, ScreenPoint{at.x, at.y - primitive.radius_y},
                  ScreenPoint{at.x, at.y + primitive.radius_y}, scene.width, scene.height, '+');
        break;
      }

      case PrimitiveKind::Text: {
        if (primitive.points.empty()) {
          break;
        }
        const int x = cell_of(primitive.points[0].x, scene.width);
        const int y = cell_of(primitive.points[0].y, scene.height);
        for (std::size_t i = 0; i < primitive.text.size(); ++i) {
          plot(rows, x + static_cast<int>(i), y, primitive.text[i]);
        }
        break;
      }
    }
  }

  std::string out;
  out.reserve(rows.size() * (static_cast<std::size_t>(scene.width) + 1));
  for (const std::string& row : rows) {
    out += row;
    out += '\n';
  }
  return out;
}

}  // namespace rocketlab::render
