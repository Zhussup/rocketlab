#include "canvas_backend.hpp"

#include <algorithm>
#include <cmath>

namespace rocketlab::tui {

namespace {

/// Keeps a coordinate in a range where the cast to int cannot overflow. A body
/// or a clipped line endpoint can legitimately be at 1e15 cell units, and an
/// undefined conversion there would be a real bug rather than a cosmetic one.
constexpr double kCoordinateLimit = 1.0e6;

[[nodiscard]] int to_cell(double value) noexcept {
  const double clamped = std::clamp(value, -kCoordinateLimit, kCoordinateLimit);
  return static_cast<int>(std::lround(clamped));
}

/// A filled ellipse, scanned in cell space.
///
/// Not Canvas::DrawBlockEllipseFilled: that one works in half-rows internally
/// and its radii are consequently measured in half-row units, which is an easy
/// thing to get subtly wrong. Filling the box directly is a few lines and its
/// units are unambiguous.
void fill_ellipse(ftxui::Canvas& canvas, double cx, double cy, double rx, double ry,
                  const ftxui::Color& color) {
  if (canvas.width() <= 0 || canvas.height() <= 0) {
    return;
  }

  // A marker is never allowed to disappear, so the radii have a floor of half a
  // cell even when the object is metres across at solar-system zoom.
  const double ax = std::max(rx, 0.5);
  const double ay = std::max(ry, 0.5);

  const int x0 = std::max(0, to_cell(std::floor(cx - ax)));
  const int x1 = std::min(canvas.width() - 1, to_cell(std::ceil(cx + ax)));
  const int y0 = std::max(0, to_cell(std::floor(cy - ay)));
  const int y1 = std::min(canvas.height() - 1, to_cell(std::ceil(cy + ay)));

  for (int y = y0; y <= y1; ++y) {
    const double dy = (static_cast<double>(y) + 0.5 - cy) / ay;
    for (int x = x0; x <= x1; ++x) {
      const double dx = (static_cast<double>(x) + 0.5 - cx) / ax;
      if (dx * dx + dy * dy <= 1.0) {
        canvas.DrawBlock(x, y, true, color);
      }
    }
  }
}

void draw_segment(ftxui::Canvas& canvas, const render::ScreenPoint& a,
                  const render::ScreenPoint& b, const ftxui::Color& color) {
  if (!std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(b.x) || !std::isfinite(b.y)) {
    return;
  }
  canvas.DrawPointLine(to_cell(a.x), to_cell(a.y), to_cell(b.x), to_cell(b.y), color);
}

}  // namespace

ftxui::Color to_ftxui(render::Color color) {
  return ftxui::Color::RGBA(color.r, color.g, color.b, color.a);
}

void paint(const render::Scene& scene, ftxui::Canvas& canvas) {
  for (const render::Primitive& primitive : scene.primitives) {
    const ftxui::Color color = to_ftxui(primitive.color);

    switch (primitive.kind) {
      case render::PrimitiveKind::Polyline: {
        for (std::size_t i = 1; i < primitive.points.size(); ++i) {
          draw_segment(canvas, primitive.points[i - 1], primitive.points[i], color);
        }
        break;
      }
      case render::PrimitiveKind::Disc: {
        if (!primitive.points.empty()) {
          fill_ellipse(canvas, primitive.points[0].x, primitive.points[0].y, primitive.radius_x,
                       primitive.radius_y, color);
        }
        break;
      }
      case render::PrimitiveKind::Cross: {
        if (primitive.points.empty()) {
          break;
        }
        // A gap in the middle, so the reticle frames the object instead of
        // covering it.
        const render::ScreenPoint at = primitive.points[0];
        const double gap = 1.0;
        draw_segment(canvas, render::ScreenPoint{at.x, at.y - primitive.radius_y},
                     render::ScreenPoint{at.x, at.y - gap}, color);
        draw_segment(canvas, render::ScreenPoint{at.x, at.y + gap},
                     render::ScreenPoint{at.x, at.y + primitive.radius_y}, color);
        draw_segment(canvas, render::ScreenPoint{at.x - primitive.radius_x, at.y},
                     render::ScreenPoint{at.x - gap, at.y}, color);
        draw_segment(canvas, render::ScreenPoint{at.x + gap, at.y},
                     render::ScreenPoint{at.x + primitive.radius_x, at.y}, color);
        break;
      }
      case render::PrimitiveKind::Text: {
        if (primitive.points.empty() || primitive.text.empty()) {
          break;
        }
        canvas.DrawText(to_cell(primitive.points[0].x), to_cell(primitive.points[0].y),
                        primitive.text, color);
        break;
      }
    }
  }
}

}  // namespace rocketlab::tui
