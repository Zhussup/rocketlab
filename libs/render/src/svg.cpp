#include "rocketlab/render/svg.hpp"

#include <format>
#include <string_view>

namespace rocketlab::render {

namespace {

/// Coordinates are emitted in cell space and the aspect is applied by a
/// non-uniform viewBox instead of by touching every number. `preserveAspectRatio="none"`
/// is what allows that, and it is why a circle in the scene stays a circle on
/// screen despite the cells not being square.
[[nodiscard]] std::string number(double value) { return std::format("{:.2f}", value); }

[[nodiscard]] std::string hex_color(const Color& color) {
  return std::format("#{:02x}{:02x}{:02x}", color.r, color.g, color.b);
}

[[nodiscard]] std::string escape(std::string_view text) {
  std::string out;
  out.reserve(text.size());
  for (const char ch : text) {
    switch (ch) {
      case '&': out += "&amp;"; break;
      case '<': out += "&lt;"; break;
      case '>': out += "&gt;"; break;
      case '"': out += "&quot;"; break;
      case '\'': out += "&apos;"; break;
      default: out += ch; break;
    }
  }
  return out;
}

/// A text anchor at a fractional x would otherwise land wherever the font
/// metrics feel like; the scene works in cells, so round to one.
[[nodiscard]] double snap(double value) { return static_cast<double>(static_cast<long>(value)); }

}  // namespace

std::string to_svg(const Scene& scene, const SvgOptions& options) {
  const double aspect = options.cell_aspect > 0.0 ? options.cell_aspect : 1.0;
  const double scale = options.scale > 0.0 ? options.scale : 1.0;

  const double width = static_cast<double>(scene.width);
  const double height = static_cast<double>(scene.height);

  std::string out;
  out += std::format(
      R"(<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {} {}" width="{:.0f}" height="{:.0f}" preserveAspectRatio="none">)",
      width, height, width * aspect * scale, height * scale);

  if (options.background) {
    out += std::format(R"(<rect x="0" y="0" width="{}" height="{}" fill="{}"/>)", width, height,
                       hex_color(colors::kBackground));
  }

  for (const Primitive& primitive : scene.primitives) {
    const std::string color = hex_color(primitive.color);

    switch (primitive.kind) {
      case PrimitiveKind::Polyline: {
        if (primitive.points.size() < 2) {
          break;
        }
        out += std::format(R"(<polyline fill="none" stroke="{}" stroke-width="{}" points=")",
                           color, number(primitive.width));
        for (const ScreenPoint& point : primitive.points) {
          out += std::format("{},{} ", number(point.x), number(point.y));
        }
        out += R"("/>)";
        break;
      }
      case PrimitiveKind::Disc: {
        if (primitive.points.empty()) {
          break;
        }
        out += std::format(R"(<ellipse cx="{}" cy="{}" rx="{}" ry="{}" fill="{}"/>)",
                           number(primitive.points[0].x), number(primitive.points[0].y),
                           number(primitive.radius_x), number(primitive.radius_y), color);
        break;
      }
      case PrimitiveKind::Cross: {
        if (primitive.points.empty()) {
          break;
        }
        const ScreenPoint at = primitive.points[0];
        out += std::format(
            R"(<path fill="none" stroke="{}" stroke-width="{}" d="M {} {} V {} M {} {} H {}"/>)",
            color, number(primitive.width), number(at.x), number(at.y - primitive.radius_y),
            number(at.y + primitive.radius_y), number(at.x - primitive.radius_x), number(at.y),
            number(at.x + primitive.radius_x));
        break;
      }
      case PrimitiveKind::Text: {
        if (primitive.points.empty()) {
          break;
        }
        const ScreenPoint at = primitive.points[0];
        out += std::format(
            R"(<text x="{}" y="{}" fill="{}" font-family="monospace" font-size="1" )"
            R"(dominant-baseline="middle">{}</text>)",
            number(snap(at.x)), number(snap(at.y)), color, escape(primitive.text));
        break;
      }
    }
  }

  out += "</svg>";
  return out;
}

}  // namespace rocketlab::render
