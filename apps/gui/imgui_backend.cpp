#include "imgui_backend.hpp"

#include <vector>

namespace rocketlab::gui {

namespace {

namespace render = rocketlab::render;

/// Packed into the byte order ImDrawList wants, which is why `Color` carries
/// four 8-bit channels and no toolkit's type.
[[nodiscard]] ImU32 packed(const render::Color& color) noexcept {
  return IM_COL32(color.r, color.g, color.b, color.a);
}

[[nodiscard]] ImVec2 point(const render::ScreenPoint& at, const ImVec2& origin) noexcept {
  return ImVec2(static_cast<float>(at.x) + origin.x, static_cast<float>(at.y) + origin.y);
}

}  // namespace

void paint(const render::Scene& scene, ImDrawList* draw_list, const ImVec2 origin) {
  if (draw_list == nullptr) {
    return;
  }

  // Reused across primitives: a scene can hold a few thousand path samples and
  // allocating a vector for each polyline would be the only per-frame heap
  // traffic in the client.
  std::vector<ImVec2> points;

  for (const render::Primitive& primitive : scene.primitives) {
    const ImU32 color = packed(primitive.color);
    // A hairline is one pixel. The scene's sub-pixel widths exist for the vector
    // backends, which can use them; a rasteriser cannot draw thinner than a pixel
    // and pretending otherwise only produces a blurred line.
    const float width = static_cast<float>(primitive.width < 1.0 ? 1.0 : primitive.width);

    switch (primitive.kind) {
      case render::PrimitiveKind::Polyline: {
        if (primitive.points.size() < 2) {
          break;
        }
        points.clear();
        points.reserve(primitive.points.size());
        for (const render::ScreenPoint& at : primitive.points) {
          points.push_back(point(at, origin));
        }
        // Antialiased, which is what the scene's fractional coordinates are for.
        draw_list->AddPolyline(points.data(), static_cast<int>(points.size()), color,
                               ImDrawFlags_None, width);
        break;
      }
      case render::PrimitiveKind::Disc: {
        if (primitive.points.empty()) {
          break;
        }
        // Filled, to match the SVG backend's `<ellipse fill=...>`. A body drawn as
        // an outline here and as a solid there would be two different pictures of
        // the same sky.
        draw_list->AddEllipseFilled(point(primitive.points[0], origin),
                                    ImVec2(static_cast<float>(primitive.radius_x),
                                           static_cast<float>(primitive.radius_y)),
                                    color);
        break;
      }
      case render::PrimitiveKind::Cross: {
        if (primitive.points.empty()) {
          break;
        }
        const ImVec2 at = point(primitive.points[0], origin);
        const float rx = static_cast<float>(primitive.radius_x);
        const float ry = static_cast<float>(primitive.radius_y);
        draw_list->AddLine(ImVec2(at.x, at.y - ry), ImVec2(at.x, at.y + ry), color, width);
        draw_list->AddLine(ImVec2(at.x - rx, at.y), ImVec2(at.x + rx, at.y), color, width);
        break;
      }
      case render::PrimitiveKind::Text: {
        if (primitive.points.empty() || primitive.text.empty()) {
          break;
        }
        // The scene anchors a label at its left edge and its vertical centre,
        // matching the SVG backend's `dominant-baseline="middle"`.
        const ImVec2 size = ImGui::CalcTextSize(primitive.text.c_str());
        const ImVec2 at = point(primitive.points[0], origin);
        draw_list->AddText(ImVec2(at.x, at.y - size.y * 0.5f), color, primitive.text.c_str());
        break;
      }
    }
  }
}

}  // namespace rocketlab::gui
