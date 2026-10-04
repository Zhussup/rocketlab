#pragma once

#include <ftxui/dom/canvas.hpp>
#include <ftxui/screen/color.hpp>

#include "rocketlab/render/scene.hpp"

namespace rocketlab::tui {

[[nodiscard]] ftxui::Color to_ftxui(render::Color color);

/// Draws a scene onto a character canvas.
///
/// This is the only place that knows the terminal is a grid of characters with
/// a 1:2 cell. The scene arrives in cell coordinates with the aspect already
/// applied to radii, so there is no correction to make here — which is exactly
/// why the correction lives in the camera instead.
void paint(const render::Scene& scene, ftxui::Canvas& canvas);

}  // namespace rocketlab::tui
