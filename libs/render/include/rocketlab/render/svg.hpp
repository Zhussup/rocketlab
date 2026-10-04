// A vector backend for Scene.
//
// Not a user-facing feature — it is how the scene layer is tested. A map that
// is only ever visible inside a terminal cannot be asserted on, and "the
// trajectory line is there and its endpoints are where the orbit says they
// should be" is exactly the sort of thing that should be checked by a test
// rather than by squinting at a terminal. Dumping the same scene the TUI draws
// also makes a visual regression obvious after a camera change.
//
// It doubles as the M4 deliverable's shared half: the GUI backend consumes the
// identical draw list.

#pragma once

#include <string>

#include "rocketlab/render/scene.hpp"

namespace rocketlab::render {

struct SvgOptions {
  /// Multiplies the viewBox so the picture is legible when opened. Units are
  /// character cells, so a cell is `cell_aspect` wide and one unit tall.
  double cell_aspect{2.0};
  double scale{8.0};
  bool background{true};
};

/// Renders the scene as a standalone SVG document.
[[nodiscard]] std::string to_svg(const Scene& scene, const SvgOptions& options = {});

}  // namespace rocketlab::render
