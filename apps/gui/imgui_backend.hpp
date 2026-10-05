// The ImDrawList backend for a scene.
//
// The fourth of them, after SVG, plain characters and the FTXUI canvas, and the
// reason the scene exists as a flat primitive list in the first place. It is a
// loop and a switch and nothing else: every decision about *where* anything goes
// was already made by `render::build_scene`, and every decision about *what*
// anything means is in the primitive's colour and kind. A backend that starts
// making decisions of its own is a backend that will disagree with the others.
//
// ImGui is third-party and this project's warning set is strict about our code,
// so the include lives here and in one `.cpp`, never in `libs/render`.

#pragma once

#include <imgui.h>

#include "rocketlab/render/scene.hpp"

namespace rocketlab::gui {

/// Paints `scene` into `draw_list`, shifted by `origin`.
///
/// The origin is there because an ImGui draw list works in screen coordinates
/// while a scene works in its own, so the caller passes the top-left of the
/// region it got from ImGui and every primitive is offset by it. Clipping is
/// ImGui's job: the window's clip rectangle is already in force inside a window.
void paint(const render::Scene& scene, ImDrawList* draw_list, ImVec2 origin);

}  // namespace rocketlab::gui
