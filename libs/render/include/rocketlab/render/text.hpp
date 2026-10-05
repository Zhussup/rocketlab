// A plain-character backend for Scene.
//
// Two reasons it exists rather than being a throwaway debug hack. It makes the
// map checkable in a test: "there is a trajectory cell and a body cell, and
// they are where the orbit says" is an assertion a vector or terminal drawing
// cannot give you for free. And it makes the map available where no terminal
// UI is — over a pipe, in a log, in a CI transcript.
//
// Shape comes from the primitive kind and identity from its colour, since a
// character has no colour of its own:
//   '#' body   'O' vessel   'x' debris   '*' trajectory
//   '.' grid   ':' axis     '+' reticle  letters for labels

#pragma once

#include <string>

#include "rocketlab/render/scene.hpp"

namespace rocketlab::render {

/// Renders a scene into `scene.height` lines of `scene.width` characters.
[[nodiscard]] std::string to_text(const Scene& scene);

}  // namespace rocketlab::render
