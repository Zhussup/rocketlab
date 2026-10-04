#pragma once

#include <cstdint>
#include <string>

#include "rocketlab/core/body.hpp"

namespace rocketlab::core {

using EntityId = std::uint64_t;

/// Zero is never a valid entity, so it doubles as "none".
inline constexpr EntityId kInvalidEntity = 0;

enum class EntityKind : std::uint8_t {
  Vessel,  // a thing with a flight computer and, later, parts
  Debris,  // spent stages, discarded fairings
};

[[nodiscard]] const char* to_string(EntityKind kind) noexcept;

/// Anything that moves through the world and is not a celestial body.
///
/// The state is stored relative to `parent`, the body currently dominating it.
/// That is the frame the physics is valid in, and it is what keeps the numbers
/// small: a vessel in low Earth orbit has coordinates of order 7e6, not the
/// 1.5e11 it would need in the root frame. The root-frame state is derived on
/// demand for rendering.
struct Entity {
  EntityId id{kInvalidEntity};
  EntityKind kind{EntityKind::Vessel};
  std::string name;
  BodyId parent{kInvalidBody};
  StateVector state;  // relative to `parent`
  double mass{0.0};    // [kg]
  double radius{0.0};  // [m], for rendering and collision
  bool controllable{false};
};

}  // namespace rocketlab::core
