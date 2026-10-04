#pragma once

#include <cstdint>

namespace rocketlab::render {

/// 8-bit RGBA. Kept independent of any UI toolkit so the scene layer does not
/// drag one in; a backend converts at the boundary (FTXUI wants its own
/// `Color`, ImGui wants a packed `ImU32`, SVG wants a hex string).
struct Color {
  std::uint8_t r{255};
  std::uint8_t g{255};
  std::uint8_t b{255};
  std::uint8_t a{255};

  [[nodiscard]] constexpr bool operator==(const Color&) const noexcept = default;
};

/// Palette. Chosen to stay legible on both light and dark terminal themes,
/// which rules out pure black and pure white as line colours.
namespace colors {

inline constexpr Color kBackground{12, 14, 20, 255};
inline constexpr Color kGrid{48, 54, 68, 255};
inline constexpr Color kAxis{72, 80, 98, 255};
inline constexpr Color kBody{255, 196, 92, 255};
inline constexpr Color kBodyDim{150, 118, 64, 255};
inline constexpr Color kSelection{255, 255, 255, 255};
inline constexpr Color kVessel{96, 220, 255, 255};
inline constexpr Color kDebris{140, 140, 150, 255};
inline constexpr Color kTrajectory{255, 120, 160, 255};
inline constexpr Color kLabel{200, 208, 224, 255};
inline constexpr Color kWarning{255, 108, 96, 255};
inline constexpr Color kPanel{22, 26, 36, 255};

}  // namespace colors

}  // namespace rocketlab::render
