// Wire format shared by the daemon and every client.
//
// Everything here is a plain-data, trivially-copyable struct with a fixed
// size and no pointers, because the same bytes have to survive a memcpy into
// shared memory. That rules out std::string, std::vector and virtual
// functions in this header.
//
// Note what is NOT here: no orbital elements beyond the summary a HUD needs,
// and no predicted path. Anything that requires propagating an orbit is a
// query (see sim_source.hpp), answered by the daemon, because clients must not
// do physics.

#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>
#include <type_traits>

namespace rocketlab::proto {

inline constexpr std::size_t kMaxEntities = 256;
inline constexpr std::size_t kMaxBodies = 32;
/// Including the terminating NUL. Names longer than this are truncated when
/// the snapshot is published, which is safe at the display layer.
inline constexpr std::size_t kMaxNameLength = 32;

/// Mirrors core::EntityKind without including core.
enum class Kind : std::uint8_t { Vessel = 0, Debris = 1 };

/// A fixed-capacity, always-NUL-terminated string. Truncation is silent and
/// deliberate: a snapshot is a wire type, not a place to report errors.
struct Name {
  char data[kMaxNameLength]{};

  void assign(std::string_view text) noexcept;
  [[nodiscard]] std::string_view view() const noexcept;
};

/// Position or velocity. `double[3]` would do, but a named type keeps the
/// conversion code in the host readable.
struct Vec3d {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

enum class Flags : std::uint8_t {
  None = 0,
  Controllable = 1U << 0U,
  /// The orbit is too degenerate for a useful element set (radial or
  /// rectilinear); the element fields below should not be displayed.
  Degenerate = 1U << 1U,
  /// The entity is on an escape trajectory; `period` is meaningless.
  Escaping = 1U << 2U,
};

[[nodiscard]] constexpr Flags operator|(Flags a, Flags b) noexcept {
  return static_cast<Flags>(static_cast<std::uint8_t>(a) | static_cast<std::uint8_t>(b));
}
[[nodiscard]] constexpr bool has_flag(Flags value, Flags bit) noexcept {
  return (static_cast<std::uint8_t>(value) & static_cast<std::uint8_t>(bit)) != 0;
}

struct EntitySnapshot {
  std::uint64_t id{0};

  /// Parent-frame state. This is what the physics uses.
  Vec3d position;
  Vec3d velocity;
  /// Parent body state in the root frame, so a client can compose the root
  /// position itself for rendering. Both are sent rather than the composed
  /// result so that a client wanting relative motion (a chase camera, a
  /// docking view) does not have to undo a subtraction.
  Vec3d parent_position;
  Vec3d parent_velocity;

  double mass{0.0};
  double radius{0.0};

  /// Orbit shape in the parent frame. Angles in radians.
  double periapsis{0.0};
  double apoapsis{0.0};
  double eccentricity{0.0};
  double inclination{0.0};
  double period{0.0};

  Name name;
  Kind kind{Kind::Vessel};
  Flags flags{Flags::None};
  std::uint8_t pad[2]{};
};

struct BodySnapshot {
  std::uint32_t id{0};
  std::uint32_t parent{0};
  Vec3d position;  // root frame
  double mu{0.0};
  double radius{0.0};
  double soi_radius{0.0};
  Name name;
  std::uint32_t pad{0};
};

/// One frame of simulation state.
///
/// `sequence` increments on every publish. A client that sees the same
/// sequence twice knows nothing has moved and can skip redrawing.
struct Snapshot {
  std::uint64_t sequence{0};
  /// Simulation time since J2000, TDB seconds. Never a wall clock.
  double tdb{0.0};
  /// Warp factor in effect. 1.0 is real time, 0.0 is paused.
  double warp{1.0};

  std::uint32_t entity_count{0};
  std::uint32_t body_count{0};
  /// Entity the camera is centred on. Shared state: the daemon owns it so
  /// that two attached clients agree on what is being watched.
  std::uint64_t selected{0};
  std::uint64_t epoch_tdb{0};

  EntitySnapshot entities[kMaxEntities]{};
  BodySnapshot bodies[kMaxBodies]{};
};

static_assert(std::is_trivially_copyable_v<Snapshot>,
              "the snapshot crosses a shared-memory boundary and must stay POD");

}  // namespace rocketlab::proto
