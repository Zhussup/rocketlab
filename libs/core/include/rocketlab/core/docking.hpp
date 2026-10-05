// Whether two vessels are in a position to be joined, and what joining them
// produces.
//
// The predicate is deliberately pure arithmetic on state both sides already
// have: separation, the two radii, and the relative speed. It lives in core
// rather than in the host that acts on it, because two copies of "close enough
// and slow enough" is how a client ends up offering a manoeuvre the host then
// refuses. The numbers have to be one set, and this is where they are.
//
// What is deliberately *not* modelled: attitude, and docking ports. Two vessels
// meet if their centres pass within a few metres at walking pace, whichever way
// up they are and wherever a port would have been. That is an abstraction, but
// a honest one — the alternative is a client told "not aligned" with no control
// that would ever align anything.

#pragma once

#include <cstdint>

#include "rocketlab/core/entity.hpp"
#include "rocketlab/core/orbital.hpp"

namespace rocketlab::core {

/// The numbers that decide whether a docking attempt is a docking.
struct DockingLimits {
  /// How far apart the two surfaces may be and still count as touching [m].
  ///
  /// Not zero, and it cannot be: the state is integrated rather than solved, so
  /// two vessels are never at exactly zero separation and a tolerance of zero
  /// would be a manoeuvre that can never be performed.
  double surface_clearance{5.0};

  /// The greatest speed at which they may approach each other [m/s].
  ///
  /// This is the part that makes docking a manoeuvre rather than a collision.
  /// Two objects in low Earth orbit pass within metres of each other at
  /// kilometres a second all the time; without this limit every near miss would
  /// be a docking, and the mass of a spent stage would silently join whatever
  /// it happened to brush past.
  double max_closing_speed{2.0};
};

/// The limits a client and the host both use unless something says otherwise.
inline constexpr DockingLimits kDefaultDockingLimits{};

/// Why a docking attempt was refused.
enum class DockBlock : std::uint8_t {
  None = 0,
  /// The same entity on both sides.
  SameEntity,
  /// Docking joins two stacks. An entity with no parts is a point mass — a
  /// scenario's placeholder, or a spent stage — and has no stack to contribute.
  NoParts,
  /// The two are in different frames. Joining them would mean re-expressing one
  /// vessel's state in the other's parent, and the pair are about to become one
  /// vessel: sharing a frame is part of what "docked" means, not an incidental
  /// detail of how the merge was implemented.
  DifferentParent,
  /// Surfaces further apart than `surface_clearance`.
  TooFar,
  /// Closing faster than `max_closing_speed`.
  TooFast,
};

[[nodiscard]] const char* to_string(DockBlock block) noexcept;

/// What the predicate looks at, in the frame the two share.
///
/// A plain value so that the host can fill it from two `Entity`s and a client
/// from two entries of a snapshot, with one predicate answering for both.
struct DockingGeometry {
  double separation{0.0};       // centre to centre [m]
  double combined_radius{0.0};  // the two radii added [m]
  double closing_speed{0.0};    // |v_absorbed - v_target| [m/s]
};

/// The geometry of a docking, from parent-frame states and radii.
///
/// Both states are read in the same frame, which is why callers must have
/// established that the two share a parent: subtract two positions in different
/// frames and the answer is not a distance.
[[nodiscard]] DockingGeometry docking_geometry(const StateVector& target, const StateVector& absorbed,
                                               double target_radius,
                                               double absorbed_radius) noexcept;

/// The same, for two entities that share a parent.
[[nodiscard]] DockingGeometry docking_geometry(const Entity& target,
                                               const Entity& absorbed) noexcept;

/// The first reason this docking cannot happen, or `None`.
///
/// Ordered by what a person would want to be told first: the structural
/// refusals before the geometric ones, and distance before speed, because "too
/// far" is the one a pilot can do something about.
[[nodiscard]] DockBlock dock_block(const DockingGeometry& geometry,
                                   const DockingLimits& limits) noexcept;

[[nodiscard]] bool can_dock(const DockingGeometry& geometry, const DockingLimits& limits) noexcept;

/// The same, for two entities, including the checks that are not geometry: that
/// they are different, that both are built from parts, and that they are in the
/// same frame.
[[nodiscard]] DockBlock dock_block(const Entity& target, const Entity& absorbed,
                                   const DockingLimits& limits) noexcept;

}  // namespace rocketlab::core
