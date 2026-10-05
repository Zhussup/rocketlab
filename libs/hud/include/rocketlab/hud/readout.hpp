// Readouts derived from a snapshot.
//
// Every value here is arithmetic on data the snapshot already carries: no
// propagation, no wall clock, no query. That is what makes it safe for a
// client to compute, and it is why the derived-and-predictive split falls
// where it does — "how fast am I going" is a dot product, "where will I be in
// an hour" is a physics question and goes to the host.
//
// Kept free of any UI toolkit because the terminal client and the graphical
// one must agree on what they display; two copies of a formatter is how the
// same orbit ends up reading as 400 km in one and 400000 m in the other.

#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "rocketlab/core/docking.hpp"
#include "rocketlab/proto/snapshot.hpp"

namespace rocketlab::hud {

/// A label and its value, already formatted. Clients lay these out; they do
/// not decide what the numbers mean.
struct Row {
  std::string label;
  std::string value;
  /// Set on rows a client should draw attention to, such as a warning.
  bool emphasised{false};
};

/// A human-readable length, scaled to whatever unit keeps the number short.
[[nodiscard]] std::string format_length(double metres);
/// A speed in m/s, switching to km/s above a kilometre per second.
[[nodiscard]] std::string format_speed(double metres_per_second);
[[nodiscard]] std::string format_mass(double kilograms);
/// A force in newtons, in kN or MN.
[[nodiscard]] std::string format_thrust(double newtons);
/// A dimensionless ratio to two places, for thrust-to-weight and the like.
[[nodiscard]] std::string format_ratio(double value);
/// Radians to a degrees string.
[[nodiscard]] std::string format_angle(double radians);
/// A duration in the mission-elapsed-time form, or "open" for an unbound orbit.
[[nodiscard]] std::string format_period(double seconds);
/// A density in kg/m^3, in the form that keeps the exponent readable. Air
/// density spans fifteen orders of magnitude from the ground to the top of the
/// atmosphere, so this is the one quantity on the panel that has to be
/// scientific notation to be read at all.
[[nodiscard]] std::string format_density(double kilograms_per_cubic_metre);

/// The parent body of an entity, or null when the snapshot does not contain
/// it. Needed because altitude is measured from the parent's surface, and the
/// body table is the only place the radius lives.
[[nodiscard]] const proto::BodySnapshot* find_body(const proto::Snapshot& snapshot,
                                                   std::uint32_t id) noexcept;

/// The entity with the given id, or null.
[[nodiscard]] const proto::EntitySnapshot* find_entity(const proto::Snapshot& snapshot,
                                                       std::uint64_t id) noexcept;

/// One word for what a computer is doing, lowercase so it fits a table column.
///
/// Lives here rather than in each client because the name a fault is reported
/// under is the difference between a reader seeing "faulted" and wondering
/// whether the script is still flying.
[[nodiscard]] const char* computer_state_name(proto::ComputerState state) noexcept;

/// Everything worth showing about one entity, in display order.
[[nodiscard]] std::vector<Row> entity_readout(const proto::Snapshot& snapshot,
                                              const proto::EntitySnapshot& entity);

/// One line per entity: its name and a one-word state, for a selection list.
struct EntityLine {
  std::uint64_t id{0};
  std::string name;
  std::string parent;
  bool selected{false};
  bool controllable{false};
};

[[nodiscard]] std::vector<EntityLine> entity_list(const proto::Snapshot& snapshot);

/// The separation, combined radius and closing speed of a docking between two
/// snapshot entities.
///
/// Both states are read in the same frame, so this only means anything once the
/// two are known to share a parent — `dock_block` establishes that before it
/// looks at the numbers. A client that wants to sort candidates by how close
/// they are uses this; sorting by centre distance would rank a large station
/// behind a small probe that is actually further from touching.
[[nodiscard]] core::DockingGeometry dock_geometry(const proto::EntitySnapshot& target,
                                                  const proto::EntitySnapshot& absorbed) noexcept;

/// Why two entities in a snapshot cannot dock, or `None` if they can.
///
/// The host decides the same question with `core::dock_block` on two live
/// `Entity`s. This is the client's copy of that decision, and it is here rather
/// than in each client so the terminal and the windowed one cannot disagree
/// about whether the dock key should do anything at all. Nothing is duplicated
/// that could drift: the tolerance comes from `core::DockingLimits` and the
/// distance from `core::docking_geometry`, leaving only the three structural
/// rules, each a look at one field the snapshot already carries.
///
/// It is not a prediction — every input was published by the host in the frame
/// the client is holding. The host still checks for itself; a client's answer
/// decides whether a key does something, never what the simulation does.
[[nodiscard]] core::DockBlock dock_block(const proto::EntitySnapshot& target,
                                         const proto::EntitySnapshot& absorbed,
                                         const core::DockingLimits& limits) noexcept;

/// The same, with the shipped limits.
[[nodiscard]] inline core::DockBlock dock_block(const proto::EntitySnapshot& target,
                                                const proto::EntitySnapshot& absorbed) noexcept {
  return dock_block(target, absorbed, core::kDefaultDockingLimits);
}

/// Which vessel a client's dock key should offer, and what to say when there is
/// none.
///
/// Here rather than in either client because both have a dock key and both must
/// mean the same thing by it: a terminal and a window that picked different
/// candidates — or that disagreed about whether there was one at all — would be
/// the same mission behaving differently in two windows.
struct DockOffer {
  /// The entity to dock, or 0 when there is none.
  std::uint64_t id{0};
  /// Why the nearest candidate was refused. Only meaningful when `id` is 0.
  core::DockBlock block{core::DockBlock::None};
  /// The name the message is about: the candidate on success, the nearest
  /// refusal otherwise.
  std::string name;
};

/// The nearest vessel that can dock onto `selected`, or the nearest that cannot.
///
/// Ranked by surface gap rather than by centre distance, because a station and
/// a probe equally far from touching are not equally far apart. Only entities
/// in the same frame are considered: subtracting two positions that are not in
/// one frame gives a number, and the number is not a distance.
///
/// The refusal is remembered as well as the success, so that a key that does
/// nothing can say why about the vessel being aimed at rather than about
/// whichever one happened to be scanned first.
[[nodiscard]] DockOffer nearest_dock(const proto::Snapshot& snapshot,
                                     std::uint64_t selected) noexcept;

/// The name of a body, or "?" when the id is not in the snapshot.
[[nodiscard]] std::string body_name(const proto::Snapshot& snapshot, std::uint32_t id);

}  // namespace rocketlab::hud
