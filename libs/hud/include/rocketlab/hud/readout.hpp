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

#include <string>
#include <string_view>
#include <vector>

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
/// Radians to a degrees string.
[[nodiscard]] std::string format_angle(double radians);
/// A duration in the mission-elapsed-time form, or "open" for an unbound orbit.
[[nodiscard]] std::string format_period(double seconds);

/// The parent body of an entity, or null when the snapshot does not contain
/// it. Needed because altitude is measured from the parent's surface, and the
/// body table is the only place the radius lives.
[[nodiscard]] const proto::BodySnapshot* find_body(const proto::Snapshot& snapshot,
                                                   std::uint32_t id) noexcept;

/// The entity with the given id, or null.
[[nodiscard]] const proto::EntitySnapshot* find_entity(const proto::Snapshot& snapshot,
                                                       std::uint64_t id) noexcept;

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

/// The name of a body, or "?" when the id is not in the snapshot.
[[nodiscard]] std::string body_name(const proto::Snapshot& snapshot, std::uint32_t id);

}  // namespace rocketlab::hud
