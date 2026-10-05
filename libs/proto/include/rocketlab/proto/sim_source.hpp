// The seam between the simulation and every client.
//
// Rule 5 of the project says clients never do physics. This interface is how
// that rule is enforced rather than merely intended: a client is handed a
// SimSource and can reach the world only through a snapshot and a query. Both
// a local source (a World in this process) and a remote one (shared memory and
// a socket) satisfy it, so a client cannot tell them apart, and the daemon
// split cannot change client behaviour.
//
// The local source exists because it makes the client layer testable without
// spawning a process, not as a shortcut around the boundary.

#pragma once

#include <cstdint>
#include <string_view>
#include <vector>

#include "rocketlab/proto/command.hpp"
#include "rocketlab/proto/snapshot.hpp"

namespace rocketlab::proto {

class SimSource {
 public:
  SimSource() = default;
  SimSource(const SimSource&) = delete;
  SimSource& operator=(const SimSource&) = delete;
  /// Movable, because a host is naturally constructed in one place and owned
  /// in another. Copying stays forbidden: two owners of one simulation is
  /// exactly the mistake the interface exists to prevent.
  SimSource(SimSource&&) = default;
  SimSource& operator=(SimSource&&) = default;
  virtual ~SimSource() = default;

  /// The most recent frame. Never null and always valid, even before the
  /// first pump: a source publishes an initial snapshot when it is built.
  [[nodiscard]] virtual const Snapshot& snapshot() const = 0;

  /// Advances the simulation by `wall_dt` seconds of real time, applying the
  /// warp factor in effect, and publishes a new frame.
  ///
  /// A `wall_dt` of zero or less means "measure the frame yourself"; the local
  /// host then uses its own FrameTimer. Either way this is the only place a
  /// client hands over the wall clock, and it hands over a duration, never a
  /// timestamp.
  virtual void pump(double wall_dt) = 0;

  /// Queues a command. Applied on the next pump, so a client never blocks on
  /// the daemon.
  virtual void send(const Command& command) = 0;

  /// Fills `out` with the entity's predicted path, sampled over `horizon`
  /// seconds from now, and returns false if the entity is unknown.
  ///
  /// The points are in the entity's **parent body frame**, matching how the
  /// entity's own position is stored. This matters more than it looks: in the
  /// root frame a low Earth orbit is dominated by the Earth's own 30 km/s
  /// travel, so a full orbit drawn in root coordinates smears across an
  /// eighth of a billion kilometres and reads as nonsense. Relative to the
  /// parent, the same orbit is the ellipse it actually is. A client composes
  /// the root form by adding the parent position it already has.
  ///
  /// This is the query half of rule 5: working out where a vessel will be in
  /// an hour is a physics problem, so the client asks rather than computes.
  /// `out` is cleared first. The path is exact, including any sphere-of-
  /// influence crossing the prediction passes through — the frame stays
  /// anchored to the body the entity started around.
  virtual bool query_trajectory(std::uint64_t id, double horizon,
                                std::vector<Vec3d>& out) const = 0;

  /// Human-readable description of where the state came from, for the client
  /// to display. Never empty.
  [[nodiscard]] virtual std::string_view describe() const = 0;
};

}  // namespace rocketlab::proto
