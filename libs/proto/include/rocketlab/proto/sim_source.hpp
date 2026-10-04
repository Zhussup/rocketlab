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
  virtual ~SimSource() = default;

  /// The most recent frame. Never null and always valid, even before the
  /// first pump: a source publishes an initial snapshot when it is built.
  [[nodiscard]] virtual const Snapshot& snapshot() const = 0;

  /// Advances the simulation by `wall_dt` seconds of real time, applying the
  /// warp factor in effect. This is the only place a client hands over the
  /// wall clock, and it hands over a duration, never a timestamp.
  virtual void pump(double wall_dt) = 0;

  /// Queues a command. Applied on the next pump, so a client never blocks on
  /// the daemon.
  virtual void send(const Command& command) = 0;

  /// Fills `out` with the entity's predicted path in the root frame, sampled
  /// over `horizon` seconds from now, and returns false if the entity is
  /// unknown.
  ///
  /// This is the query half of rule 5: working out where a vessel will be in
  /// an hour is a physics problem, so the client asks rather than computes.
  /// `out` is cleared first; a degenerate or escaping orbit still returns
  /// true, with whatever points are meaningful.
  virtual bool query_trajectory(std::uint64_t id, double horizon,
                                std::vector<Vec3d>& out) const = 0;

  /// Human-readable description of where the state came from, for the client
  /// to display. Never empty.
  [[nodiscard]] virtual std::string_view describe() const = 0;
};

}  // namespace rocketlab::proto
