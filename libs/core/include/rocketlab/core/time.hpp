#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <string>

namespace rocketlab::core {

using Seconds = double;

/// The simulation epoch is J2000 = 2000-01-01 12:00:00 TT, JD 2451545.0.
/// All simulation time is expressed as TDB seconds since that instant.
inline constexpr double kJulianDateJ2000 = 2451545.0;
inline constexpr double kSecondsPerDay = 86400.0;

/// Simulation time, TDB seconds since J2000.
///
/// Simulation code never reads the wall clock. Everything that needs "now"
/// takes an `Instant`, which is what makes time warp, replay and headless
/// regression tests possible at all.
struct Instant {
  Seconds tdb{0.0};

  [[nodiscard]] constexpr double julian_date() const noexcept {
    return kJulianDateJ2000 + tdb / kSecondsPerDay;
  }

  friend constexpr bool operator==(Instant, Instant) = default;
};

struct CalendarDate {
  int year{2000};
  int month{1};
  int day{1};
  int hour{12};
  int minute{0};
  double second{0.0};
};

/// Civil date for a raw Julian date.
///
/// A double JD near the present day resolves to only about 50 microseconds,
/// because the value itself is ~2.46e6 and the fraction has to share the
/// mantissa with it. Use the `Instant` overload below for anything that came
/// out of the simulation.
[[nodiscard]] CalendarDate to_calendar(double julian_date) noexcept;

/// Civil date for a simulation instant.
///
/// Splits the whole days from the seconds-of-day *before* converting, so the
/// fractional day never has to share a mantissa with a six-digit day number.
/// Roughly four orders of magnitude more precise than the raw-JD overload.
[[nodiscard]] CalendarDate to_calendar(Instant instant) noexcept;

[[nodiscard]] double to_julian_date(const CalendarDate& date) noexcept;

/// Inverse of `to_calendar(Instant)`. Builds the instant from a whole-day
/// count plus seconds-of-day, for the same precision reason.
[[nodiscard]] Instant from_calendar(const CalendarDate& date) noexcept;

/// Formats TDB seconds since J2000 as the usual mission-elapsed-time stamp,
/// e.g. `T+ 003:04:12:07` for three days, four hours, twelve minutes.
[[nodiscard]] std::string format_met(Seconds tdb);

/// Formats a duration compactly for telemetry readouts, e.g. `1h 04m 12s`.
[[nodiscard]] std::string format_duration(Seconds seconds);

/// Wall-clock timer. The only place in the codebase allowed to read real
/// time; it exists so the UI can convert a frame's real duration into a
/// simulation-time span via `SimClock::frame_span`.
class FrameTimer {
 public:
  FrameTimer() noexcept;

  /// Seconds since the previous call to `tick()`. Clamped to `max_span` so a
  /// debugger breakpoint or a suspended laptop does not fast-forward the
  /// whole mission on the next frame.
  Seconds tick(Seconds max_span = 0.25) noexcept;

 private:
  std::chrono::steady_clock::time_point last_;
};

/// Discrete warp factors, mirroring the familiar ladder from 1x to 100000x.
inline constexpr std::array<double, 8> kWarpLadder{1.0, 5.0, 10.0, 50.0, 100.0, 1000.0, 10000.0, 100000.0};

/// Advances simulation time. Deliberately knows nothing about physics: the
/// propagator decides how finely a span must be subdivided, because only it
/// knows where the events (sphere-of-influence crossings, burns) are.
class SimClock {
 public:
  SimClock() noexcept = default;

  [[nodiscard]] Instant now() const noexcept { return Instant{tdb_}; }
  [[nodiscard]] Seconds tdb() const noexcept { return tdb_; }
  [[nodiscard]] double warp() const noexcept { return warp_; }

  void set_tdb(Seconds tdb) noexcept { tdb_ = tdb; }

  void set_warp(double warp) noexcept { warp_ = warp > 0.0 ? warp : 0.0; }

  /// Moves up or down `kWarpLadder`. `direction` must be -1, 0 or +1.
  void step_warp(int direction) noexcept;

  void advance(Seconds sim_dt) noexcept { tdb_ += sim_dt; }

  /// Simulation time covered by one rendered frame, given how long that frame
  /// really took. This is the only bridge between wall time and sim time.
  [[nodiscard]] Seconds frame_span(Seconds wall_dt) const noexcept {
    return wall_dt > 0.0 ? wall_dt * warp_ : 0.0;
  }

 private:
  Seconds tdb_{0.0};
  double warp_{1.0};
};

}  // namespace rocketlab::core
