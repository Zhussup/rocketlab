#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "rocketlab/core/time.hpp"

using namespace rocketlab::core;
using Catch::Matchers::WithinAbs;

TEST_CASE("the J2000 epoch converts to the expected civil date", "[time]") {
  const CalendarDate date = to_calendar(kJulianDateJ2000);
  CHECK(date.year == 2000);
  CHECK(date.month == 1);
  CHECK(date.day == 1);
  CHECK(date.hour == 12);
  CHECK(date.minute == 0);
  CHECK_THAT(date.second, WithinAbs(0.0, 1e-6));

  CHECK_THAT(to_julian_date(CalendarDate{2000, 1, 1, 12, 0, 0.0}),
             WithinAbs(kJulianDateJ2000, 1e-9));
}

TEST_CASE("well-known Julian dates land on the right day", "[time]") {
  SECTION("the Unix epoch") {
    const CalendarDate date = to_calendar(2440587.5);
    CHECK(date.year == 1970);
    CHECK(date.month == 1);
    CHECK(date.day == 1);
    CHECK(date.hour == 0);
  }

  SECTION("midnight before J2000 noon") {
    const CalendarDate date = to_calendar(kJulianDateJ2000 - 0.5);
    CHECK(date.year == 2000);
    CHECK(date.month == 1);
    CHECK(date.day == 1);
    CHECK(date.hour == 0);
  }
}

TEST_CASE("calendar conversion round-trips", "[time]") {
  const CalendarDate dates[] = {
      {2000, 1, 1, 12, 0, 0.0},   {2026, 10, 5, 0, 30, 15.5},
      {1969, 7, 20, 20, 17, 40.0}, {2035, 12, 31, 23, 59, 59.0},
      {2000, 2, 29, 6, 0, 0.0},   {2100, 3, 1, 0, 0, 0.0},
  };

  for (const CalendarDate& original : dates) {
    // The Instant path, not a raw JD: a double JD cannot hold sub-50us
    // resolution at present-day magnitudes, and the simulation never uses one.
    const CalendarDate back = to_calendar(from_calendar(original));
    INFO(original.year << "-" << original.month << "-" << original.day);
    CHECK(back.year == original.year);
    CHECK(back.month == original.month);
    CHECK(back.day == original.day);
    CHECK(back.hour == original.hour);
    CHECK(back.minute == original.minute);
    // A TDB second count near the year 2100 is ~3.2e9, where a double's ulp is
    // about 0.5us. Sub-microsecond agreement is the best the representation
    // allows, and is many orders of magnitude finer than the simulation needs.
    CHECK_THAT(back.second, WithinAbs(original.second, 1e-5));
  }
}

TEST_CASE("the instant and raw-JD paths agree where the JD is exact", "[time]") {
  // Noon is exactly representable as a JD, so this is the band where both
  // conversions must produce identical civil dates.
  for (double day : {0.0, 1.0, -1.0, 100.0, 36583.0}) {
    const Instant noon{day * kSecondsPerDay};
    const CalendarDate from_instant = to_calendar(noon);
    const CalendarDate from_jd = to_calendar(noon.julian_date());

    INFO("day offset " << day);
    CHECK(from_instant.year == from_jd.year);
    CHECK(from_instant.month == from_jd.month);
    CHECK(from_instant.day == from_jd.day);
    CHECK(from_instant.hour == from_jd.hour);
    CHECK(from_instant.minute == from_jd.minute);
  }
}

TEST_CASE("instants report the Julian date of their epoch offset", "[time]") {
  const Instant epoch{};
  CHECK_THAT(epoch.julian_date(), WithinAbs(kJulianDateJ2000, 1e-12));

  const Instant later{kSecondsPerDay};
  CHECK_THAT(later.julian_date(), WithinAbs(kJulianDateJ2000 + 1.0, 1e-12));
}

TEST_CASE("mission elapsed time is formatted the way telemetry reads", "[time]") {
  CHECK(format_met(0.0) == "T+ 000:00:00:00");
  CHECK(format_met(3 * 86400.0 + 4 * 3600.0 + 12 * 60.0 + 7.0) == "T+ 003:04:12:07");
  CHECK(format_met(-65.0) == "T- 000:00:01:05");
}

TEST_CASE("durations are formatted at a sensible precision", "[time]") {
  CHECK(format_duration(0.5) == "0.5s");
  CHECK(format_duration(61.0) == "1m 01.0s");
  CHECK(format_duration(3661.0) == "1h 01m 01s");
  CHECK(format_duration(90061.0) == "1d 01h 01m 01s");
}

TEST_CASE("the warp ladder steps up and down without overshooting", "[time]") {
  SimClock clock;
  CHECK(clock.warp() == 1.0);

  clock.step_warp(1);
  CHECK(clock.warp() == 5.0);
  clock.step_warp(1);
  CHECK(clock.warp() == 10.0);

  clock.step_warp(-1);
  CHECK(clock.warp() == 5.0);

  // A custom factor between rungs snaps to the neighbouring rung rather than
  // skipping past it.
  clock.set_warp(25.0);
  clock.step_warp(1);
  CHECK(clock.warp() == 50.0);
  clock.set_warp(25.0);
  clock.step_warp(-1);
  CHECK(clock.warp() == 10.0);

  // The ends of the ladder hold.
  clock.set_warp(kWarpLadder.back());
  clock.step_warp(1);
  CHECK(clock.warp() == kWarpLadder.back());
}

TEST_CASE("frame spans scale wall time by the warp factor", "[time]") {
  SimClock clock;
  CHECK(clock.frame_span(1.0 / 60.0) == 1.0 / 60.0);

  clock.set_warp(1000.0);
  CHECK(clock.frame_span(1.0 / 60.0) == 1000.0 / 60.0);

  // A stalled frame must not advance time backwards.
  CHECK(clock.frame_span(-1.0) == 0.0);
  CHECK(clock.frame_span(0.0) == 0.0);
}
