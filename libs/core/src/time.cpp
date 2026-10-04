#include "rocketlab/core/time.hpp"

#include <cmath>
#include <cstdio>

namespace rocketlab::core {
namespace {

/// Julian date <-> Gregorian civil date, after Meeus, "Astronomical
/// Algorithms", ch. 7. Handles the 1582 reform so that dates before it come
/// out Julian rather than nonsense.

constexpr double kJulianReform = 2299161.0;

/// The integer Julian day number carrying J2000's noon. Kept separate from
/// the JD so that day arithmetic never has to add 0.5 to a six-digit number.
constexpr long kJulianDayNumberJ2000 = 2451545;

/// Converts an integer Julian day number plus a day fraction into a civil
/// date. `fraction` is not required to lie in [0, 1): a value of 1.5 rolls
/// over into the following day, which is what lets the Instant overload hand
/// in the noon-based offset and the seconds-of-day together.
CalendarDate civil_from_day_and_fraction(long z, double fraction) {
  long a = z;
  if (z >= kJulianReform) {
    const auto alpha = static_cast<long>(std::floor((z - 1867216.25) / 36524.25));
    a = z + 1 + alpha - alpha / 4;
  }

  const long b = a + 1524;
  const auto c = static_cast<long>(std::floor((static_cast<double>(b) - 122.1) / 365.25));
  const auto d = static_cast<long>(std::floor(365.25 * static_cast<double>(c)));
  const auto e = static_cast<long>(std::floor((static_cast<double>(b) - static_cast<double>(d)) / 30.6001));

  const double day_with_fraction =
      static_cast<double>(b) - static_cast<double>(d) -
      std::floor(30.6001 * static_cast<double>(e)) + fraction;

  CalendarDate out;
  out.day = static_cast<int>(std::floor(day_with_fraction));
  out.month = static_cast<int>(e < 14 ? e - 1 : e - 13);
  out.year = static_cast<int>(out.month > 2 ? c - 4716 : c - 4715);

  double frac = day_with_fraction - static_cast<double>(out.day);
  frac *= 24.0;
  out.hour = static_cast<int>(frac);
  frac = (frac - static_cast<double>(out.hour)) * 60.0;
  out.minute = static_cast<int>(frac);
  out.second = (frac - static_cast<double>(out.minute)) * 60.0;

  // Rounding can push seconds to exactly 60.0; keep it inside the minute.
  if (out.second >= 60.0) {
    out.second -= 60.0;
    if (++out.minute >= 60) {
      out.minute -= 60;
      if (++out.hour >= 24) {
        out.hour -= 24;
        out.day += 1;
      }
    }
  }
  return out;
}

}  // namespace

CalendarDate to_calendar(double julian_date) noexcept {
  const double shifted = julian_date + 0.5;
  const double z_int = std::floor(shifted);
  return civil_from_day_and_fraction(static_cast<long>(z_int), shifted - z_int);
}

CalendarDate to_calendar(Instant instant) noexcept {
  // A Julian day turns over at noon, so the day number is floor(t + 0.5).
  // Doing that on the small day count rather than on the six-digit JD is what
  // preserves the seconds: `total_days` is of order 1e4, where a double still
  // resolves well under a microsecond.
  const double total_days = instant.tdb / kSecondsPerDay;
  const double day_number = std::floor(total_days + 0.5);
  const double fraction = total_days + 0.5 - day_number;  // [0, 1)
  return civil_from_day_and_fraction(kJulianDayNumberJ2000 + static_cast<long>(day_number),
                                     fraction);
}

Instant from_calendar(const CalendarDate& date) noexcept {
  // Midnight's JD is an exact half-integer, and the epoch offset is exact too,
  // so the day count survives intact and only the seconds carry any rounding.
  const double midnight_jd =
      to_julian_date(CalendarDate{date.year, date.month, date.day, 0, 0, 0.0});
  const double days_from_epoch = midnight_jd - kJulianDateJ2000;
  const double seconds_of_day = static_cast<double>(date.hour) * 3600.0 +
                                static_cast<double>(date.minute) * 60.0 + date.second;
  return Instant{days_from_epoch * kSecondsPerDay + seconds_of_day};
}

double to_julian_date(const CalendarDate& date) noexcept {
  int year = date.year;
  int month = date.month;
  if (month <= 2) {
    year -= 1;
    month += 12;
  }
  const long a = year / 100;
  const long b = 2 - a + a / 4;  // Gregorian correction
  const double day = static_cast<double>(date.day) +
                     (static_cast<double>(date.hour) +
                      (static_cast<double>(date.minute) + date.second / 60.0) / 60.0) /
                         24.0;
  return std::floor(365.25 * static_cast<double>(year + 4716)) +
         std::floor(30.6001 * static_cast<double>(month + 1)) + day +
         static_cast<double>(b) - 1524.5;
}

std::string format_duration(Seconds seconds) {
  const bool negative = seconds < 0.0;
  double total = std::fabs(seconds);

  const auto days = static_cast<long>(total / kSecondsPerDay);
  total -= static_cast<double>(days) * kSecondsPerDay;
  const auto hours = static_cast<long>(total / 3600.0);
  total -= static_cast<double>(hours) * 3600.0;
  const auto minutes = static_cast<long>(total / 60.0);
  total -= static_cast<double>(minutes) * 60.0;

  char buffer[64];
  if (days > 0) {
    std::snprintf(buffer, sizeof(buffer), "%s%ldd %02ldh %02ldm %02.0fs",
                  negative ? "-" : "", days, hours, minutes, total);
  } else if (hours > 0) {
    std::snprintf(buffer, sizeof(buffer), "%s%ldh %02ldm %02.0fs",
                  negative ? "-" : "", hours, minutes, total);
  } else if (minutes > 0) {
    std::snprintf(buffer, sizeof(buffer), "%s%ldm %04.1fs",
                  negative ? "-" : "", minutes, total);
  } else {
    std::snprintf(buffer, sizeof(buffer), "%s%.1fs", negative ? "-" : "", total);
  }
  return buffer;
}

std::string format_met(Seconds tdb) {
  const bool negative = tdb < 0.0;
  double total = std::fabs(tdb);

  const auto days = static_cast<long>(total / kSecondsPerDay);
  total -= static_cast<double>(days) * kSecondsPerDay;
  const auto hours = static_cast<long>(total / 3600.0);
  total -= static_cast<double>(hours) * 3600.0;
  const auto minutes = static_cast<long>(total / 60.0);
  total -= static_cast<double>(minutes) * 60.0;

  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%s %03ld:%02ld:%02ld:%02ld",
                negative ? "T-" : "T+", days, hours, minutes,
                static_cast<long>(total));
  return buffer;
}

FrameTimer::FrameTimer() noexcept : last_(std::chrono::steady_clock::now()) {}

Seconds FrameTimer::tick(Seconds max_span) noexcept {
  const auto now = std::chrono::steady_clock::now();
  const std::chrono::duration<double> elapsed = now - last_;
  last_ = now;
  const double seconds = elapsed.count();
  return seconds < max_span ? seconds : max_span;
}

void SimClock::step_warp(int direction) noexcept {
  if (direction == 0) {
    return;
  }
  if (direction > 0) {
    // Next rung strictly above the current factor; a custom value between
    // rungs therefore snaps up to the nearest one rather than past it.
    for (const double rung : kWarpLadder) {
      if (rung > warp_) {
        warp_ = rung;
        return;
      }
    }
    warp_ = kWarpLadder.back();
    return;
  }
  for (auto it = kWarpLadder.rbegin(); it != kWarpLadder.rend(); ++it) {
    if (*it < warp_) {
      warp_ = *it;
      return;
    }
  }
  warp_ = kWarpLadder.front();
}

}  // namespace rocketlab::core
