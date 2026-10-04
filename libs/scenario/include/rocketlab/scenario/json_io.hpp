#pragma once

#include <filesystem>
#include <string>
#include <string_view>

#include "rocketlab/core/scenario.hpp"

namespace rocketlab::scenario {

/// Scenario files are JSON. This header deliberately does not mention the JSON
/// library: the dependency stays inside the implementation, so nothing that
/// includes it inherits the compile cost or the coupling.
///
/// The shape is:
///
/// ```json
/// {
///   "name": "leo",
///   "epoch": "2000-01-01T12:00:00",
///   "entities": [
///     {
///       "name": "Explorer-1",
///       "parent": "Earth",
///       "orbit": {
///         "periapsis_altitude": 200000,
///         "apoapsis_altitude": 200000,
///         "inclination_deg": 51.6
///       },
///       "mass": 1000.0
///     }
///   ]
/// }
/// ```
///
/// `epoch` may be an ISO-like `YYYY-MM-DDTHH:MM:SS` string or, as `epoch_tdb`,
/// a raw TDB second count since J2000. Any orbit field may be omitted; the
/// defaults give a 200 km circular equatorial orbit.
///
/// Throws `std::runtime_error` with a message naming the offending field when
/// the document does not fit.

[[nodiscard]] core::Scenario parse_scenario(std::string_view json_text);

[[nodiscard]] core::Scenario load_scenario_file(const std::filesystem::path& path);

[[nodiscard]] std::string write_scenario(const core::Scenario& scenario);

/// A small scenario used when none is supplied, so the simulator always has
/// something to fly.
[[nodiscard]] core::Scenario default_scenario();

}  // namespace rocketlab::scenario
