#pragma once

#include <string>
#include <vector>

#include "rocketlab/core/body.hpp"
#include "rocketlab/core/time.hpp"

namespace rocketlab::core {

/// One entity as written down in a scenario file.
///
/// The orbital shape is given as altitudes above the parent's surface rather
/// than as elements, because "200 km circular orbit" is what a person means and
/// element-space is where the confusing numbers live.
struct ScenarioEntity {
  std::string name{"unnamed"};
  std::string parent_body{"Earth"};
  double periapsis_altitude{200e3};      // [m]
  double apoapsis_altitude{200e3};       // [m]
  double inclination_deg{0.0};
  double raan_deg{0.0};
  double argp_deg{0.0};
  double true_anomaly_deg{0.0};
  double mass{1000.0};  // [kg]
  double radius{1.0};   // [m]
  bool controllable{true};
};

struct Scenario {
  std::string name{"untitled"};
  Instant epoch{};
  std::vector<ScenarioEntity> entities;
};

/// Converts altitudes and angles into a state relative to `body`.
///
/// Equal altitudes give a circular orbit; unequal ones give an ellipse whose
/// line of apsides is set by `argp_deg`. Throws `std::invalid_argument` for a
/// negative altitude or an apoapsis below the periapsis.
[[nodiscard]] StateVector state_from_altitudes(const CelestialBody& body, double periapsis_altitude,
                                               double apoapsis_altitude, double inclination_deg,
                                               double raan_deg, double argp_deg,
                                               double true_anomaly_deg);

}  // namespace rocketlab::core
