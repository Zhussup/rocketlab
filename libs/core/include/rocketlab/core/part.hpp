#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace rocketlab::core {

/// What a part is for.
///
/// The kind is not decoration: it decides which of the numeric fields below
/// carry meaning, and it is what a future assembly editor groups by. A tank
/// has propellant and no thrust, an engine the other way round, and reading the
/// wrong field on the wrong kind gives a zero rather than a wrong answer.
enum class PartKind : std::uint8_t {
  CommandPod,  // carries the flight computer and the reaction wheels
  Tank,        // holds propellant for the engines below it
  Engine,      // turns that propellant into thrust
  Decoupler,   // a staging boundary, not a load-bearing part
  Payload,     // inert cargo: instruments, supplies, satellites
  Structure,   // adapters, fins, anything that only has mass
};

[[nodiscard]] const char* to_string(PartKind kind) noexcept;

/// One part, all SI.
///
/// `dry_mass` is the structure alone and `propellant` is what the part can
/// hold, so a tank is heavy twice over and an engine only once. Thrust and Isp
/// are vacuum figures: the atmosphere is not modelled yet, and pretending it is
/// would make every number here wrong in a way nobody could see.
struct Part {
  std::string name;
  PartKind kind{PartKind::Structure};
  double dry_mass{0.0};    // [kg]
  double propellant{0.0};  // [kg] capacity
  double thrust{0.0};      // [N]
  double isp{0.0};         // [s]
  double radius{0.0};      // [m]
  double length{0.0};      // [m]
  double torque{0.0};      // [N m] control authority
};

/// The parts a scenario is allowed to name, and the only place their numbers
/// live. Scenarios name a part; they never carry its mass, so a vessel's
/// performance cannot depend on who wrote the file.
class PartCatalogue {
 public:
  [[nodiscard]] static PartCatalogue stock();

  [[nodiscard]] const std::vector<Part>& parts() const noexcept { return parts_; }

  /// Null when the name is not in the catalogue.
  [[nodiscard]] const Part* find(std::string_view name) const noexcept;

  [[nodiscard]] std::size_t size() const noexcept { return parts_.size(); }

 private:
  std::vector<Part> parts_;
};

}  // namespace rocketlab::core
