#include "rocketlab/hud/readout.hpp"

#include <cmath>
#include <format>
#include <utility>

namespace rocketlab::hud {

namespace {

constexpr double kRadiansToDegrees = 57.29577951308232;

/// Three significant figures, never scientific notation. A telemetry panel is
/// read at a glance, and "4.31e+08 m" is not read at a glance.
[[nodiscard]] std::string compact(double value, std::string_view unit, double scale) {
  const double scaled = value * scale;
  const double magnitude = std::abs(scaled);
  int decimals = 2;
  if (magnitude >= 1000.0) {
    decimals = 0;
  } else if (magnitude >= 100.0) {
    decimals = 1;
  }
  return std::format("{:.{}f} {}", scaled, decimals, unit);
}

/// An instruction count with its digits grouped, because a flight computer's
/// budget is read as "how close to the limit is it", and 1000000 has to be
/// counted digit by digit where 1 000 000 does not. Locale-free on purpose: a
/// telemetry panel that changed its separators with the environment would be a
/// different panel on every machine.
[[nodiscard]] std::string format_count(double value) {
  const long long total = static_cast<long long>(value + 0.5);
  const std::string digits = std::to_string(total < 0 ? -total : total);
  std::string out;
  out.reserve(digits.size() + digits.size() / 3 + 1);
  for (std::size_t i = 0; i < digits.size(); ++i) {
    if (i > 0 && (digits.size() - i) % 3 == 0) {
      out.push_back(' ');
    }
    out.push_back(digits[i]);
  }
  return total < 0 ? "-" + out : out;
}

}  // namespace

std::string format_length(double metres) {
  const double magnitude = std::abs(metres);
  if (magnitude >= 1.0e12) {
    return compact(metres, "Tm", 1.0e-12);
  }
  if (magnitude >= 1.0e9) {
    return compact(metres, "Gm", 1.0e-9);
  }
  if (magnitude >= 1.0e6) {
    return compact(metres, "Mm", 1.0e-6);
  }
  if (magnitude >= 1.0e3) {
    return compact(metres, "km", 1.0e-3);
  }
  return compact(metres, "m", 1.0);
}

std::string format_speed(double metres_per_second) {
  if (std::abs(metres_per_second) >= 1000.0) {
    return compact(metres_per_second, "km/s", 1.0e-3);
  }
  return compact(metres_per_second, "m/s", 1.0);
}

std::string format_mass(double kilograms) {
  if (std::abs(kilograms) >= 1000.0) {
    return compact(kilograms, "t", 1.0e-3);
  }
  return compact(kilograms, "kg", 1.0);
}

std::string format_thrust(double newtons) {
  if (std::abs(newtons) >= 1.0e6) {
    return compact(newtons, "MN", 1.0e-6);
  }
  return compact(newtons, "kN", 1.0e-3);
}

std::string format_ratio(double value) {
  return std::format("{:.2f}", value);
}

std::string format_angle(double radians) {
  return std::format("{:.2f}°", radians * kRadiansToDegrees);
}

std::string format_period(double seconds) {
  if (!(seconds > 0.0) || seconds > 1.0e12) {
    return "open";
  }
  const long total = static_cast<long>(seconds);
  const long days = total / 86400;
  const long hours = (total % 86400) / 3600;
  const long minutes = (total % 3600) / 60;
  const long secs = total % 60;
  if (days > 0) {
    return std::format("{}d {:02}h {:02}m", days, hours, minutes);
  }
  if (hours > 0) {
    return std::format("{}h {:02}m {:02}s", hours, minutes, secs);
  }
  if (minutes > 0) {
    return std::format("{}m {:02}s", minutes, secs);
  }
  return std::format("{}s", secs);
}

std::string format_density(double kilograms_per_cubic_metre) {
  if (!(kilograms_per_cubic_metre > 0.0)) {
    return "vacuum";
  }
  return std::format("{:.2e} kg/m³", kilograms_per_cubic_metre);
}

const proto::BodySnapshot* find_body(const proto::Snapshot& snapshot, std::uint32_t id) noexcept {
  for (std::uint32_t i = 0; i < snapshot.body_count && i < proto::kMaxBodies; ++i) {
    if (snapshot.bodies[i].id == id) {
      return &snapshot.bodies[i];
    }
  }
  return nullptr;
}

const proto::EntitySnapshot* find_entity(const proto::Snapshot& snapshot,
                                         std::uint64_t id) noexcept {
  for (std::uint32_t i = 0; i < snapshot.entity_count && i < proto::kMaxEntities; ++i) {
    if (snapshot.entities[i].id == id) {
      return &snapshot.entities[i];
    }
  }
  return nullptr;
}

const char* computer_state_name(proto::ComputerState state) noexcept {
  switch (state) {
    case proto::ComputerState::None:
      return "none";
    case proto::ComputerState::Idle:
      return "idle";
    case proto::ComputerState::Running:
      return "running";
    case proto::ComputerState::Finished:
      return "finished";
    case proto::ComputerState::Faulted:
      return "faulted";
  }
  return "none";
}

std::string body_name(const proto::Snapshot& snapshot, std::uint32_t id) {
  const proto::BodySnapshot* body = find_body(snapshot, id);
  return body != nullptr ? std::string(body->name.view()) : std::string("?");
}

std::vector<Row> entity_readout(const proto::Snapshot& snapshot,
                                const proto::EntitySnapshot& entity) {
  std::vector<Row> rows;
  rows.reserve(14);

  const proto::BodySnapshot* parent = find_body(snapshot, entity.parent);

  rows.push_back(Row{"parent", parent != nullptr ? std::string(parent->name.view()) : "?",
                     false});
  rows.push_back(Row{"kind",
                     entity.kind == proto::Kind::Debris ? "debris" : "vessel", false});
  rows.push_back(Row{"mass", format_mass(entity.mass), false});
  rows.push_back(Row{"radius", format_length(entity.radius), false});

  const bool degenerate = proto::has_flag(entity.flags, proto::Flags::Degenerate);
  const bool escaping = proto::has_flag(entity.flags, proto::Flags::Escaping);

  if (degenerate) {
    // The element set is meaningless for a rectilinear orbit, so printing
    // zeroes would be a lie dressed up as data.
    rows.push_back(Row{"orbit", "degenerate", true});
  } else {
    const double surface = parent != nullptr ? parent->radius : 0.0;
    rows.push_back(Row{"periapsis", format_length(entity.periapsis - surface), false});
    rows.push_back(Row{"apoapsis",
                       escaping ? "escape" : format_length(entity.apoapsis - surface), false});
    rows.push_back(Row{"eccentricity", std::format("{:.4f}", entity.eccentricity), false});
    rows.push_back(Row{"inclination", format_angle(entity.inclination), false});
    rows.push_back(Row{"period", escaping ? "open" : format_period(entity.period), escaping});
  }

  // Instantaneous quantities, straight off the state vector.
  const double speed = std::sqrt(entity.velocity.x * entity.velocity.x +
                                 entity.velocity.y * entity.velocity.y +
                                 entity.velocity.z * entity.velocity.z);
  const double radius = std::sqrt(entity.position.x * entity.position.x +
                                  entity.position.y * entity.position.y +
                                  entity.position.z * entity.position.z);
  rows.push_back(Row{"speed", format_speed(speed), false});
  if (parent != nullptr) {
    rows.push_back(Row{"altitude", format_length(radius - parent->radius),
                       radius < parent->radius});
  }

  // Only worth a row when there is air: "vacuum" on every entity in the
  // catalogue would be eleven lines of nothing, and the row would stop being
  // read on the one vessel where it matters.
  if (entity.air_density > 0.0) {
    rows.push_back(Row{"air", format_density(entity.air_density), false});
  }

  // Distance from the root body, which is the number that tells you whether a
  // transfer is going anywhere.
  const double root_x = entity.parent_position.x + entity.position.x;
  const double root_y = entity.parent_position.y + entity.position.y;
  const double root_z = entity.parent_position.z + entity.position.z;
  rows.push_back(
      Row{"sun range", format_length(std::sqrt(root_x * root_x + root_y * root_y + root_z * root_z)),
          false});

  // Propulsion, for an entity that has parts. A point mass has nothing to say
  // here, and printing a row of zeroes would suggest it did.
  if (entity.stage_count > 0) {
    rows.push_back(Row{"stage", std::format("{} of {}", entity.stage + 1, entity.stage_count),
                       false});
    rows.push_back(Row{"throttle",
                       entity.throttle > 0.0 ? std::format("{:.0f}%", entity.throttle * 100.0)
                                            : std::string("off"),
                       false});
    rows.push_back(Row{"thrust", format_thrust(entity.thrust), false});

    const double capacity = entity.propellant_capacity;
    rows.push_back(Row{"propellant",
                       capacity > 0.0
                           ? std::format("{} of {}", format_mass(entity.propellant),
                                         format_mass(capacity))
                           : std::string("none"),
                       capacity > 0.0 && entity.propellant <= 0.0});
    rows.push_back(Row{"delta-v", format_speed(entity.delta_v), false});

    // Thrust-to-weight where the vessel actually is. Local gravity is mu over r
    // squared, which is arithmetic on the snapshot and not a prediction; it is
    // the number that says whether a stage can hold its own weight at all.
    if (parent != nullptr && radius > 0.0 && entity.mass > 0.0) {
      const double gravity = parent->mu / (radius * radius);
      rows.push_back(Row{"twr", format_ratio(entity.thrust / (entity.mass * gravity)), false});
    }
  }

  // The flight computer, when one is attached. A fault is the one thing on this
  // panel that must not be missed, so it is emphasised and it is the only place
  // the script's own words reach the screen.
  if (entity.computer != proto::ComputerState::None) {
    const bool faulted = entity.computer == proto::ComputerState::Faulted;
    rows.push_back(Row{"computer", computer_state_name(entity.computer), faulted});
    rows.push_back(Row{"instructions", format_count(entity.computer_instructions), false});

    const std::string_view message = entity.computer_message.view();
    if (!message.empty()) {
      rows.push_back(Row{"script", std::string(message), faulted});
    }
  }

  return rows;
}

std::vector<EntityLine> entity_list(const proto::Snapshot& snapshot) {
  std::vector<EntityLine> lines;
  lines.reserve(snapshot.entity_count);
  for (std::uint32_t i = 0; i < snapshot.entity_count && i < proto::kMaxEntities; ++i) {
    const proto::EntitySnapshot& entity = snapshot.entities[i];
    EntityLine line;
    line.id = entity.id;
    line.name = std::string(entity.name.view());
    line.parent = body_name(snapshot, entity.parent);
    line.selected = entity.id == snapshot.selected;
    line.controllable = proto::has_flag(entity.flags, proto::Flags::Controllable);
    lines.push_back(std::move(line));
  }
  return lines;
}

core::DockingGeometry dock_geometry(const proto::EntitySnapshot& target,
                                    const proto::EntitySnapshot& absorbed) noexcept {
  return core::docking_geometry(
      core::StateVector{{target.position.x, target.position.y, target.position.z},
                        {target.velocity.x, target.velocity.y, target.velocity.z}},
      core::StateVector{{absorbed.position.x, absorbed.position.y, absorbed.position.z},
                        {absorbed.velocity.x, absorbed.velocity.y, absorbed.velocity.z}},
      target.radius, absorbed.radius);
}

core::DockBlock dock_block(const proto::EntitySnapshot& target,
                           const proto::EntitySnapshot& absorbed,
                           const core::DockingLimits& limits) noexcept {
  // The three structural rules, in the order `core::dock_block` gives them.
  if (target.id == absorbed.id) return core::DockBlock::SameEntity;
  if (target.stage_count == 0 || absorbed.stage_count == 0) return core::DockBlock::NoParts;
  if (target.parent != absorbed.parent) return core::DockBlock::DifferentParent;
  return core::dock_block(dock_geometry(target, absorbed), limits);
}

DockOffer nearest_dock(const proto::Snapshot& snapshot, std::uint64_t selected) noexcept {
  const proto::EntitySnapshot* from = find_entity(snapshot, selected);
  if (from == nullptr) {
    return DockOffer{0, core::DockBlock::SameEntity, {}};
  }
  if (from->stage_count == 0) {
    return DockOffer{0, core::DockBlock::NoParts, std::string(from->name.view())};
  }

  const proto::EntitySnapshot* candidate = nullptr;
  double candidate_gap = 0.0;
  const proto::EntitySnapshot* refused = nullptr;
  double refused_gap = 0.0;
  core::DockBlock refused_block = core::DockBlock::TooFar;

  for (std::uint32_t i = 0; i < snapshot.entity_count && i < proto::kMaxEntities; ++i) {
    const proto::EntitySnapshot& other = snapshot.entities[i];
    if (other.id == from->id || other.parent != from->parent) {
      continue;
    }
    const core::DockBlock block = dock_block(*from, other);
    const double gap = dock_geometry(*from, other).separation;
    if (block == core::DockBlock::None) {
      if (candidate == nullptr || gap < candidate_gap) {
        candidate = &other;
        candidate_gap = gap;
      }
    } else if (refused == nullptr || gap < refused_gap) {
      refused = &other;
      refused_gap = gap;
      refused_block = block;
    }
  }

  if (candidate != nullptr) {
    return DockOffer{candidate->id, core::DockBlock::None, std::string(candidate->name.view())};
  }
  if (refused != nullptr) {
    return DockOffer{0, refused_block, std::string(refused->name.view())};
  }
  return DockOffer{0, core::DockBlock::TooFar, {}};
}

}  // namespace rocketlab::hud
