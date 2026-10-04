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

  // Distance from the root body, which is the number that tells you whether a
  // transfer is going anywhere.
  const double root_x = entity.parent_position.x + entity.position.x;
  const double root_y = entity.parent_position.y + entity.position.y;
  const double root_z = entity.parent_position.z + entity.position.z;
  rows.push_back(
      Row{"sun range", format_length(std::sqrt(root_x * root_x + root_y * root_y + root_z * root_z)),
          false});

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

}  // namespace rocketlab::hud
