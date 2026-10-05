#include "rocketlab/hud/assembly.hpp"

#include <cmath>
#include <format>
#include <utility>

namespace rocketlab::hud {

std::vector<StackLine> stack_lines(const core::Vessel& vessel) {
  std::vector<StackLine> lines;
  lines.reserve(vessel.parts.size());
  for (std::size_t i = 0; i < vessel.parts.size(); ++i) {
    const core::Part& part = vessel.parts[i];
    StackLine line;
    line.part = part.name;
    line.kind = core::to_string(part.kind);
    line.stage = i < vessel.stage.size() ? vessel.stage[i] : 0;
    line.dry_mass = part.dry_mass;
    line.propellant = i < vessel.propellant.size() ? vessel.propellant[i] : 0.0;
    lines.push_back(std::move(line));
  }
  return lines;
}

std::vector<PartLine> part_lines(const core::PartCatalogue& catalogue) {
  std::vector<PartLine> lines;
  lines.reserve(catalogue.size());
  for (const core::Part& part : catalogue.parts()) {
    PartLine line;
    line.name = part.name;
    line.kind = core::to_string(part.kind);
    line.mass = format_mass(part.dry_mass);

    // Only the field that carries meaning for this kind of part, because a tank
    // has no thrust and an engine has no capacity and printing a zero for either
    // would make the picker read as a table of broken parts.
    switch (part.kind) {
      case core::PartKind::Engine:
        line.detail = std::format("{}, {:g} s", format_thrust(part.thrust), part.isp);
        break;
      case core::PartKind::Tank:
        line.detail = format_mass(part.propellant);
        break;
      default:
        break;
    }
    lines.push_back(std::move(line));
  }
  return lines;
}

std::vector<Row> stack_summary(const core::Vessel& vessel) {
  std::vector<Row> rows;
  rows.reserve(7);

  const double propellant = vessel.propellant_capacity();
  const double wet = vessel.mass();

  rows.push_back(Row{"parts", std::to_string(vessel.parts.size()), false});
  rows.push_back(Row{"stages", std::to_string(vessel.stage_count), false});
  // Structure and capacity separately, because "how much of this is fuel" is the
  // question a person building a rocket is actually asking.
  rows.push_back(Row{"dry mass", format_mass(wet - propellant), false});
  rows.push_back(Row{"propellant", format_mass(propellant), false});
  rows.push_back(Row{"mass", format_mass(wet), false});
  rows.push_back(Row{"length", format_length(vessel.length()), false});
  rows.push_back(Row{"delta-v", format_speed(core::total_delta_v(vessel)), false});
  return rows;
}

std::vector<Row> stage_rows(const core::Vessel& vessel) {
  std::vector<Row> rows;

  for (const core::StageReport& report : core::stage_table(vessel)) {
    Row row;
    row.label = std::format("stage {}", report.index);

    if (report.thrust > 0.0 && report.propellant > 0.0) {
      row.value = std::format("{}  {}", format_thrust(report.thrust),
                              format_speed(report.delta_v));
      if (std::isfinite(report.burn_time) && report.burn_time > 0.0) {
        row.value += " in " + format_period(report.burn_time);
      }
    } else if (report.thrust > 0.0) {
      // An engine and nothing to feed it. The fuel is somewhere else in the
      // stack, in a stage that will already have been dropped by the time this
      // one lights.
      row.value = "no propellant";
      row.emphasised = true;
    } else if (report.propellant > 0.0) {
      // Tanks and nothing to burn them in. Legitimate — it is a coast — and
      // named rather than flagged, because a person who meant it should not be
      // told they made a mistake.
      row.value = std::format("coast, {}", format_mass(report.propellant));
    } else {
      row.value = "empty";
    }
    rows.push_back(std::move(row));
  }
  return rows;
}

}  // namespace rocketlab::hud
