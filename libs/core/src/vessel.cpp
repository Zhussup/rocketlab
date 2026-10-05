#include "rocketlab/core/vessel.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "rocketlab/core/constants.hpp"

namespace rocketlab::core {
namespace {

/// The stage table for a vessel, with `full` deciding whether the tanks are
/// taken at capacity or at whatever is actually in them.
///
/// One implementation for both, because the pre-launch budget and the
/// "what have I got left" figure must agree: a second copy is how the two
/// quietly start disagreeing.
[[nodiscard]] std::vector<StageReport> analyse(const Vessel& vessel, bool full) {
  std::vector<StageReport> table;
  if (vessel.parts.empty()) {
    return table;
  }

  const auto propellant_of = [&vessel, full](std::size_t i) {
    return full ? vessel.parts[i].propellant : vessel.propellant[i];
  };

  for (int s = 0; s < vessel.stage_count; ++s) {
    StageReport report;
    report.index = s;

    double thrust = 0.0;
    double thrust_over_isp = 0.0;
    for (std::size_t i = 0; i < vessel.parts.size(); ++i) {
      if (vessel.stage[i] != s) {
        continue;
      }
      const Part& part = vessel.parts[i];
      report.dry_mass += part.dry_mass;
      if (part.kind == PartKind::Engine && part.isp > 0.0) {
        thrust += part.thrust;
        // Summing T/Isp rather than averaging Isp is what makes two different
        // engines behave like one of the same total impulse: the mass flows add,
        // and 1/Isp is proportional to mass flow per unit thrust.
        thrust_over_isp += part.thrust / part.isp;
      }
      if (part.kind == PartKind::Tank) {
        report.propellant += propellant_of(i);
      }
    }
    report.thrust = thrust;
    report.isp = thrust_over_isp > 0.0 ? thrust / thrust_over_isp : 0.0;

    // Everything still attached when this stage lights: its own parts plus all
    // the later stages that have not been dropped yet.
    double wet = 0.0;
    for (std::size_t i = 0; i < vessel.parts.size(); ++i) {
      if (vessel.stage[i] < s) {
        continue;
      }
      wet += vessel.parts[i].dry_mass + propellant_of(i);
    }
    report.ignition_mass = wet;
    report.final_mass = wet - report.propellant;

    const double exhaust = report.isp * kStandardGravity;
    if (report.propellant > 0.0 && report.final_mass > 0.0 &&
        report.ignition_mass > report.final_mass) {
      report.delta_v = exhaust * std::log(report.ignition_mass / report.final_mass);
      if (thrust > 0.0 && exhaust > 0.0) {
        // Propellant divided by mass flow, and mass flow is thrust over exhaust
        // velocity. A stage with no engine cannot burn, and says so by leaving
        // the burn time at infinity rather than at a number.
        report.burn_time = report.propellant / (thrust / exhaust);
      }
    }
    table.push_back(report);
  }
  return table;
}

}  // namespace

double JettisonedStage::mass() const noexcept {
  double total = propellant;
  for (const Part& part : parts) {
    total += part.dry_mass;
  }
  return total;
}

double JettisonedStage::radius() const noexcept {
  double widest = 0.0;
  for (const Part& part : parts) {
    widest = std::max(widest, part.radius);
  }
  return widest;
}

double Vessel::mass() const noexcept {
  double total = 0.0;
  for (std::size_t i = 0; i < parts.size(); ++i) {
    total += parts[i].dry_mass + propellant[i];
  }
  return total;
}

double Vessel::propellant_left() const noexcept {
  double total = 0.0;
  for (std::size_t i = 0; i < parts.size(); ++i) {
    if (parts[i].kind == PartKind::Tank) {
      total += propellant[i];
    }
  }
  return total;
}

double Vessel::propellant_capacity() const noexcept {
  double total = 0.0;
  for (const Part& part : parts) {
    if (part.kind == PartKind::Tank) {
      total += part.propellant;
    }
  }
  return total;
}

double Vessel::stage_propellant(int stage_index) const noexcept {
  double total = 0.0;
  for (std::size_t i = 0; i < parts.size(); ++i) {
    if (stage[i] == stage_index && parts[i].kind == PartKind::Tank) {
      total += propellant[i];
    }
  }
  return total;
}

double Vessel::draw_propellant(double kilograms) noexcept {
  if (!(kilograms > 0.0)) {
    return 0.0;
  }
  const double available = stage_propellant(current_stage);
  if (available <= 0.0) {
    return 0.0;
  }
  const double taken = std::min(kilograms, available);

  // Spread in proportion to what each tank holds, so a stage with an uneven load
  // drains evenly rather than emptying one tank and leaving the other full.
  double removed = 0.0;
  for (std::size_t i = 0; i < parts.size(); ++i) {
    if (stage[i] != current_stage || parts[i].kind != PartKind::Tank || propellant[i] <= 0.0) {
      continue;
    }
    const double share = taken * (propellant[i] / available);
    propellant[i] = std::max(0.0, propellant[i] - share);
    removed += share;
  }
  return removed;
}

double Vessel::radius() const noexcept {
  double widest = 0.0;
  for (const Part& part : parts) {
    widest = std::max(widest, part.radius);
  }
  return widest;
}

double Vessel::length() const noexcept {
  double total = 0.0;
  for (const Part& part : parts) {
    total += part.length;
  }
  return total;
}

double Vessel::thrust() const noexcept {
  if (!(throttle > 0.0)) {
    return 0.0;
  }

  // An engine is fed by the tanks of its own stage. A stage that carries none
  // has nothing to burn, however much propellant is still aboard somewhere else.
  if (stage_propellant(current_stage) <= 0.0) {
    return 0.0;
  }

  double total = 0.0;
  for (std::size_t i = 0; i < parts.size(); ++i) {
    if (stage[i] == current_stage && parts[i].kind == PartKind::Engine) {
      total += parts[i].thrust;
    }
  }
  return total * std::min(1.0, std::max(0.0, throttle));
}

double Vessel::exhaust_velocity() const noexcept {
  double thrust_sum = 0.0;
  double thrust_over_isp = 0.0;
  for (std::size_t i = 0; i < parts.size(); ++i) {
    if (stage[i] == current_stage && parts[i].kind == PartKind::Engine && parts[i].isp > 0.0) {
      thrust_sum += parts[i].thrust;
      thrust_over_isp += parts[i].thrust / parts[i].isp;
    }
  }
  if (thrust_over_isp <= 0.0) {
    return 0.0;
  }
  return (thrust_sum / thrust_over_isp) * kStandardGravity;
}

bool Vessel::can_thrust() const noexcept {
  return thrust() > 0.0 && exhaust_velocity() > 0.0;
}

JettisonedStage Vessel::jettison() {
  JettisonedStage dropped;
  if (parts.empty()) {
    return dropped;
  }

  // The next current stage is the lowest-numbered one above this that still has
  // parts. No other test: a stage with no engine is a coast or a fairing, and
  // skipping it would silently swallow the one burn the player was waiting for.
  int next = -1;
  for (int s = current_stage + 1; s < stage_count && next < 0; ++s) {
    if (std::find(stage.begin(), stage.end(), s) != stage.end()) {
      next = s;
    }
  }
  // The last populated stage is the vessel's core, and there is nothing above it
  // to jettison to. Refusing leaves the vessel with something to fly.
  if (next < 0) {
    return dropped;
  }

  std::vector<Part> kept_parts;
  std::vector<int> kept_stage;
  std::vector<double> kept_propellant;
  for (std::size_t i = 0; i < parts.size(); ++i) {
    if (stage[i] == current_stage) {
      dropped.parts.push_back(parts[i]);
      dropped.propellant += propellant[i];
      continue;
    }
    kept_parts.push_back(parts[i]);
    kept_stage.push_back(stage[i]);
    kept_propellant.push_back(propellant[i]);
  }
  if (dropped.parts.empty()) {
    return dropped;
  }

  parts = std::move(kept_parts);
  stage = std::move(kept_stage);
  propellant = std::move(kept_propellant);
  current_stage = next;
  return dropped;
}

Vessel dock_vessels(const Vessel& target, const Vessel& absorbed) {
  Vessel joined = target;
  if (absorbed.parts.empty()) {
    return joined;
  }

  // Inserted at the front, because the vector is held top first: the arriving
  // vessel goes on top, which is where a stack keeps the thing that fires last.
  const std::size_t arriving = absorbed.parts.size();
  joined.parts.insert(joined.parts.begin(), absorbed.parts.begin(), absorbed.parts.end());
  joined.stage.insert(joined.stage.begin(), absorbed.stage.begin(), absorbed.stage.end());
  joined.propellant.insert(joined.propellant.begin(), absorbed.propellant.begin(),
                           absorbed.propellant.end());

  // Re-numbered above everything the target had. The absorbed vessel keeps its
  // own internal order — its stage 0 still fires first among its own parts —
  // and the whole band moves up by the target's stage count, so no two parts
  // anywhere in the joined stack share a number.
  const int base = target.stage_count;
  for (std::size_t i = 0; i < arriving; ++i) {
    joined.stage[i] = base + absorbed.stage[i];
  }
  joined.stage_count = base + absorbed.stage_count;

  // The target keeps flying. Its current stage, throttle and steering are what
  // the pilot had set, and a docking is not a reason for the engine to cut.
  joined.current_stage = target.current_stage;
  joined.throttle = target.throttle;
  joined.thrust_dir = target.thrust_dir;
  return joined;
}

std::vector<StageReport> stage_table(const Vessel& vessel) { return analyse(vessel, true); }

double total_delta_v(const Vessel& vessel) {
  double total = 0.0;
  for (const StageReport& report : stage_table(vessel)) {
    total += report.delta_v;
  }
  return total;
}

double remaining_delta_v(const Vessel& vessel) {
  double total = 0.0;
  for (const StageReport& report : analyse(vessel, false)) {
    if (report.index >= vessel.current_stage) {
      total += report.delta_v;
    }
  }
  return total;
}

Vessel build_vessel(const std::vector<StackPart>& stack, const PartCatalogue& catalogue) {
  Vessel vessel;
  int highest = -1;

  for (const StackPart& entry : stack) {
    const Part* part = catalogue.find(entry.part);
    if (part == nullptr) {
      throw std::invalid_argument("rocketlab: unknown part '" + entry.part + "'");
    }
    if (entry.stage < 0) {
      throw std::invalid_argument("rocketlab: part '" + entry.part + "' has a negative stage");
    }
    vessel.parts.push_back(*part);
    vessel.stage.push_back(entry.stage);
    vessel.propellant.push_back(part->propellant);
    highest = std::max(highest, entry.stage);
  }

  vessel.stage_count = highest + 1;

  // Start at the lowest stage that actually has parts, not blindly at 0. A stack
  // whose parts are numbered from 1 leaves stage 0 empty, and starting there
  // would be a vessel that can neither thrust (its stage has no engine) nor
  // jettison to the stage that can (there is nothing below 0 to walk up from) —
  // wedged on the pad with no input that unsticks it. Every shipped scenario
  // numbers from 0, so this picks the same stage it always did.
  vessel.current_stage = 0;
  for (int s = 0; s < vessel.stage_count; ++s) {
    if (std::find(vessel.stage.begin(), vessel.stage.end(), s) != vessel.stage.end()) {
      vessel.current_stage = s;
      break;
    }
  }
  return vessel;
}

}  // namespace rocketlab::core
