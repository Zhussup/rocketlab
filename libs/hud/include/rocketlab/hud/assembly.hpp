// Readouts for a stack being assembled.
//
// Separate from `readout.hpp` because that file is about a snapshot — what the
// simulation is doing now — and this one is about a design that has not been
// built yet. They share the formatters and the `Row` type and nothing else.
//
// The reason this lives in `hud` rather than in either client is the same
// reason the snapshot readout does: the terminal editor and the windowed one
// have to show the same stack, the same stage table and the same delta-v, and
// two copies of "is this stage flyable" is how they start disagreeing.

#pragma once

#include <string>
#include <vector>

#include "rocketlab/core/assembly.hpp"
#include "rocketlab/core/part.hpp"
#include "rocketlab/core/vessel.hpp"
#include "rocketlab/hud/readout.hpp"

namespace rocketlab::hud {

/// One part in a stack, as the editor lists it. Top first, like the stack.
struct StackLine {
  std::string part;  // the catalogue name, never a resolved `Part`
  std::string kind;  // "tank", "engine", ...
  int stage{0};
  double dry_mass{0.0};
  /// What the part holds now. Capacity for a freshly resolved assembly, what is
  /// left for one that is flying.
  double propellant{0.0};
};

/// A vessel's parts, top first, one entry each.
[[nodiscard]] std::vector<StackLine> stack_lines(const core::Vessel& vessel);

/// One entry of the part picker: everything a person needs to choose a part
/// without opening the catalogue source.
struct PartLine {
  std::string name;
  std::string kind;
  std::string mass;    // "500 kg"
  std::string detail;  // "60 kN, 345 s" for an engine, "4000 kg" of propellant
                       // for a tank, empty for anything inert
};

[[nodiscard]] std::vector<PartLine> part_lines(const core::PartCatalogue& catalogue);

/// What the whole stack weighs, measures and is worth. Takes a resolved vessel,
/// so a caller that has already built one does not build it twice.
[[nodiscard]] std::vector<Row> stack_summary(const core::Vessel& vessel);

/// One row per stage, in firing order, describing what that stage can do.
///
/// A stage that carries an engine but no propellant in its own tanks is
/// emphasised: an engine is fed by the tanks that share its stage number, so an
/// engine with the fuel a stage below is the single mistake this table exists to
/// catch. A stage with fuel and no engine is a coast, which is a design choice
/// and reads as one.
[[nodiscard]] std::vector<Row> stage_rows(const core::Vessel& vessel);

}  // namespace rocketlab::hud
