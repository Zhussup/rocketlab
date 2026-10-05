// Tests for the assembly readouts.
//
// The stack, the picker and the stage table are what both the terminal editor
// and the windowed one put on screen, so they are checked here rather than by
// looking at either.

#include <catch2/catch_test_macros.hpp>

#include <string>
#include <vector>

#include "rocketlab/core/assembly.hpp"
#include "rocketlab/hud/assembly.hpp"
#include "rocketlab/hud/readout.hpp"

using namespace rocketlab;

namespace {

namespace core = rocketlab::core;
namespace hud = rocketlab::hud;

const hud::Row* find_row(const std::vector<hud::Row>& rows, std::string_view label) {
  for (const hud::Row& row : rows) {
    if (row.label == label) {
      return &row;
    }
  }
  return nullptr;
}

/// The shipped tug, resolved. Six parts, two stages, a decoupler between them.
[[nodiscard]] core::Vessel tug(const core::PartCatalogue& catalogue) {
  core::Assembly assembly;
  assembly.name = "tug";
  const char* names[] = {"probe.core", "tank.fl100", "engine.terrier",
                         "decoupler.large", "tank.fl400", "engine.skipper"};
  const int stages[] = {1, 1, 1, 0, 0, 0};
  for (std::size_t i = 0; i < 6; ++i) {
    core::insert_part(assembly, assembly.stack.size(), names[i], stages[i]);
  }
  return core::resolve(assembly, catalogue);
}

}  // namespace

TEST_CASE("the stack lists the parts in the order they are stacked", "[assembly][hud]") {
  const core::PartCatalogue catalogue = core::PartCatalogue::stock();
  const std::vector<hud::StackLine> lines = hud::stack_lines(tug(catalogue));

  REQUIRE(lines.size() == 6);
  // Top first, which is how the stack is held and how a rocket reads.
  CHECK(lines[0].part == "probe.core");
  CHECK(lines[0].kind == "pod");
  CHECK(lines[0].stage == 1);
  CHECK(lines[2].part == "engine.terrier");
  CHECK(lines[2].kind == "engine");
  CHECK(lines[3].part == "decoupler.large");
  CHECK(lines[5].part == "engine.skipper");
  CHECK(lines[5].stage == 0);

  // A freshly resolved assembly has full tanks, so a tank's line carries its
  // capacity and not a zero.
  CHECK(lines[1].propellant == 4000.0);
  CHECK(lines[1].dry_mass == 500.0);
}

TEST_CASE("the picker describes each part by what it can do", "[assembly][hud]") {
  const core::PartCatalogue catalogue = core::PartCatalogue::stock();
  const std::vector<hud::PartLine> lines = hud::part_lines(catalogue);

  REQUIRE(lines.size() == catalogue.size());

  const auto find = [&lines](std::string_view name) -> const hud::PartLine* {
    for (const hud::PartLine& line : lines) {
      if (line.name == name) {
        return &line;
      }
    }
    return nullptr;
  };

  const hud::PartLine* engine = find("engine.terrier");
  REQUIRE(engine != nullptr);
  CHECK(engine->kind == "engine");
  CHECK(engine->mass == "500.0 kg");
  CHECK(engine->detail.find("kN") != std::string::npos);
  CHECK(engine->detail.find("345") != std::string::npos);

  // A tank's detail is what it holds, never a thrust it does not have. Printing
  // "0 N" against every tank would make the picker read as a list of faults.
  const hud::PartLine* tank = find("tank.fl400");
  REQUIRE(tank != nullptr);
  CHECK(tank->detail == "32.00 t");

  const hud::PartLine* pod = find("probe.core");
  REQUIRE(pod != nullptr);
  CHECK(pod->detail.empty());
}

TEST_CASE("the summary weighs the stack it was given", "[assembly][hud]") {
  const core::PartCatalogue catalogue = core::PartCatalogue::stock();
  const core::Vessel vessel = tug(catalogue);
  const std::vector<hud::Row> rows = hud::stack_summary(vessel);

  const hud::Row* parts = find_row(rows, "parts");
  const hud::Row* stages = find_row(rows, "stages");
  const hud::Row* dry = find_row(rows, "dry mass");
  const hud::Row* propellant = find_row(rows, "propellant");
  const hud::Row* mass = find_row(rows, "mass");
  const hud::Row* delta_v = find_row(rows, "delta-v");
  REQUIRE(parts != nullptr);
  REQUIRE(stages != nullptr);
  REQUIRE(dry != nullptr);
  REQUIRE(propellant != nullptr);
  REQUIRE(mass != nullptr);
  REQUIRE(delta_v != nullptr);

  CHECK(parts->value == "6");
  CHECK(stages->value == "2");
  CHECK(propellant->value == "36.00 t");
  // Dry plus propellant is the wet mass, which is the one identity this panel
  // has to satisfy: the two halves are shown separately, so they must add up.
  CHECK(dry->value == "8.29 t");
  CHECK(mass->value == "44.29 t");
  CHECK(delta_v->value.find("km/s") != std::string::npos);

  // A vessel with no parts is not a crash: the panel just says so.
  const std::vector<hud::Row> none = hud::stack_summary(core::Vessel{});
  const hud::Row* empty_parts = find_row(none, "parts");
  REQUIRE(empty_parts != nullptr);
  CHECK(empty_parts->value == "0");
  CHECK(find_row(none, "delta-v")->value == "0.00 m/s");
}

TEST_CASE("the stage table separates a coast from an engine with no fuel", "[assembly][hud]") {
  const core::PartCatalogue catalogue = core::PartCatalogue::stock();

  const std::vector<hud::Row> rows = hud::stage_rows(tug(catalogue));
  REQUIRE(rows.size() == 2);
  CHECK(rows[0].label == "stage 0");
  CHECK(rows[1].label == "stage 1");
  CHECK_FALSE(rows[0].emphasised);
  CHECK_FALSE(rows[1].emphasised);
  CHECK(rows[0].value.find("km/s") != std::string::npos);
  CHECK(rows[0].value.find(" in ") != std::string::npos);  // a burn time

  SECTION("an engine whose own stage has no tank is flagged") {
    core::Assembly assembly;
    // The tank is in stage 0 and the engine in stage 1, so the engine lights
    // only after the stage holding its fuel has already been dropped.
    core::insert_part(assembly, 0, "tank.fl100", 0);
    core::insert_part(assembly, 1, "engine.terrier", 1);
    const std::vector<hud::Row> broken = hud::stage_rows(core::resolve(assembly, catalogue));
    REQUIRE(broken.size() == 2);
    CHECK(broken[1].value == "no propellant");
    CHECK(broken[1].emphasised);
  }

  SECTION("tanks and no engine is a coast, not a fault") {
    core::Assembly assembly;
    core::insert_part(assembly, 0, "tank.fl100", 0);
    const std::vector<hud::Row> coast = hud::stage_rows(core::resolve(assembly, catalogue));
    REQUIRE(coast.size() == 1);
    CHECK(coast[0].value == "coast, 4.00 t");
    // Named and explained rather than emphasised: a person who meant to build a
    // coast stage should not be told they made a mistake.
    CHECK_FALSE(coast[0].emphasised);
  }

  SECTION("a stage with neither is empty") {
    core::Assembly assembly;
    core::insert_part(assembly, 0, "decoupler.stack", 0);
    core::insert_part(assembly, 1, "tank.fl100", 1);
    const std::vector<hud::Row> empty = hud::stage_rows(core::resolve(assembly, catalogue));
    REQUIRE(empty.size() == 2);
    CHECK(empty[0].value == "empty");
    CHECK_FALSE(empty[0].emphasised);
  }
}
