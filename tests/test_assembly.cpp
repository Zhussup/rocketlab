#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <stdexcept>

#include "rocketlab/core/assembly.hpp"
#include "rocketlab/core/vessel.hpp"

using namespace rocketlab::core;
using Catch::Matchers::WithinRel;

namespace {

/// The stock catalogue, held once. Looking a part up rather than naming a
/// pointer keeps the test reading like the catalogue it is testing.
[[nodiscard]] const Part& part(const PartCatalogue& catalogue, std::string_view name) {
  const Part* found = catalogue.find(name);
  REQUIRE(found != nullptr);
  return *found;
}

/// The shipped tug's stack, which is the fixture the auto-stager has to
/// reproduce.
[[nodiscard]] Assembly tug_assembly() {
  Assembly assembly;
  assembly.name = "tug";
  const char* names[] = {"probe.core", "tank.fl100", "engine.terrier",
                         "decoupler.large", "tank.fl400", "engine.skipper"};
  const int stages[] = {1, 1, 1, 0, 0, 0};
  for (std::size_t i = 0; i < 6; ++i) {
    StackPart entry;
    entry.part = names[i];
    entry.stage = stages[i];
    assembly.stack.push_back(std::move(entry));
  }
  return assembly;
}

}  // namespace

TEST_CASE("an assembly holds a stack of part names", "[assembly]") {
  const PartCatalogue catalogue = PartCatalogue::stock();
  Assembly assembly;

  CHECK(empty(assembly));
  CHECK(part_count(assembly) == 0);
  CHECK(stage_count(assembly) == 0);
  CHECK(stage_of(assembly, 0) == -1);

  insert_part(assembly, 0, "probe.core", 0);
  insert_part(assembly, 1, "tank.fl100", 0);
  insert_part(assembly, 2, "engine.terrier", 0);

  REQUIRE(part_count(assembly) == 3);
  CHECK(assembly.stack[0].part == "probe.core");
  CHECK(assembly.stack[2].part == "engine.terrier");

  // The assembly names parts and nothing else: there is no mass anywhere in the
  // document, so a saved stack cannot carry a performance figure that a
  // different catalogue would contradict.
  const Vessel vessel = resolve(assembly, catalogue);
  CHECK_THAT(vessel.mass(), WithinRel(part(catalogue, "probe.core").dry_mass +
                                          part(catalogue, "tank.fl100").dry_mass +
                                          part(catalogue, "tank.fl100").propellant +
                                          part(catalogue, "engine.terrier").dry_mass,
                                      1e-12));
}

TEST_CASE("inserting clamps to the ends of the stack", "[assembly]") {
  Assembly assembly;
  insert_part(assembly, 0, "probe.core", 0);
  insert_part(assembly, 99, "engine.ant", 0);  // past the bottom
  REQUIRE(part_count(assembly) == 2);
  CHECK(assembly.stack[1].part == "engine.ant");

  // Inserting into the middle pushes the rest down, top first.
  insert_part(assembly, 1, "tank.fl100", 0);
  REQUIRE(part_count(assembly) == 3);
  CHECK(assembly.stack[0].part == "probe.core");
  CHECK(assembly.stack[1].part == "tank.fl100");
  CHECK(assembly.stack[2].part == "engine.ant");

  // A negative stage is not a stage. It is clamped rather than stored, because
  // a stage number that reaches the flight model has to be one it can fire.
  insert_part(assembly, 0, "fins.basic", -4);
  CHECK(stage_of(assembly, 0) == 0);
}

TEST_CASE("a new part joins whatever it is bolted onto", "[assembly]") {
  const Assembly tug = tug_assembly();

  // Inserting at index 3 lands the new part directly below the service stage's
  // engine and above the decoupler, so it belongs with the part it will sit on:
  // the decoupler, which is stage 0.
  CHECK(stage_for_insert(tug, 3) == 0);
  // Directly below the probe core, inside the service stage.
  CHECK(stage_for_insert(tug, 1) == 1);
  // Past the bottom is the same request as at the bottom.
  CHECK(stage_for_insert(tug, 6) == 0);
  CHECK(stage_for_insert(tug, 99) == 0);
  // Nothing to bolt onto.
  CHECK(stage_for_insert(Assembly{}, 0) == 0);

  Assembly grown = tug;
  insert_part(grown, 3, "fins.basic", stage_for_insert(grown, 3));
  REQUIRE(part_count(grown) == 7);
  CHECK(grown.stack[3].part == "fins.basic");
  CHECK(grown.stack[3].stage == 0);
}

TEST_CASE("a part can be moved along the stack but not off it", "[assembly]") {
  Assembly assembly = tug_assembly();

  CHECK(move_part(assembly, 0, 1));  // probe core down one place
  CHECK(assembly.stack[0].part == "tank.fl100");
  CHECK(assembly.stack[1].part == "probe.core");

  CHECK(move_part(assembly, 1, -1));  // and back
  CHECK(assembly.stack[0].part == "probe.core");

  // The ends hold. The answer is false rather than a wrap, so a caller can stop
  // a cursor at the top rather than teleporting it to the bottom.
  CHECK_FALSE(move_part(assembly, 0, -1));
  CHECK_FALSE(move_part(assembly, 5, 1));
  CHECK_FALSE(move_part(assembly, 0, 0));
  CHECK_FALSE(move_part(assembly, 99, 1));
  CHECK(assembly.stack[0].part == "probe.core");
  CHECK(assembly.stack[5].part == "engine.skipper");
}

TEST_CASE("erasing past the end is harmless", "[assembly]") {
  Assembly assembly = tug_assembly();
  erase_part(assembly, 99);
  CHECK(part_count(assembly) == 6);
  erase_part(assembly, 5);
  CHECK(part_count(assembly) == 5);
  erase_part(assembly, 5);  // now out of range, and still harmless
  CHECK(part_count(assembly) == 5);
}

TEST_CASE("auto-staging reproduces the shipped tug", "[assembly]") {
  const PartCatalogue catalogue = PartCatalogue::stock();
  Assembly assembly = tug_assembly();
  // Deliberately scrambled first, so passing cannot be an accident of the
  // fixture already being in the right shape.
  for (StackPart& entry : assembly.stack) {
    entry.stage = 7;
  }
  CHECK(stage_count(assembly) == 8);

  auto_stage(assembly, catalogue);

  const int expected[] = {1, 1, 1, 0, 0, 0};
  for (std::size_t i = 0; i < 6; ++i) {
    INFO("part " << i << " (" << assembly.stack[i].part << ")");
    CHECK(assembly.stack[i].stage == expected[i]);
  }
  CHECK(stage_count(assembly) == 2);
}

TEST_CASE("auto-staging counts decouplers by kind, not by name", "[assembly]") {
  const PartCatalogue catalogue = PartCatalogue::stock();
  Assembly assembly;
  // The name says decoupler; the catalogue says it is a tank. Reading the name
  // would split the stack here, reading the kind does not.
  StackPart entry;
  entry.part = "tank.fl100";
  entry.stage = 0;
  assembly.stack.push_back(entry);
  entry.part = "decoupler.stack";
  entry.stage = 0;
  assembly.stack.push_back(entry);

  auto_stage(assembly, catalogue);
  CHECK(assembly.stack[0].stage == 1);  // above the decoupler: the next stage up
  CHECK(assembly.stack[1].stage == 0);  // the decoupler leaves with the booster

  // And an assembly with nothing that separates is one stage, which is what the
  // flight model would do with it anyway.
  Assembly solid;
  insert_part(solid, 0, "probe.core", 3);
  insert_part(solid, 1, "engine.ant", 3);
  auto_stage(solid, catalogue);
  CHECK(solid.stack[0].stage == 0);
  CHECK(solid.stack[1].stage == 0);

  // A name the catalogue does not have cannot be a decoupler.
  Assembly unknown;
  insert_part(unknown, 0, "not.a.part", 0);
  insert_part(unknown, 1, "engine.ant", 0);
  auto_stage(unknown, catalogue);
  CHECK(unknown.stack[0].stage == 0);
  CHECK(unknown.stack[1].stage == 0);
}

TEST_CASE("a decoupler at the bottom of a stack is left alone in stage zero", "[assembly]") {
  const PartCatalogue catalogue = PartCatalogue::stock();
  Assembly assembly;
  insert_part(assembly, 0, "probe.core", 0);
  insert_part(assembly, 1, "tank.fl100", 0);
  insert_part(assembly, 2, "engine.ant", 0);
  insert_part(assembly, 3, "decoupler.stack", 0);  // the last element is the bottom
  auto_stage(assembly, catalogue);

  // The rule is that a decoupler belongs to the stage below it, and this one has
  // no stage below it, so it forms one by itself. The same rule followed where
  // it leads rather than special-cased — and the stack still flies: stage 0
  // drops the ring and stage 1 lights the engine.
  CHECK(assembly.stack[0].stage == 1);
  CHECK(assembly.stack[1].stage == 1);
  CHECK(assembly.stack[2].stage == 1);
  CHECK(assembly.stack[3].stage == 0);

  Vessel vessel = resolve(assembly, catalogue);
  vessel.throttle = 1.0;
  CHECK(vessel.current_stage == 0);
  CHECK_FALSE(vessel.can_thrust());  // a ring on its own has nothing to burn
  const JettisonedStage dropped = vessel.jettison();
  REQUIRE(dropped.parts.size() == 1);
  CHECK(dropped.parts[0].name == "decoupler.stack");
  CHECK(vessel.current_stage == 1);
  CHECK(vessel.can_thrust());
}

TEST_CASE("a resolved assembly keeps the stage the flight model expects", "[assembly]") {
  const PartCatalogue catalogue = PartCatalogue::stock();
  Assembly assembly = tug_assembly();
  auto_stage(assembly, catalogue);

  Vessel vessel = resolve(assembly, catalogue);
  REQUIRE(vessel.parts.size() == 6);
  CHECK(vessel.stage_count == 2);
  CHECK(vessel.current_stage == 0);

  // Stage 0 is the skipper booster: it can burn, and dropping it leaves the
  // terrier service stage as the current one. A freshly built vessel is at zero
  // throttle — there is no "engine on" flag to get out of step with it — so the
  // throttle is opened before asking what the stage is worth.
  vessel.throttle = 1.0;
  CHECK(vessel.can_thrust());
  const double booster_thrust = vessel.thrust();
  CHECK_THAT(booster_thrust, WithinRel(part(catalogue, "engine.skipper").thrust, 1e-12));

  const JettisonedStage dropped = vessel.jettison();
  CHECK_FALSE(dropped.empty());
  CHECK(dropped.parts.size() == 3);
  CHECK(vessel.current_stage == 1);
  CHECK(vessel.can_thrust());
  CHECK_THAT(vessel.thrust(), WithinRel(part(catalogue, "engine.terrier").thrust, 1e-12));
}

TEST_CASE("a vessel numbered from one still flies", "[assembly]") {
  // A stack with nothing in stage 0 used to be wedged: `current_stage` was set
  // to 0 blindly, and stage 0 had no engine to burn and nothing below it to
  // jettison to. It now opens on the lowest stage that has parts.
  const PartCatalogue catalogue = PartCatalogue::stock();
  Assembly assembly;
  insert_part(assembly, 0, "probe.core", 1);
  insert_part(assembly, 1, "tank.fl100", 1);
  insert_part(assembly, 2, "engine.terrier", 1);

  Vessel vessel = resolve(assembly, catalogue);
  CHECK(vessel.stage_count == 2);
  CHECK(vessel.current_stage == 1);
  vessel.throttle = 1.0;
  CHECK(vessel.can_thrust());
  CHECK_THAT(vessel.thrust(), WithinRel(part(catalogue, "engine.terrier").thrust, 1e-12));

  // And the shipped fixtures are untouched: both number from 0, so the stage
  // that opens the mission is still stage 0.
  Assembly tug = tug_assembly();
  CHECK(resolve(tug, catalogue).current_stage == 0);
}

TEST_CASE("an assembly that names a part the catalogue does not have will not fly", "[assembly]") {
  const PartCatalogue catalogue = PartCatalogue::stock();
  Assembly assembly;
  insert_part(assembly, 0, "probe.core", 0);
  insert_part(assembly, 1, "engine.warp", 1);

  CHECK_THROWS_AS(resolve(assembly, catalogue), std::invalid_argument);
}
