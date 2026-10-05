// Tests for the assembly editor both clients drive.
//
// The state machine is shared precisely so that the terminal editor and the
// windowed one build the same rocket, which means what is worth testing is the
// part that is *not* obvious from the widget on top of it: where the cursor goes
// when a part is deleted, which stage an insert joins, and that a save writes
// one entity without disturbing the rest of the document. The arithmetic of the
// stack is `core`'s and is tested there; what is tested here is the editing.

#include <catch2/catch_test_macros.hpp>

#include <string>
#include <vector>

#include "rocketlab/core/part.hpp"
#include "rocketlab/core/scenario.hpp"
#include "rocketlab/hud/assembly.hpp"
#include "rocketlab/hud/editor.hpp"

using namespace rocketlab;

namespace {

namespace core = rocketlab::core;
namespace hud = rocketlab::hud;

core::Scenario two_entity_scenario() {
  core::Scenario scenario;
  scenario.name = "editor test";

  core::ScenarioEntity tug;
  tug.name = "Tug";
  tug.parts = {{"probe.core", 1}, {"tank.fl100", 1}, {"decoupler.large", 0},
               {"tank.fl400", 0}, {"engine.skipper", 0}};
  scenario.entities.push_back(tug);

  core::ScenarioEntity relay;
  relay.name = "Relay";
  relay.parts = {{"probe.core", 0}};
  scenario.entities.push_back(relay);
  return scenario;
}

/// The name of the catalogue part at `index` in the picker, which is the order
/// `Editor::insert` chooses from.
std::string picked_name(const core::PartCatalogue& catalogue, int index) {
  return hud::part_lines(catalogue)[static_cast<std::size_t>(index)].name;
}

}  // namespace

TEST_CASE("the editor loads the stack of the entity it was pointed at", "[hud][editor]") {
  const core::Scenario scenario = two_entity_scenario();
  const core::PartCatalogue catalogue = core::PartCatalogue::stock();

  hud::Editor editor;
  CHECK_FALSE(editor.open);

  SECTION("by name, and it opens") {
    CHECK(editor.load(scenario, "Tug"));
    CHECK(editor.open);
    CHECK(editor.entity == "Tug");
    CHECK(editor.assembly.name == "Tug");
    CHECK(editor.assembly.stack.size() == 5);
    CHECK(editor.stack_cursor == 0);
    CHECK(editor.part_cursor == 0);
    CHECK(editor.status.empty());
  }

  SECTION("a point mass loads empty rather than being refused") {
    core::Scenario sparse = two_entity_scenario();
    sparse.entities[1].parts.clear();

    CHECK(editor.load(sparse, "Relay"));
    CHECK(editor.open);
    CHECK(editor.assembly.stack.empty());
    // Said plainly, because an empty stack is a state with one obvious next
    // step and a refusal would leave no way to take it.
    CHECK_FALSE(editor.status.empty());

    editor.insert(catalogue);
    CHECK(editor.assembly.stack.size() == 1);
  }

  SECTION("an entity that is not in the document is refused, and says so") {
    CHECK_FALSE(editor.load(scenario, "Nonesuch"));
    CHECK(editor.open);
    CHECK(editor.assembly.stack.empty());
    CHECK_FALSE(editor.status.empty());
  }

  SECTION("loading a second stack replaces the first entirely") {
    REQUIRE(editor.load(scenario, "Tug"));
    editor.move_cursor(+2);
    REQUIRE(editor.stack_cursor == 2);

    REQUIRE(editor.load(scenario, "Relay"));
    CHECK(editor.stack_cursor == 0);
    CHECK(editor.part_cursor == 0);
    CHECK(editor.assembly.name == "Relay");
    CHECK(editor.assembly.stack.size() == 1);
  }
}

TEST_CASE("the cursor stays inside the stack it is moving in", "[hud][editor]") {
  const core::Scenario scenario = two_entity_scenario();
  hud::Editor editor;
  REQUIRE(editor.load(scenario, "Tug"));  // five parts

  SECTION("it wraps at both ends") {
    editor.move_cursor(-1);
    CHECK(editor.stack_cursor == 4);
    editor.move_cursor(+1);
    CHECK(editor.stack_cursor == 0);
  }

  SECTION("deleting the last part pulls it back") {
    editor.move_cursor(+4);
    REQUIRE(editor.stack_cursor == 4);
    editor.erase();
    CHECK(editor.assembly.stack.size() == 4);
    CHECK(editor.stack_cursor == 3);
  }

  SECTION("deleting the only part leaves the cursor at the top") {
    REQUIRE(editor.load(scenario, "Relay"));
    editor.erase();
    CHECK(editor.assembly.stack.empty());
    CHECK(editor.stack_cursor == 0);
    // Erasing an empty stack is a no-op, so a client can hold the key down.
    editor.erase();
    CHECK(editor.assembly.stack.empty());
  }

  SECTION("a cursor left over from a longer stack is clamped by the next action") {
    editor.move_cursor(+4);
    // Somewhere the stack shrank underneath the cursor. Clamping happens when
    // the cursor is used rather than when it is set, so nothing here can point
    // past the end of the vector.
    editor.assembly.stack.resize(2);
    editor.erase();
    CHECK(editor.assembly.stack.size() == 1);
  }
}

TEST_CASE("an insert joins the part it is bolted onto", "[hud][editor]") {
  const core::Scenario scenario = two_entity_scenario();
  const core::PartCatalogue catalogue = core::PartCatalogue::stock();

  hud::Editor editor;
  REQUIRE(editor.load(scenario, "Tug"));

  SECTION("the new part takes the cursor's place and the rest shifts down") {
    editor.move_cursor(+1);
    REQUIRE(editor.stack_cursor == 1);
    const std::string below = editor.assembly.stack[1].part;
    const std::string before = editor.assembly.stack[0].part;

    editor.insert(catalogue);

    CHECK(editor.assembly.stack.size() == 6);
    CHECK(editor.assembly.stack[0].part == before);          // above is untouched
    CHECK(editor.assembly.stack[1].part == picked_name(catalogue, 0));
    CHECK(editor.assembly.stack[2].part == below);           // and this one moved
    CHECK(editor.stack_cursor == 1);
  }

  SECTION("the stage comes from the neighbour below, not from a fresh number") {
    // Somewhere inside the booster: the part joins stage 0 rather than starting
    // a stage of its own, which is what makes a second stage something a person
    // builds by putting a decoupler in.
    editor.move_cursor(+4);
    REQUIRE(editor.stack_cursor == 4);
    const int stage_below = core::stage_of(editor.assembly, 4);

    editor.insert(catalogue);
    CHECK(core::stage_of(editor.assembly, 4) == stage_below);
  }

  SECTION("inserting at the top of an empty stack is stage 0") {
    REQUIRE(editor.load(scenario, "Relay"));
    editor.erase();
    editor.insert(catalogue);
    CHECK(core::stage_of(editor.assembly, 0) == 0);
  }

  SECTION("the picker cursor chooses what goes in") {
    editor.pick(catalogue, +2);
    REQUIRE(editor.part_cursor == 2);
    editor.insert(catalogue);
    CHECK(editor.assembly.stack[0].part == picked_name(catalogue, 2));
  }

  SECTION("the picker wraps both ways") {
    const int count = static_cast<int>(hud::part_lines(catalogue).size());
    REQUIRE(count > 0);
    editor.pick(catalogue, -1);
    CHECK(editor.part_cursor == count - 1);
    editor.pick(catalogue, +1);
    CHECK(editor.part_cursor == 0);
  }
}

TEST_CASE("moving a part stops at the ends of the stack", "[hud][editor]") {
  const core::Scenario scenario = two_entity_scenario();
  hud::Editor editor;
  REQUIRE(editor.load(scenario, "Tug"));

  const std::vector<std::string> before = [&editor] {
    std::vector<std::string> names;
    for (const core::StackPart& part : editor.assembly.stack) {
      names.push_back(part.part);
    }
    return names;
  }();

  SECTION("down moves toward the booster") {
    editor.shift(+1);
    CHECK(editor.stack_cursor == 1);
    CHECK(editor.assembly.stack[0].part == before[1]);
    CHECK(editor.assembly.stack[1].part == before[0]);
  }

  SECTION("off the top does nothing at all, cursor included") {
    editor.shift(-1);
    CHECK(editor.stack_cursor == 0);
    for (std::size_t i = 0; i < before.size(); ++i) {
      CHECK(editor.assembly.stack[i].part == before[i]);
    }
  }

  SECTION("off the bottom does nothing at all") {
    editor.move_cursor(+4);
    REQUIRE(editor.stack_cursor == 4);
    editor.shift(+1);
    CHECK(editor.stack_cursor == 4);
    for (std::size_t i = 0; i < before.size(); ++i) {
      CHECK(editor.assembly.stack[i].part == before[i]);
    }
  }
}

TEST_CASE("restaging moves one part and nothing else", "[hud][editor]") {
  const core::Scenario scenario = two_entity_scenario();
  hud::Editor editor;
  REQUIRE(editor.load(scenario, "Tug"));

  SECTION("the cursor's part moves up and down") {
    const int stage = core::stage_of(editor.assembly, 0);
    editor.restage(+1);
    CHECK(core::stage_of(editor.assembly, 0) == stage + 1);
    editor.restage(-1);
    CHECK(core::stage_of(editor.assembly, 0) == stage);

    // The neighbours are untouched: restaging is one part, which is the point
    // of it being a per-part key beside the automatic one.
    editor.restage(+1);
    CHECK(core::stage_of(editor.assembly, 1) == scenario.entities[0].parts[1].stage);
  }

  SECTION("it will not go below zero") {
    for (int i = 0; i < 4; ++i) {
      editor.restage(-1);
    }
    CHECK(core::stage_of(editor.assembly, 0) == 0);
  }

  SECTION("auto-staging reproduces the decoupler convention") {
    editor.auto_restage(core::PartCatalogue::stock());
    // Top first: two parts above the ring, the ring with the two below it. The
    // decoupler belongs to the stage it releases.
    CHECK(core::stage_of(editor.assembly, 0) == 1);
    CHECK(core::stage_of(editor.assembly, 1) == 1);
    CHECK(core::stage_of(editor.assembly, 2) == 0);
    CHECK(core::stage_of(editor.assembly, 3) == 0);
    CHECK(core::stage_of(editor.assembly, 4) == 0);
    CHECK_FALSE(editor.status.empty());
  }
}

TEST_CASE("a stack that will not build is reported, not thrown", "[hud][editor]") {
  core::Scenario scenario = two_entity_scenario();
  hud::Editor editor;
  REQUIRE(editor.load(scenario, "Tug"));

  const core::PartCatalogue catalogue = core::PartCatalogue::stock();
  core::Vessel vessel;
  std::string error;
  CHECK(editor.resolve(catalogue, vessel, error));
  CHECK(error.empty());
  CHECK_FALSE(vessel.empty());

  // A name the catalogue does not have. The editor cannot produce one, but a
  // document written by something else can, and the panel still has to draw.
  editor.assembly.stack.push_back({"engine.nonesuch", 0});
  CHECK_FALSE(editor.resolve(catalogue, vessel, error));
  CHECK_FALSE(error.empty());

  // And an empty stack resolves to an empty vessel rather than failing: an
  // empty stack is not an error, it is a stack a person is partway through.
  editor.assembly.stack.clear();
  CHECK(editor.resolve(catalogue, vessel, error));
  CHECK(vessel.empty());
}

TEST_CASE("a save writes one entity and leaves the document alone", "[hud][editor]") {
  core::Scenario scenario = two_entity_scenario();
  const core::PartCatalogue catalogue = core::PartCatalogue::stock();

  hud::Editor editor;
  REQUIRE(editor.load(scenario, "Tug"));

  SECTION("the edited stack replaces that entity's parts") {
    editor.pick(catalogue, +2);
    editor.insert(catalogue);
    REQUIRE(editor.write(scenario));

    CHECK(scenario.entities[0].parts.size() == 6);
    CHECK(scenario.entities[0].parts[0].part == picked_name(catalogue, 2));
    CHECK(scenario.entities[0].controllable);
    // The other entity, and the document's own name, are not the editor's to
    // touch.
    CHECK(scenario.entities[1].parts.size() == 1);
    CHECK(scenario.name == "editor test");
  }

  SECTION("writing an entity that is not there is refused") {
    editor.entity = "Nonesuch";
    CHECK_FALSE(editor.write(scenario));
    CHECK_FALSE(editor.status.empty());
    CHECK(scenario.entities[0].parts.size() == 5);
  }

  SECTION("what was written is what resolves") {
    editor.auto_restage(catalogue);
    editor.pick(catalogue, +1);
    editor.insert(catalogue);
    REQUIRE(editor.write(scenario));

    // The saved document and the previewed stack are the same rocket: a save
    // that wrote something other than what the panel showed would be the one
    // bug that makes the editor untrustworthy.
    core::Vessel previewed;
    std::string error;
    REQUIRE(editor.resolve(catalogue, previewed, error));

    hud::Editor reopened;
    REQUIRE(reopened.load(scenario, "Tug"));
    core::Vessel written;
    REQUIRE(reopened.resolve(catalogue, written, error));

    CHECK(previewed.parts.size() == written.parts.size());
    CHECK(previewed.mass() == written.mass());
    CHECK(previewed.stage_count == written.stage_count);
  }
}

TEST_CASE("an edit disarms a pending overwrite", "[hud][editor]") {
  core::Scenario scenario = two_entity_scenario();
  const core::PartCatalogue catalogue = core::PartCatalogue::stock();

  hud::Editor editor;
  REQUIRE(editor.load(scenario, "Tug"));

  editor.save_armed = true;
  editor.insert(catalogue);
  CHECK_FALSE(editor.save_armed);

  editor.save_armed = true;
  editor.erase();
  CHECK_FALSE(editor.save_armed);

  editor.save_armed = true;
  editor.move_cursor(+1);
  // Moving the cursor is not an edit: nothing about the stack changed, so the
  // confirmation a person gave is still about the same file.
  editor.shift(+1);
  CHECK_FALSE(editor.save_armed);

  editor.save_armed = true;
  editor.restage(+1);
  CHECK_FALSE(editor.save_armed);

  editor.save_armed = true;
  editor.auto_restage(catalogue);
  CHECK_FALSE(editor.save_armed);

  SECTION("closing disarms it too") {
    editor.save_armed = true;
    editor.close();
    CHECK_FALSE(editor.save_armed);
    CHECK_FALSE(editor.open);
  }
}
