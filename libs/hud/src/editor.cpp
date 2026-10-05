#include "rocketlab/hud/editor.hpp"

#include <algorithm>
#include <exception>
#include <utility>
#include <vector>

#include "rocketlab/hud/assembly.hpp"

namespace rocketlab::hud {

namespace {

/// The picker, which is the catalogue in the order a person reads it. Insert
/// needs it to turn `part_cursor` into a name, and every move of the picker
/// cursor needs its length.
///
/// Built per call rather than cached. A catalogue is a handful of entries and
/// an editor acts on a keypress, so the cost is invisible — and a cache would
/// mean the editor holding a copy of a catalogue it was built from, which is
/// one more thing to keep in step.
[[nodiscard]] std::vector<PartLine> picker(const core::PartCatalogue& catalogue) {
  return part_lines(catalogue);
}

/// The cursor clamped into a stack of `count` parts.
[[nodiscard]] int clamp_cursor(int cursor, std::size_t count) noexcept {
  if (count == 0) {
    return 0;
  }
  return std::min(std::max(0, cursor), static_cast<int>(count) - 1);
}

}  // namespace

core::ScenarioEntity* find_scenario_entity(core::Scenario& scenario, std::string_view name) noexcept {
  for (core::ScenarioEntity& entity : scenario.entities) {
    if (entity.name == name) {
      return &entity;
    }
  }
  return nullptr;
}

bool Editor::load(const core::Scenario& scenario, std::string_view entity_name) {
  assembly = core::Assembly{};
  assembly.name = std::string(entity_name);
  entity = std::string(entity_name);
  stack_cursor = 0;
  part_cursor = 0;
  open = true;
  save_armed = false;

  for (const core::ScenarioEntity& candidate : scenario.entities) {
    if (candidate.name != entity_name) {
      continue;
    }
    assembly.stack = candidate.parts;
    status = assembly.stack.empty() ? "a point mass: add the first part" : "";
    return true;
  }

  status = "not in the scenario file, so this stack cannot be saved or flown";
  return false;
}

void Editor::close() noexcept {
  open = false;
  save_armed = false;
}

void Editor::settle() noexcept {
  stack_cursor = clamp_cursor(stack_cursor, core::part_count(assembly));
}

void Editor::move_cursor(int direction) noexcept {
  const int count = static_cast<int>(core::part_count(assembly));
  if (count <= 0) {
    stack_cursor = 0;
    return;
  }
  stack_cursor = ((stack_cursor + direction) % count + count) % count;
}

void Editor::insert(const core::PartCatalogue& catalogue) {
  const std::vector<PartLine> parts = picker(catalogue);
  if (parts.empty()) {
    return;
  }
  settle();
  // Clamped to *one past* the cursor, not to it: inserting above the last part
  // means the new part goes at the end, and every index up to the count is a
  // place a part can be put.
  const std::size_t at = std::min(static_cast<std::size_t>(stack_cursor), core::part_count(assembly));
  const std::size_t chosen =
      std::min(static_cast<std::size_t>(std::max(0, part_cursor)), parts.size() - 1);

  core::insert_part(assembly, at, parts[chosen].name, core::stage_for_insert(assembly, at));
  stack_cursor = static_cast<int>(at);
  touch();
  status.clear();
}

void Editor::erase() noexcept {
  if (core::empty(assembly)) {
    stack_cursor = 0;
    return;
  }
  settle();
  core::erase_part(assembly, static_cast<std::size_t>(stack_cursor));
  stack_cursor = clamp_cursor(stack_cursor, core::part_count(assembly));
  touch();
  status.clear();
}

void Editor::shift(int delta) noexcept {
  settle();
  const std::size_t at = static_cast<std::size_t>(stack_cursor);
  if (core::move_part(assembly, at, delta)) {
    stack_cursor += delta;
    touch();
  }
}

void Editor::restage(int delta) noexcept {
  settle();
  const std::size_t at = static_cast<std::size_t>(stack_cursor);
  const int stage = core::stage_of(assembly, at);
  if (stage < 0) {
    return;
  }
  core::set_part_stage(assembly, at, stage + delta);
  touch();
}

void Editor::pick(const core::PartCatalogue& catalogue, int delta) noexcept {
  const int count = static_cast<int>(picker(catalogue).size());
  if (count <= 0) {
    return;
  }
  part_cursor = ((part_cursor + delta) % count + count) % count;
}

void Editor::auto_restage(const core::PartCatalogue& catalogue) noexcept {
  core::auto_stage(assembly, catalogue);
  touch();
  status = "staged from the decouplers";
}

bool Editor::resolve(const core::PartCatalogue& catalogue, core::Vessel& out,
                     std::string& error) const {
  try {
    out = core::resolve(assembly, catalogue);
    return true;
  } catch (const std::exception& failure) {
    error = failure.what();
    return false;
  }
}

bool Editor::write(core::Scenario& scenario) noexcept {
  core::ScenarioEntity* spec = find_scenario_entity(scenario, entity);
  if (spec == nullptr) {
    status = "not in the scenario file, so there is nothing to write";
    return false;
  }
  spec->parts = assembly.stack;
  // A stack that was written down is one somebody built, so it is flown by
  // whoever is aboard it. An entity with no parts keeps whatever the file said,
  // because a point mass has no computer to fly.
  if (!assembly.stack.empty()) {
    spec->controllable = true;
  }
  return true;
}

}  // namespace rocketlab::hud
