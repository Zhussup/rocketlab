#include "rocketlab/core/assembly.hpp"

#include <algorithm>

namespace rocketlab::core {

std::size_t part_count(const Assembly& assembly) noexcept { return assembly.stack.size(); }

bool empty(const Assembly& assembly) noexcept { return assembly.stack.empty(); }

int stage_of(const Assembly& assembly, std::size_t index) noexcept {
  return index < assembly.stack.size() ? assembly.stack[index].stage : -1;
}

int stage_count(const Assembly& assembly) noexcept {
  int highest = -1;
  for (const StackPart& part : assembly.stack) {
    highest = std::max(highest, part.stage);
  }
  return highest + 1;
}

int stage_for_insert(const Assembly& assembly, std::size_t index) noexcept {
  // Clamped rather than checked: inserting past the bottom is the same request
  // as inserting at the bottom, and both are attached to the last part.
  const std::size_t at = std::min(index, assembly.stack.size());
  if (assembly.stack.empty()) {
    return 0;
  }
  if (at < assembly.stack.size()) {
    return assembly.stack[at].stage;
  }
  return assembly.stack.back().stage;
}

void insert_part(Assembly& assembly, std::size_t index, std::string_view part_name, int stage) {
  const std::size_t at = std::min(index, assembly.stack.size());
  StackPart entry;
  entry.part = std::string(part_name);
  entry.stage = std::max(0, stage);
  assembly.stack.insert(assembly.stack.begin() + static_cast<std::ptrdiff_t>(at),
                        std::move(entry));
}

void erase_part(Assembly& assembly, std::size_t index) {
  if (index >= assembly.stack.size()) {
    return;
  }
  assembly.stack.erase(assembly.stack.begin() + static_cast<std::ptrdiff_t>(index));
}

bool move_part(Assembly& assembly, std::size_t index, int delta) {
  if (index >= assembly.stack.size() || delta == 0) {
    return false;
  }
  // Cast through a signed type: comparing an unsigned index against a negative
  // target is the classic way this check passes when it should not.
  const std::ptrdiff_t target = static_cast<std::ptrdiff_t>(index) + delta;
  if (target < 0 || target >= static_cast<std::ptrdiff_t>(assembly.stack.size())) {
    return false;
  }
  std::swap(assembly.stack[index], assembly.stack[static_cast<std::size_t>(target)]);
  return true;
}

void set_part_stage(Assembly& assembly, std::size_t index, int stage) {
  if (index >= assembly.stack.size() || stage < 0) {
    return;
  }
  assembly.stack[index].stage = stage;
}

void auto_stage(Assembly& assembly, const PartCatalogue& catalogue) {
  // Walked from the bottom up, because that is the order the stages fire in and
  // the counter is the answer. A decoupler is counted with the parts below it,
  // so the boundary is crossed on the way *up* past one: everything at or below
  // a decoupler is in its stage, everything above it is one stage higher.
  //
  // The kind comes from the catalogue and not from the name: "decoupler.large"
  // is a naming convention the stock catalogue happens to follow, and an
  // assembly holding a part called `ring.experimental` would be silently
  // mis-staged by anything that read the string.
  //
  // Note the consequence at the very bottom of a stack: a decoupler there has
  // no stage below it to belong to, so it is left alone in stage 0 and fires
  // off by itself. That is the same rule followed where it leads rather than a
  // special case, and it is a stack that flies: one press stages the ring off
  // and the rest of the rocket lights.
  int stage = 0;
  for (std::size_t i = assembly.stack.size(); i > 0; --i) {
    StackPart& entry = assembly.stack[i - 1];
    entry.stage = stage;
    const Part* part = catalogue.find(entry.part);
    if (part != nullptr && part->kind == PartKind::Decoupler) {
      ++stage;
    }
  }
}

Vessel resolve(const Assembly& assembly, const PartCatalogue& catalogue) {
  return build_vessel(assembly.stack, catalogue);
}

}  // namespace rocketlab::core
