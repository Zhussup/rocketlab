#pragma once

#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

#include "rocketlab/core/part.hpp"
#include "rocketlab/core/vessel.hpp"

namespace rocketlab::core {

/// A stack under construction: the document an assembly editor edits.
///
/// This is design-time data, not simulation state, and it is deliberately in
/// core rather than in a client. Both clients edit a stack, and both have to
/// agree on what "insert above" and "renumber the stages" mean — a second
/// implementation is how one client's stack quietly starts meaning something
/// different from the other's. Nothing in here touches a `World`, so rule 5 is
/// untouched: building a rocket is arithmetic over the part catalogue, and the
/// same call that builds one from a scenario builds one from an editor.
///
/// It is exactly the shape a scenario stores under `parts`, which is what makes
/// "design it, then save it" and "load it, then fly it" the same document.
///
/// Parts are held top first, the way a rocket reads on paper. The bottom of the
/// stack is the booster and it carries the lowest stage number, because stage 0
/// fires first.
struct Assembly {
  std::string name{"untitled"};
  std::vector<StackPart> stack;
};

[[nodiscard]] std::size_t part_count(const Assembly& assembly) noexcept;
[[nodiscard]] bool empty(const Assembly& assembly) noexcept;

/// The stage of the part at `index`, or -1 when there is no such part.
[[nodiscard]] int stage_of(const Assembly& assembly, std::size_t index) noexcept;

/// One past the highest stage number in use, so an empty assembly has none.
[[nodiscard]] int stage_count(const Assembly& assembly) noexcept;

/// The stage a new part should join if it is inserted at `index`: whatever it
/// is being bolted onto, which is the part that will sit directly below it.
///
/// An editor that started every insert at a fresh stage would make a person
/// re-tag every part of a booster by hand, and one that always said 0 would make
/// a second stage impossible to build. Following the neighbour is the answer
/// that needs no explaining, and an empty stack is stage 0 by definition.
[[nodiscard]] int stage_for_insert(const Assembly& assembly, std::size_t index) noexcept;

/// Inserts a part so that it ends up at `index`, clamped to the ends of the
/// stack; everything from `index` down shifts one place toward the bottom. The
/// new part joins `stage`, which a caller with no opinion should get from
/// `stage_for_insert`.
///
/// Only the name is stored, never a resolved `Part`: an assembly is what a
/// scenario writes down, and a scenario names parts rather than carrying their
/// masses, so that a vessel's performance cannot depend on who wrote the file.
void insert_part(Assembly& assembly, std::size_t index, std::string_view part_name, int stage);

/// Removes the part at `index`. Out of range is a no-op, so a caller that has
/// just deleted the last element can call this again without checking.
void erase_part(Assembly& assembly, std::size_t index);

/// Moves the part at `index` `delta` places along the stack, positive toward the
/// bottom. A move that would take it off either end does nothing and returns
/// false, which lets a caller stop a cursor at the end rather than wrapping it
/// to the other one.
bool move_part(Assembly& assembly, std::size_t index, int delta);

/// Puts the part at `index` in `stage`. Out of range, or a negative stage, is a
/// no-op: a stage is a number and there is no stage -1 to be in.
void set_part_stage(Assembly& assembly, std::size_t index, int stage);

/// Recomputes every stage number from the stack, with a decoupler as the
/// staging boundary: stage 0 is everything from the bottom up to and including
/// the first decoupler, stage 1 everything up to the next one, and so on.
///
/// The decoupler belongs to the stage it releases, which is why it is counted
/// with the parts below it — the spent booster leaves together with the ring
/// that held it. That is the convention `scenarios/tug.json` is already written
/// in (a `decoupler.large` at stage 0 with the booster, the service stage above
/// it at stage 1), so this reproduces the shipped fixture rather than inventing
/// a second convention beside it.
///
/// What counts as a decoupler is decided by `PartKind`, read out of the
/// catalogue, and not by the part's name.
///
/// It is not cosmetic. Staging is by number and the first firing stage is the
/// lowest one that has parts, so a stack whose booster is numbered 3 flies its
/// upper stage first and drops the wrong half of the rocket. Auto-staging from
/// the decouplers is how a stack built by hand in the editor is kept in the
/// shape the flight model expects. An assembly with no decoupler at all comes
/// out as a single stage, which is the truthful answer: nothing in it separates.
void auto_stage(Assembly& assembly, const PartCatalogue& catalogue);

/// The assembly resolved against the catalogue, ready to fly, with full tanks.
///
/// Throws `std::invalid_argument` for a part the catalogue does not have, the
/// same as `build_vessel`. The editor only ever inserts names it took from the
/// catalogue, so a name that will not resolve means the document was written by
/// something that did not.
[[nodiscard]] Vessel resolve(const Assembly& assembly, const PartCatalogue& catalogue);

}  // namespace rocketlab::core
