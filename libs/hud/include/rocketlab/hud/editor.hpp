// The assembly editor both clients drive.
//
// A third file beside the snapshot readout and the stack readout, and for the
// same reason as those two: the terminal editor and the windowed one have to
// agree on what "insert above the cursor" does, where the cursor goes when a
// part is deleted, and which stack a save writes. Two copies of that is how one
// client's stack quietly starts meaning something the other's does not — and
// the disagreement would show up as a rocket that flies differently depending
// on which window it was built in.
//
// What stays in each client is only what is genuinely per-medium: the keys, the
// widgets, and the rebuilding of that client's own host. Nothing here touches a
// `World` or a `SimSource`, so a stack under construction is a document and
// never simulation state — rule 5 is untouched, because building a rocket is
// arithmetic over the part catalogue.
//
// It edits a `core::Scenario`, which is the point: a scenario's `parts` list and
// an editor's stack are the same thing, so "design it, then save it" and "load
// it, then fly it" are one document rather than two that have to be converted.

#pragma once

#include <cstddef>
#include <string>
#include <string_view>

#include "rocketlab/core/assembly.hpp"
#include "rocketlab/core/part.hpp"
#include "rocketlab/core/scenario.hpp"
#include "rocketlab/core/vessel.hpp"

namespace rocketlab::hud {

/// A stack being edited, with the cursor positions and the last word about it.
///
/// Plain data and free functions' worth of behaviour: a client holds one of
/// these, draws it, and calls the methods. Nothing here reads a clock, a
/// snapshot or a file.
struct Editor {
  /// The stack itself, top first, exactly the shape a scenario stores.
  core::Assembly assembly;

  /// Which part the operations act on. An index into `assembly.stack`, always
  /// kept inside it.
  int stack_cursor{0};

  /// Which catalogue part a new insert takes. An index into the *picker*, which
  /// is `part_lines`' order and so is not the same list.
  int part_cursor{0};

  /// True while the editor is showing. A client that is showing the editor
  /// should send its keys here first and let the ones this declines fall
  /// through to the flight keys.
  bool open{false};

  /// Set once a save has been asked for over a file that already exists. The
  /// second ask is the confirmation: the target is usually the file the mission
  /// was loaded from, and losing it to one keystroke would be too easy.
  bool save_armed{false};

  /// The scenario entity whose stack is loaded, by name. The name rather than
  /// the id, because an applied edit rebuilds the world and every id the client
  /// was holding then describes a world that no longer exists.
  std::string entity;

  /// What the last action did, or why it did nothing. Empty when there is
  /// nothing to say, which is the ordinary case.
  std::string status;

  /// An empty editor, closed.
  Editor() = default;

  /// Loads the stack of the named entity and opens. False, with `status` set,
  /// when the scenario has no such entity — the stack is then empty and saving
  /// or building it would write a vessel nobody asked for.
  ///
  /// An entity that is a point mass loads as an empty stack rather than being
  /// refused: adding the first part is the thing to do next, and a refusal
  /// would leave no way to do it.
  [[nodiscard]] bool load(const core::Scenario& scenario, std::string_view entity_name);

  /// Closes, forgetting the armed save.
  void close() noexcept;

  /// Moves the cursor along the stack, wrapping at both ends. A stack with
  /// nothing in it puts the cursor at the top and leaves it there.
  void move_cursor(int direction) noexcept;

  /// Pulls the cursor back inside the stack.
  ///
  /// The editor's own operations keep it there, so this is for the case where
  /// something outside did not — a document loaded over the top of it, or a
  /// client that wrote the cursor directly. A line, and it turns a key that
  /// would silently do nothing into one that acts on the end of the stack.
  void settle() noexcept;

  /// Inserts the picked catalogue part above the cursor and leaves the cursor on
  /// it.
  ///
  /// Above, in a top-first stack, means at a lower index, so the new part takes
  /// the cursor's place and everything below shifts one toward the booster. The
  /// stage it joins is the one it is bolted onto, which is what makes a second
  /// stage something a person builds by putting a decoupler in rather than by
  /// typing numbers.
  void insert(const core::PartCatalogue& catalogue);

  /// Removes the part under the cursor and keeps the cursor inside the stack.
  void erase() noexcept;

  /// Moves the cursor's part along the stack, positive toward the bottom. A
  /// move off either end does nothing: the cursor stops at the end rather than
  /// wrapping and letting the part jump past it.
  void shift(int delta) noexcept;

  /// Moves the cursor's part up or down one stage number.
  void restage(int delta) noexcept;

  /// Moves the picker cursor, wrapping.
  void pick(const core::PartCatalogue& catalogue, int delta) noexcept;

  /// Renumbers every stage from the decouplers. The one key that keeps a
  /// hand-built stack in the shape the flight model expects.
  void auto_restage(const core::PartCatalogue& catalogue) noexcept;

  /// The stack resolved against the catalogue, with full tanks.
  ///
  /// Reports rather than throws: the editor has to draw a stage table for a
  /// stack that will not build, because that is exactly the stack a person is
  /// trying to fix. `error` is left alone on success.
  [[nodiscard]] bool resolve(const core::PartCatalogue& catalogue, core::Vessel& out,
                             std::string& error) const;

  /// Writes the edited stack into the named entity of `scenario`, leaving the
  /// rest of the document alone. False, with `status` set, when there is no such
  /// entity.
  ///
  /// The document only: this does not build, launch or reload anything. A design
  /// is worth writing down before it is worth flying, and an edit that saved and
  /// relaunched at once would make "keep this and try something else"
  /// impossible.
  [[nodiscard]] bool write(core::Scenario& scenario) noexcept;

  /// Disarms a pending save. Called by every edit, so that confirming an
  /// overwrite cannot carry over to a different stack.
  void touch() noexcept { save_armed = false; }
};

/// The entity with this name, or null. `Scenario` holds a vector rather than a
/// map because it is a document read in order, and a document has no index.
[[nodiscard]] core::ScenarioEntity* find_scenario_entity(core::Scenario& scenario,
                                                         std::string_view name) noexcept;

}  // namespace rocketlab::hud
