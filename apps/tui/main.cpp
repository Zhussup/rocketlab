// The terminal client.
//
// It owns no physics. Everything on screen comes from a snapshot or from a
// query the simulation host answered, and the only value that ever crosses
// back is a wall-clock frame duration. That is what lets the same client later
// talk to a daemon over a socket without changing a line of the drawing code.
//
// Layout: the map takes whatever is left, telemetry and the entity list take a
// fixed column on the right, and the keybar at the bottom says what the keys
// do rather than hiding them in a help screen nobody opens.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <format>
#include <fstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <unistd.h>
#include <vector>

#include <ftxui/component/component.hpp>
#include <ftxui/component/event.hpp>
#include <ftxui/component/screen_interactive.hpp>
#include <ftxui/dom/elements.hpp>
#include <ftxui/screen/box.hpp>

#include "canvas_backend.hpp"
#include "rocketlab/core/assembly.hpp"
#include "rocketlab/core/world.hpp"
#include "rocketlab/hud/assembly.hpp"
#include "rocketlab/hud/editor.hpp"
#include "rocketlab/hud/readout.hpp"
#include "rocketlab/render/scene.hpp"
#include "rocketlab/scenario/json_io.hpp"
#include "rocketlab/simhost/local_sim_source.hpp"

namespace {

namespace core = rocketlab::core;
namespace proto = rocketlab::proto;
namespace render = rocketlab::render;
namespace hud = rocketlab::hud;
namespace simhost = rocketlab::simhost;
namespace scenario_io = rocketlab::scenario;

using namespace ftxui;

/// How often the client redraws, in hertz. Above the terminal's own repaint
/// rate a faster loop just burns cpu on frames nobody sees.
constexpr int kTickHz = 30;

/// Longest predicted path drawn, in seconds of simulation time. Capped so a
/// vessel on a wildly eccentric orbit does not ask the host to sample a path
/// spanning years.
constexpr double kMaxHorizon = 4.0e7;

struct App {
  explicit App(simhost::LocalSimSource source) : host(std::move(source)) {}

  simhost::LocalSimSource host;
  render::Camera2D camera;
  render::SceneOptions scene_options;
  render::Scene scene;

  /// The predicted path of `queried_id`, in the root frame.
  std::vector<proto::Vec3d> trajectory;
  std::uint64_t queried_id{0};
  double queried_tdb{0.0};
  bool has_query{false};

  /// Where the map was drawn last frame. Mouse coordinates arrive in screen
  /// space, so without this a click could not be turned into a map position.
  Box map_box;

  int list_cursor{0};
  bool show_help{false};
  /// Warp to come back to when unpausing.
  double resume_warp{1.0};
  /// Cleared until the map has been framed against a real canvas size. The
  /// camera cannot be framed before the first render, because until then the
  /// viewport is still the placeholder size and the zoom it would pick is
  /// wrong by the ratio between them.
  bool framed{false};

  std::string scenario_name;

  /// What the last flight key did, or why it did nothing, shown in the header.
  /// Cleared by the next keypress rather than by a timer, so a message cannot
  /// disappear while it is being read and cannot outlive the state it describes.
  std::string status;

  /// The scenario the world was built from. The editor edits *this*, and
  /// applying an edit rebuilds the host from it — because the parts a vessel is
  /// made of are a document, not simulation state, and the snapshot rightly
  /// carries only what the vessel is doing. A consequence worth knowing: an
  /// applied edit restarts the mission from the epoch, so the thing to do is
  /// design, apply, and then fly.
  core::Scenario base_scenario;
  core::PartCatalogue catalogue{core::PartCatalogue::stock()};

  /// The stack being edited. The state machine is `hud::Editor`, shared with the
  /// windowed client, so both build the same rocket from the same keystrokes.
  hud::Editor editor;
  /// Where `s` writes. Taken from the scenario path, or `--save` if given.
  std::string save_path;
};

/// Asks the host for the selected entity's path when the answer would
/// actually differ.
///
/// The path is anchored at the current position, so it goes stale as the
/// vessel moves. Re-querying every frame would be wasteful at real time and
/// still too slow at high warp; re-querying once per sample step tracks the
/// motion closely in both cases, because the step shrinks as the horizon does.
void refresh_trajectory(App& app) {
  const proto::Snapshot& snapshot = app.host.snapshot();
  if (snapshot.selected == 0) {
    app.trajectory.clear();
    app.has_query = false;
    return;
  }

  const proto::EntitySnapshot* entity = hud::find_entity(snapshot, snapshot.selected);
  if (entity == nullptr) {
    app.trajectory.clear();
    app.has_query = false;
    return;
  }

  // One full orbit when the orbit closes, otherwise an hour: a hyperbola has
  // no period to sample and drawing a year of it would be noise.
  double horizon = entity->period;
  const bool escaping = proto::has_flag(entity->flags, proto::Flags::Escaping);
  const bool degenerate = proto::has_flag(entity->flags, proto::Flags::Degenerate);
  if (escaping || degenerate || !(horizon > 0.0) || horizon > kMaxHorizon) {
    horizon = 3600.0;
  }

  const bool same_target = app.has_query && app.queried_id == snapshot.selected;
  const double step = horizon / static_cast<double>(simhost::LocalSimSource::kTrajectorySamples);
  if (same_target && std::abs(snapshot.tdb - app.queried_tdb) < step) {
    return;
  }

  if (app.host.query_trajectory(snapshot.selected, horizon, app.trajectory)) {
    app.queried_id = snapshot.selected;
    app.queried_tdb = snapshot.tdb;
    app.has_query = true;
  } else {
    app.trajectory.clear();
    app.has_query = false;
  }
}

[[nodiscard]] Element render_header(const App& app) {
  const proto::Snapshot& snapshot = app.host.snapshot();
  const bool paused = !(snapshot.warp > 0.0);

  Elements items;
  items.push_back(text(app.scenario_name) | bold);
  items.push_back(separator());
  items.push_back(text(core::format_met(snapshot.tdb)));
  items.push_back(separator());
  items.push_back(text(paused ? "PAUSED" : std::format("warp x{:g}", snapshot.warp)) |
                   (paused ? color(Color::Yellow) : color(Color::Green)));

  // Whether the flight computers are being run is a property of the host, not
  // of the selection, so it belongs in the header next to the clock rather than
  // in the telemetry panel. Showing it only when something is switched off
  // keeps the ordinary case quiet.
  if (!app.host.computers_enabled()) {
    items.push_back(separator());
    items.push_back(text("AUTOPILOT OFF") | color(Color::Magenta) | bold);
  }

  // A refusal is the one thing a pilot has to see to act on, and it belongs
  // beside the clock rather than in a panel they might not be looking at.
  if (!app.status.empty()) {
    items.push_back(separator());
    items.push_back(text(app.status) | color(Color::Yellow));
  }
  items.push_back(filler());
  items.push_back(text(std::format("{} entities  {} bodies", snapshot.entity_count,
                                   snapshot.body_count)) |
                   dim);
  return hbox(std::move(items));
}

[[nodiscard]] Element render_entity_list(const App& app) {
  const std::vector<hud::EntityLine> lines = hud::entity_list(app.host.snapshot());

  Elements rows;
  if (lines.empty()) {
    rows.push_back(text("no entities") | dim);
  }
  for (std::size_t i = 0; i < lines.size(); ++i) {
    const hud::EntityLine& line = lines[i];
    const bool cursor = static_cast<int>(i) == app.list_cursor;

    Element row = hbox({
        text(line.controllable ? "* " : "  "),
        text(line.name) | flex,
        text(line.parent) | dim,
    });

    if (line.selected) {
      row = row | color(Color::Cyan) | bold;
    }
    if (cursor) {
      row = row | inverted;
    } else if (line.selected) {
      row = row | bgcolor(Color::RGB(20, 40, 50));
    }
    rows.push_back(std::move(row));
  }

  return vbox(std::move(rows)) | border | size(HEIGHT, LESS_THAN, 12);
}

[[nodiscard]] Element render_readout(const App& app) {
  const proto::Snapshot& snapshot = app.host.snapshot();
  const proto::EntitySnapshot* entity = hud::find_entity(snapshot, snapshot.selected);

  Elements rows;
  if (entity == nullptr) {
    rows.push_back(text("no selection") | dim);
    rows.push_back(text("press tab to pick one") | dim);
  } else {
    rows.push_back(text(std::string(entity->name.view())) | bold);
    rows.push_back(separator());
    for (const hud::Row& row : hud::entity_readout(snapshot, *entity)) {
      Element value = text(row.value);
      if (row.emphasised) {
        value = value | color(Color::Red) | bold;
      }
      rows.push_back(hbox({
          text(row.label) | dim | size(WIDTH, EQUAL, 13),
          std::move(value),
        }));
    }
  }

  return vbox(std::move(rows)) | border | flex;
}

[[nodiscard]] Element render_keybar(const App& app) {
  if (app.editor.open) {
    return hbox({
        text(" assembly editor   n p part   enter insert   backspace remove   ctrl+up/down move   "
             "[ ] stage   u auto-stage   b build & fly   s save   e close ") |
            dim,
        filler(),
    });
  }
  if (app.show_help) {
    return vbox({
               text("keys") | bold,
               text("[ ]   warp down / up          space  pause"),
               text(", .   zoom out / in           tab    next target"),
               text("up/dn move the list           enter  select it"),
               text("f     toggle following        g      toggle grid"),
               text("t     toggle trajectory       l      toggle labels"),
               text("x     drop the current stage  z c    throttle -/+"),
               text("a     toggle the flight computers"),
               text("e     open the assembly editor for the selection"),
               text("d     dock the nearest vessel onto the selection"),
               text("r     reset the view          q      quit"),
           }) |
           border;
  }
  return hbox({
      text(" [ ] warp  , . zoom  tab target  space pause  x stage  z c throttle  a autopilot  "
           "e assembly  d dock  f follow  h help  q quit ") |
          dim,
      filler(),
  });
}

/// Defined below, but the map needs it to frame itself on its first real
/// canvas size.
void reset_view(App& app);

[[nodiscard]] Element render_map(App& app) {
  const proto::Snapshot& snapshot = app.host.snapshot();

  Element map = canvas([&app, &snapshot](Canvas& surface) {
    app.camera.width = surface.width();
    app.camera.height = surface.height();
    app.camera.cell_aspect = 2.0;

    // The very first frame is where the real viewport becomes known. Framing
    // before it exists would pick a zoom from the placeholder size, which is
    // wrong by the ratio between the two, so the initial framing waits here.
    if (!app.framed) {
      reset_view(app);
      app.framed = true;
    }

    app.camera.target = snapshot.selected;
    render::follow_target(snapshot, app.camera);

    refresh_trajectory(app);
    render::build_scene(snapshot, app.camera, app.scene_options, app.trajectory, app.scene);
    rocketlab::tui::paint(app.scene, surface);
  });

  // The box is recorded so a click can be turned into a map coordinate. It is
  // taken inside the border, so the origin is the canvas's own top-left.
  return map | reflect(app.map_box) | border | flex;
}

[[nodiscard]] Element render_assembly_picker(const App& app) {
  const std::vector<hud::PartLine> parts = hud::part_lines(app.catalogue);

  Elements rows;
  for (std::size_t i = 0; i < parts.size(); ++i) {
    const hud::PartLine& line = parts[i];
    Element row = hbox({
        text(line.name) | flex,
        text(line.detail.empty() ? line.mass : line.detail) | dim,
    });
    if (static_cast<int>(i) == app.editor.part_cursor) {
      row = row | inverted;
    }
    rows.push_back(std::move(row));
  }
  return vbox(std::move(rows)) | border;
}

/// The stack, top first, with the cursor on the part an action would touch.
[[nodiscard]] Element render_assembly_stack(const App& app, const std::vector<hud::StackLine>& lines) {
  Elements rows;
  if (lines.empty()) {
    rows.push_back(text("empty") | dim);
  }
  for (std::size_t i = 0; i < lines.size(); ++i) {
    const hud::StackLine& line = lines[i];
    Element row = hbox({
        text(std::format("st{:<2}", line.stage)) | dim,
        text(line.part) | flex,
        text(line.kind) | dim,
    });
    if (static_cast<int>(i) == app.editor.stack_cursor) {
      row = row | inverted;
    }
    rows.push_back(std::move(row));
  }
  return vbox(std::move(rows)) | border;
}

[[nodiscard]] Element render_assembly_numbers(const core::Vessel& vessel) {
  Elements rows;
  for (const hud::Row& row : hud::stack_summary(vessel)) {
    rows.push_back(hbox({
        text(row.label) | dim | size(WIDTH, EQUAL, 13),
        text(row.value),
    }));
  }
  rows.push_back(separator());
  for (const hud::Row& row : hud::stage_rows(vessel)) {
    Element value = text(row.value);
    if (row.emphasised) {
      value = value | color(Color::Red) | bold;
    }
    rows.push_back(hbox({
        text(row.label) | dim | size(WIDTH, EQUAL, 13),
        std::move(value),
    }));
  }
  return vbox(std::move(rows)) | border;
}

/// The editor, which takes the whole body while it is open.
///
/// The map is not shown beside it on purpose. A design is a document with a
/// part list, a stack and a stage table, and squeezing those into a third of a
/// terminal to keep a picture of a rocket that is not flying yet would make the
/// one screen that has to be dense the one that is cramped. `e` puts the map
/// back.
[[nodiscard]] Element render_editor(const App& app) {
  core::Vessel vessel;
  std::string error;
  const bool resolved = app.editor.resolve(app.catalogue, vessel, error);

  Elements left;
  left.push_back(text("parts") | bold);
  left.push_back(render_assembly_picker(app));

  Elements right;
  right.push_back(text(std::format("stack: {}", app.editor.assembly.name)) | bold);
  if (resolved) {
    right.push_back(render_assembly_stack(app, hud::stack_lines(vessel)));
  } else {
    right.push_back(text(error) | color(Color::Red));
  }

  Elements lower;
  if (resolved) {
    lower.push_back(render_assembly_numbers(vessel));
  }
  lower.push_back(
      text(app.editor.status.empty() ? "n p pick a part   enter insert it above the cursor   "
                                      "backspace remove   ctrl+up/down move it   [ ] restage"
                                    : app.editor.status) |
      (app.editor.status.empty() ? dim : color(Color::Yellow)));

  return vbox({
      hbox({
          vbox(std::move(left)) | size(WIDTH, EQUAL, 28),
          vbox(std::move(right)) | flex,
      }) | flex,
      vbox(std::move(lower)),
  });
}

[[nodiscard]] Element render_flight(App& app) {
  return hbox({
      render_map(app),
      vbox({
          render_entity_list(app),
          render_readout(app),
      }) | size(WIDTH, EQUAL, 36),
  }) | flex;
}

[[nodiscard]] Element render_app(App& app) {
  return vbox({
             render_header(app),
             app.editor.open ? render_editor(app) : render_flight(app),
             render_keybar(app),
         }) |
         bgcolor(Color::RGB(12, 14, 20));
}

/// The entity nearest a click, within a generous radius so that a small marker
/// can be hit without pixel hunting.
[[nodiscard]] std::uint64_t entity_at(const App& app, double x, double y) {
  const proto::Snapshot& snapshot = app.host.snapshot();
  std::uint64_t best = 0;
  double best_distance = 3.0;

  const render::Camera2D& camera = app.camera;
  for (std::uint32_t i = 0; i < snapshot.entity_count && i < proto::kMaxEntities; ++i) {
    const render::ScreenPoint at = camera.project(render::root_position(snapshot.entities[i]));
    const double distance = std::hypot(at.x - x, at.y - y);
    if (distance < best_distance) {
      best_distance = distance;
      best = snapshot.entities[i].id;
    }
  }
  return best;
}

void move_cursor(App& app, int direction) {
  const int count = static_cast<int>(hud::entity_list(app.host.snapshot()).size());
  if (count <= 0) {
    app.list_cursor = 0;
    return;
  }
  app.list_cursor = ((app.list_cursor + direction) % count + count) % count;
}

/// Selects whichever entity the list cursor is on.
void select_cursor(App& app) {
  const std::vector<hud::EntityLine> lines = hud::entity_list(app.host.snapshot());
  if (app.list_cursor >= 0 && static_cast<std::size_t>(app.list_cursor) < lines.size()) {
    app.host.send(proto::Command::select(lines[app.list_cursor].id));
  }
}

void adjust_warp(App& app, int direction) {
  app.host.send(proto::Command::step_warp(direction));
  const double warp = app.host.snapshot().warp;
  if (warp > 0.0) {
    app.resume_warp = warp;
  }
}

void toggle_pause(App& app) {
  const double warp = app.host.snapshot().warp;
  if (warp > 0.0) {
    app.resume_warp = warp;
    app.host.send(proto::Command::set_warp(0.0));
  } else {
    app.host.send(proto::Command::set_warp(app.resume_warp > 0.0 ? app.resume_warp : 1.0));
  }
}

/// Nudges the throttle of the selection by a quarter, which is coarse enough to
/// reach either end in four presses and fine enough to trim a circularisation
/// burn by eye. Deliberately not a text entry: a flight computer is what should
/// be setting precise throttles, and that is M3.
void adjust_throttle(App& app, int steps) {
  const proto::Snapshot& snapshot = app.host.snapshot();
  const proto::EntitySnapshot* entity = hud::find_entity(snapshot, snapshot.selected);
  if (entity == nullptr || entity->stage_count == 0) {
    return;
  }
  const double next = entity->throttle + 0.25 * static_cast<double>(steps);
  app.host.send(proto::Command::set_throttle(std::min(1.0, std::max(0.0, next))));
}

/// Docks the nearest vessel the rules allow onto the selection.
///
/// The client answers *whether*, with the predicate the host also uses, and
/// sends only the id: what a docking does to the world is the host's question
/// and stays there. The choice of candidate is `hud::nearest_dock`, so the
/// windowed client's dock key picks the same vessel as this one.
void dock_nearest(App& app) {
  const proto::Snapshot& snapshot = app.host.snapshot();
  const hud::DockOffer offer = hud::nearest_dock(snapshot, snapshot.selected);
  if (offer.id != 0) {
    app.host.send(proto::Command::dock(offer.id));
    app.status = "docking with " + offer.name;
    return;
  }
  if (offer.name.empty()) {
    // Nothing to name: either nothing is selected, or nothing else is in the
    // selection's frame.
    app.status = hud::find_entity(snapshot, snapshot.selected) == nullptr
                     ? "select a vessel before docking"
                     : "nothing else is in this frame";
    return;
  }
  app.status = "cannot dock with " + offer.name + ": " + core::to_string(offer.block);
}

/// Frames the selection: a body-sized object fills a quarter of the view, and
/// something in deep space gets framed by its own orbit instead.
void reset_view(App& app) {
  const proto::Snapshot& snapshot = app.host.snapshot();
  const proto::EntitySnapshot* entity = hud::find_entity(snapshot, snapshot.selected);
  if (entity == nullptr) {
    return;
  }
  const proto::BodySnapshot* parent = hud::find_body(snapshot, entity->parent);
  const double radius = parent != nullptr && parent->radius > 0.0 ? parent->radius
                                                                 : entity->periapsis;
  app.camera.metres_per_pixel = 1.0e3;
  app.camera.frame(render::root_position(*entity), radius > 0.0 ? radius : 1.0e7, 0.25);
}

// --- the assembly editor -----------------------------------------------------
//
// The editor edits the *scenario*, not the world. That is the whole shape of
// it: a stack is a document, the snapshot holds what a vessel is doing rather
// than what it is made of, and applying an edit means rebuilding the host from
// the document. So the three verbs are "design it", "write it down", and "fly
// it", and only the last one restarts the mission.
//
// The state machine itself is `hud::Editor`, which the windowed client drives
// too. What is left here is what is particular to a terminal: which key does
// what, and rebuilding *this* client's host.

/// Opens the editor on whatever is selected, loading its stack out of the
/// scenario document.
///
/// By name and not by id: an entity id is handed out by the world when it is
/// built, so it means nothing inside a document, and after an apply every id is
/// different anyway.
void begin_editing(App& app) {
  const proto::Snapshot& snapshot = app.host.snapshot();
  const proto::EntitySnapshot* entity = hud::find_entity(snapshot, snapshot.selected);
  if (entity == nullptr) {
    // Still opens: an editor with nothing loaded is where a person goes to find
    // out that they have to select something first.
    app.editor = hud::Editor{};
    app.editor.open = true;
    app.editor.status = "nothing is selected, so there is nothing to edit";
    return;
  }
  // The return is the same as the status it leaves behind — an entity that is
  // not in the scenario opens with the stack empty so the panel can still say
  // so — so the value is dropped deliberately.
  (void)app.editor.load(app.base_scenario, entity->name.view());
}

/// Rebuilds the host from the edited document.
///
/// The mission restarts: the world is recreated from the scenario, so the clock
/// goes back to the epoch and every entity returns to its launch state. That is
/// the honest consequence of changing what a vessel is made of — the stack is
/// part of the launch, not something to be swapped out in flight — and it is
/// why the key is a deliberate one rather than something the editor does on its
/// own.
void apply_assembly(App& app) {
  hud::Editor& editor = app.editor;
  if (editor.entity.empty()) {
    editor.status = "nothing is loaded to build";
    return;
  }

  // Resolved before anything is torn down. A stack that will not build costs
  // nothing to refuse, and a half-built world costs the mission.
  core::Vessel vessel;
  std::string error;
  if (!editor.resolve(app.catalogue, vessel, error)) {
    editor.status = std::string("cannot build: ") + error;
    return;
  }
  if (vessel.empty()) {
    editor.status = "an empty stack is not something that can fly";
    return;
  }

  // The document first, so that what was built and what would be saved are the
  // same stack. A refusal below leaves the document edited but the world as it
  // was, which is the recoverable half of getting it wrong.
  if (!editor.write(app.base_scenario)) {
    return;
  }
  const std::string name = editor.entity;

  // Everything the client had set on the host, to be put back on the new one.
  // Warp most of all: an edit made at a thousand times real time should not
  // silently drop the mission back to real time.
  const double warp = app.host.snapshot().warp;
  const bool computers = app.host.computers_enabled();
  const core::Seconds max_step = app.host.max_step();

  try {
    app.host = simhost::LocalSimSource::from_scenario(app.base_scenario);
  } catch (const std::exception& failure) {
    editor.status = std::string("the rebuilt scenario will not load: ") + failure.what();
    return;
  }
  app.host.set_max_step(max_step);
  app.host.set_computers_enabled(computers);
  app.host.send(proto::Command::set_warp(warp));

  // Every id the client was holding described the world that just went away.
  app.trajectory.clear();
  app.has_query = false;
  app.framed = false;
  app.list_cursor = 0;

  // The ids are handed out again from one, so the entity being edited is found
  // by name in the new snapshot.
  const proto::Snapshot& rebuilt = app.host.snapshot();
  for (std::uint32_t i = 0; i < rebuilt.entity_count && i < proto::kMaxEntities; ++i) {
    if (name == rebuilt.entities[i].name.view()) {
      app.host.send(proto::Command::select(rebuilt.entities[i].id));
      app.list_cursor = static_cast<int>(i);
      break;
    }
  }

  editor.status = "built " + name + "; the mission restarted";
  editor.save_armed = false;
}

/// Writes the scenario out with the edited stack in it.
///
/// The document, not the sky: saving does not apply. That separation is the
/// point of having both keys — a design is worth writing down before it is
/// worth flying, and a save that also relaunched would make "keep this, try
/// something else" impossible.
void save_assembly(App& app) {
  hud::Editor& editor = app.editor;
  if (app.save_path.empty()) {
    editor.status = "no path to save to; start with --save <file>";
    return;
  }

  // Overwriting a file that is already there is not a one-keystroke action: the
  // file is usually the one the mission was loaded from.
  const bool exists = std::ifstream(app.save_path).good();
  if (exists && !editor.save_armed) {
    editor.save_armed = true;
    editor.status = "press s again to overwrite " + app.save_path;
    return;
  }
  editor.save_armed = false;

  core::Scenario out = app.base_scenario;
  if (!editor.write(out)) {
    return;
  }

  try {
    const std::string text = scenario_io::write_scenario(out);
    std::ofstream file(app.save_path, std::ios::trunc);
    file << text;
    if (!file) {
      editor.status = "failed while writing " + app.save_path;
      return;
    }
  } catch (const std::exception& failure) {
    editor.status = std::string("cannot write: ") + failure.what();
    return;
  }
  editor.status = "wrote " + app.save_path;
}

bool handle_editor_event(App& app, const Event& event) {
  hud::Editor& editor = app.editor;
  if (event == Event::Character('e') || event == Event::Escape) {
    editor.close();
    return true;
  }
  if (event == Event::Return || event == Event::Character('i')) {
    editor.insert(app.catalogue);
    return true;
  }
  if (event == Event::Backspace || event == Event::Delete) {
    editor.erase();
    return true;
  }
  // Ctrl and the arrows together, because a drag is what moving a part is, and
  // w/x as the spelling for a terminal that cannot send the modified keys.
  if (event == Event::ArrowUpCtrl || event == Event::Character('w')) {
    editor.shift(-1);
    return true;
  }
  if (event == Event::ArrowDownCtrl || event == Event::Character('x')) {
    editor.shift(+1);
    return true;
  }
  if (event == Event::Character('[')) {
    editor.restage(-1);
    return true;
  }
  if (event == Event::Character(']')) {
    editor.restage(+1);
    return true;
  }
  if (event == Event::Character('n')) {
    editor.pick(app.catalogue, +1);
    return true;
  }
  if (event == Event::Character('p')) {
    editor.pick(app.catalogue, -1);
    return true;
  }
  if (event == Event::Character('u')) {
    editor.auto_restage(app.catalogue);
    return true;
  }
  if (event == Event::Character('b')) {
    apply_assembly(app);
    return true;
  }
  if (event == Event::Character('s')) {
    save_assembly(app);
    return true;
  }
  if (event == Event::ArrowUp || event == Event::Character('k')) {
    editor.move_cursor(-1);
    return true;
  }
  if (event == Event::ArrowDown || event == Event::Character('j')) {
    editor.move_cursor(+1);
    return true;
  }
  // Everything else, quitting included, falls through to the flight keys.
  return false;
}

bool handle_event(App& app, Event& event, ScreenInteractive& screen) {
  // A redraw tick. The simulation advances here and nowhere else, so the
  // client's own thread does no stepping and input stays responsive.
  if (event == Event::Custom) {
    app.host.pump(0.0);
    return false;
  }

  if (event.is_mouse()) {
    const Mouse& mouse = event.mouse();
    const bool pressed = mouse.button == Mouse::Left && mouse.motion == Mouse::Pressed;
    const bool wheel = mouse.button == Mouse::WheelUp || mouse.button == Mouse::WheelDown;
    if (!pressed && !wheel) {
      return false;
    }

    if (wheel) {
      // Wheel zoom is anchored on the centre, which is what following gives;
      // panning the cursor off centre would fight the tracking camera.
      app.camera.zoom_by(mouse.button == Mouse::WheelUp ? 1.0 / 1.25 : 1.25);
      return true;
    }

    if (app.map_box.Contain(mouse.x, mouse.y)) {
      const double local_x = static_cast<double>(mouse.x - app.map_box.x_min);
      const double local_y = static_cast<double>(mouse.y - app.map_box.y_min);
      const std::uint64_t hit = entity_at(app, local_x, local_y);
      if (hit != 0) {
        app.host.send(proto::Command::select(hit));
        // Keep the list cursor with the map, or the two would disagree about
        // what "enter" is going to select.
        const std::vector<hud::EntityLine> lines = hud::entity_list(app.host.snapshot());
        for (std::size_t i = 0; i < lines.size(); ++i) {
          if (lines[i].id == hit) {
            app.list_cursor = static_cast<int>(i);
            break;
          }
        }
      }
      return true;
    }
    return false;
  }

  if (app.show_help) {
    // In the help view only the keys that leave it do anything, so that a
    // stray keypress cannot change the simulation while reading.
    if (event == Event::Escape || event == Event::Character('h') ||
        event == Event::Character('q')) {
      app.show_help = false;
      return true;
    }
    return true;
  }

  // The editor has first refusal on a key, so that a stack can be built without
  // the flight keys firing underneath it. It declines anything it does not use
  // — quitting among them — and those fall through to the flight keys below.
  if (app.editor.open && handle_editor_event(app, event)) {
    return true;
  }

  // Cleared here rather than on a timer: the header reports the last thing that
  // happened, and a message that vanished on its own would be one the reader
  // could miss. A keypress is the moment they have stopped reading it.
  app.status.clear();

  if (event == Event::Character('q') || event == Event::Escape) {
    screen.Exit();
    return true;
  }
  if (event == Event::Character('h') || event == Event::Character('?')) {
    app.show_help = true;
    return true;
  }
  if (event == Event::Character(' ') || event == Event::Character('p')) {
    toggle_pause(app);
    return true;
  }
  if (event == Event::Return) {
    select_cursor(app);
    return true;
  }
  if (event == Event::Character('[')) {
    adjust_warp(app, -1);
    return true;
  }
  if (event == Event::Character(']')) {
    adjust_warp(app, +1);
    return true;
  }
  if (event == Event::Character(',')) {
    app.camera.zoom_by(1.25);
    return true;
  }
  if (event == Event::Character('.')) {
    app.camera.zoom_by(1.0 / 1.25);
    return true;
  }
  if (event == Event::Character('f')) {
    app.camera.following = !app.camera.following;
    return true;
  }
  if (event == Event::Character('g')) {
    app.scene_options.show_grid = !app.scene_options.show_grid;
    return true;
  }
  if (event == Event::Character('t')) {
    app.scene_options.show_trajectory = !app.scene_options.show_trajectory;
    return true;
  }
  if (event == Event::Character('l')) {
    app.scene_options.show_labels = !app.scene_options.show_labels;
    return true;
  }
  if (event == Event::Character('r')) {
    reset_view(app);
    return true;
  }
  if (event == Event::Character('a')) {
    // A flight computer is only worth trusting once you have watched it fly,
    // and the only way to watch it is to be able to switch it off and see what
    // the mission does without it. Pausing first makes the difference legible.
    app.host.set_computers_enabled(!app.host.computers_enabled());
    return true;
  }
  if (event == Event::Character('e')) {
    begin_editing(app);
    return true;
  }
  if (event == Event::Character('d')) {
    dock_nearest(app);
    return true;
  }
  if (event == Event::Character('x')) {
    // Staging the selection: the dropped stage becomes debris the world owns,
    // so the mass that went up is still accounted for.
    app.host.send(proto::Command::stage());
    return true;
  }
  if (event == Event::Character('z')) {
    adjust_throttle(app, -1);
    return true;
  }
  if (event == Event::Character('c')) {
    adjust_throttle(app, +1);
    return true;
  }
  if (event == Event::Tab) {
    app.host.send(proto::Command::cycle_target(+1));
    return true;
  }
  if (event == Event::TabReverse) {
    app.host.send(proto::Command::cycle_target(-1));
    return true;
  }
  if (event == Event::ArrowUp || event == Event::Character('k')) {
    move_cursor(app, -1);
    return true;
  }
  if (event == Event::ArrowDown || event == Event::Character('j')) {
    move_cursor(app, +1);
    return true;
  }
  return false;
}

void print_usage() {
  std::puts(R"(rocketlab-tui - terminal client for the space mission simulator

usage:
  rocketlab-tui [scenario.json] [--warp <factor>] [--save <file>]

keys:
  [ ]        warp down / up          space   pause and resume
  , .        zoom out / in           tab     next target
  up down    move the entity list    enter   select from the list
  f          toggle camera following g       toggle the grid
  t          toggle trajectory       l       toggle labels
  x          drop the current stage  z c     throttle down / up
  a          autopilot on / off      d       dock the nearest vessel
  e          assembly editor         r       reset the view
  h          help                    q       quit

assembly editor:
  e escape   leave the editor        enter   insert the highlighted part
  n p        next / previous part    backsp  delete the highlighted part
  up down    move down the stack     ctrl-up/down (or w x)  move the part
  [ ]        stage down / up         u       stage from the decouplers
  b          build (restarts the mission from the epoch)
  s          save to --save, or to the file that was loaded

the mouse wheel zooms, and clicking an object selects it.
)");
}

}  // namespace

int main(int argc, char** argv) {
  std::vector<std::string> args(argv + 1, argv + argc);
  std::string path;
  std::string save_path;
  double warp = 1.0;

  try {
    for (std::size_t i = 0; i < args.size(); ++i) {
      const std::string& arg = args[i];
      if (arg == "-h" || arg == "--help") {
        print_usage();
        return EXIT_SUCCESS;
      }
      if (arg == "--warp") {
        if (i + 1 >= args.size()) {
          throw std::runtime_error("--warp needs a value");
        }
        warp = std::stod(args[++i]);
        continue;
      }
      if (arg == "--save") {
        if (i + 1 >= args.size()) {
          throw std::runtime_error("--save needs a value");
        }
        save_path = args[++i];
        continue;
      }
      if (!arg.empty() && arg[0] == '-') {
        throw std::runtime_error("unknown option '" + arg + "'");
      }
      if (!path.empty()) {
        throw std::runtime_error("more than one scenario file given");
      }
      path = arg;
    }
  } catch (const std::exception& error) {
    std::fprintf(stderr, "rocketlab-tui: %s\n", error.what());
    return EXIT_FAILURE;
  }

  core::Scenario scenario;
  try {
    scenario = path.empty() ? scenario_io::default_scenario() : scenario_io::load_scenario_file(path);
  } catch (const std::exception& error) {
    std::fprintf(stderr, "rocketlab-tui: %s\n", error.what());
    return EXIT_FAILURE;
  }

  if (!isatty(STDOUT_FILENO)) {
    std::fputs("rocketlab-tui: needs a terminal; use the `rocketlab` cli for headless runs\n",
               stderr);
    return EXIT_FAILURE;
  }

  App app{simhost::LocalSimSource::from_scenario(scenario)};
  app.scenario_name = scenario.name.empty() ? std::string("scenario") : scenario.name;
  // The document the editor works on. Kept beside the host rather than read
  // back out of it, because a scenario is what a vessel is *built* from and the
  // snapshot rightly carries only what the vessel is doing.
  app.base_scenario = scenario;
  // Without --save the file that was loaded is where a save goes, so that
  // editing the scenario you started from is one keystroke rather than a path
  // to retype. A default scenario has no file behind it, and saving then says
  // so rather than guessing a name.
  app.save_path = save_path.empty() ? path : save_path;
  app.camera.metres_per_pixel = 1.0e6;
  app.camera.following = true;
  app.host.set_max_step(60.0);
  app.host.send(proto::Command::set_warp(warp));

  ScreenInteractive screen = ScreenInteractive::Fullscreen();

  // The clock is driven by a timer rather than by input, so the simulation
  // runs whether or not a key is pressed. The thread only posts an event; the
  // stepping itself happens on the ui thread, which keeps the host free of
  // any locking.
  std::atomic<bool> ticking{true};
  std::thread ticker([&screen, &ticking] {
    const auto period = std::chrono::milliseconds(1000 / kTickHz);
    while (ticking.load(std::memory_order_relaxed)) {
      std::this_thread::sleep_for(period);
      screen.PostEvent(Event::Custom);
    }
  });

  Component component = Renderer([&app] { return render_app(app); });
  component = CatchEvent(component, [&app, &screen](Event event) {
    return handle_event(app, event, screen);
  });

  screen.Loop(component);

  ticking.store(false, std::memory_order_relaxed);
  ticker.join();
  return EXIT_SUCCESS;
}
