// The graphical client.
//
// It is the terminal client's twin, and deliberately so: the same
// `render::Camera3D`, the same `render::build_scene`, the same `hud` readout and
// `hud::Editor`, the same `SimSource`. The only difference between the two
// pictures is the medium — a terminal draws characters into cells, this one
// draws primitives into an ImDrawList — and the only thing this file adds is the
// window around them. Anything that differed here would be a bug in the shared
// layers, which is exactly the property the scene abstraction was built for.
//
// The map is the 3D camera at zero pitch, so the flat view is not a second
// renderer: tilting is a change of angle, not of code, and an object cannot sit
// somewhere different in one than in the other.
//
// Like the TUI it owns no physics and reads no simulation state directly. The
// clock it does read is the frame timer, and only to make the camera glide at a
// speed a person can follow rather than at the simulation's warp factor.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <format>
#include <fstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include "imgui_backend.hpp"
#include "rocketlab/core/assembly.hpp"
#include "rocketlab/core/world.hpp"
#include "rocketlab/hud/assembly.hpp"
#include "rocketlab/hud/editor.hpp"
#include "rocketlab/hud/readout.hpp"
#include "rocketlab/render/camera3d.hpp"
#include "rocketlab/render/scene.hpp"
#include "rocketlab/scenario/json_io.hpp"
#include "rocketlab/simhost/local_sim_source.hpp"

namespace {

namespace core = rocketlab::core;
namespace gui = rocketlab::gui;
namespace hud = rocketlab::hud;
namespace proto = rocketlab::proto;
namespace render = rocketlab::render;
namespace simhost = rocketlab::simhost;
namespace scenario_io = rocketlab::scenario;

using namespace rocketlab;

/// Longest predicted path drawn, in seconds of simulation time. The same cap the
/// terminal client uses, and for the same reason: a vessel on a wildly eccentric
/// orbit would otherwise ask the host to sample a path spanning years.
constexpr double kMaxHorizon = 4.0e7;

/// Width of the side panel, in pixels.
constexpr float kPanelWidth = 340.0f;

/// The tilt the `v` key switches to, in radians. A little under 32 degrees:
/// enough that orbits read as ellipses seen at an angle rather than as lines,
/// shallow enough to keep the inner solar system inside one screen.
constexpr double kTilt = 0.55;

/// Radians of yaw and pitch per pixel of a right-drag. A full turn is about one
/// window width of dragging, which is the rate a person expects from an orbit.
constexpr double kOrbitPerPixel = 0.008;

struct App {
  explicit App(simhost::LocalSimSource source) : host(std::move(source)) {}

  simhost::LocalSimSource host;
  render::Camera3D camera;
  render::CameraEase ease;
  render::SceneOptions scene_options;
  render::Scene scene;

  /// The predicted path of the selection, in its parent body's frame.
  std::vector<proto::Vec3d> trajectory;
  std::uint64_t queried_id{0};
  double queried_tdb{0.0};
  bool has_query{false};

  /// Size of the map region in pixels, which is what the camera projects into.
  int map_width{1};
  int map_height{1};
  bool framed{false};

  int list_cursor{0};
  bool show_help{false};
  std::string scenario_name;
  std::string status;  // the last thing the client itself had to say

  /// The scenario the world was built from, and the catalogue the editor reads.
  /// The editor edits the *document*: a stack is what a vessel is built from,
  /// and the snapshot rightly carries only what the vessel is doing.
  core::Scenario base_scenario;
  core::PartCatalogue catalogue{core::PartCatalogue::stock()};

  /// The stack being edited. The state machine is `hud::Editor`, shared with the
  /// terminal client, so both build the same rocket from the same operations.
  hud::Editor editor;
  std::string save_path;
};

[[nodiscard]] const proto::EntitySnapshot* selected_entity(const App& app) {
  const proto::Snapshot& snapshot = app.host.snapshot();
  return hud::find_entity(snapshot, snapshot.selected);
}

/// Asks the host for the selected entity's path when the answer would differ.
///
/// The same policy as the terminal client: the path is anchored at the current
/// position, so it goes stale as the vessel moves, and re-querying once per
/// sample step keeps it glued to the motion without asking every frame.
void refresh_trajectory(App& app) {
  const proto::Snapshot& snapshot = app.host.snapshot();
  const proto::EntitySnapshot* entity = selected_entity(app);
  if (entity == nullptr) {
    app.trajectory.clear();
    app.has_query = false;
    return;
  }

  double horizon = entity->period;
  const bool escaping = proto::has_flag(entity->flags, proto::Flags::Escaping);
  const bool degenerate = proto::has_flag(entity->flags, proto::Flags::Degenerate);
  if (escaping || degenerate || !(horizon > 0.0) || horizon > kMaxHorizon) {
    horizon = 3600.0;
  }

  const int samples = static_cast<int>(simhost::LocalSimSource::kTrajectorySamples);
  const double anchor = horizon / (4.0 * samples);
  const bool stale = !app.has_query || app.queried_id != snapshot.selected ||
                     !(snapshot.tdb - app.queried_tdb < anchor);
  if (!stale) {
    return;
  }

  app.trajectory.clear();
  if (app.host.query_trajectory(snapshot.selected, horizon, app.trajectory)) {
    app.queried_id = snapshot.selected;
    app.queried_tdb = snapshot.tdb;
    app.has_query = true;
  } else {
    app.has_query = false;
  }
}

/// Frames the selection: a body-sized object fills a quarter of the view, and
/// something in deep space is framed by its own orbit instead.
void reset_view(App& app) {
  const proto::Snapshot& snapshot = app.host.snapshot();
  const proto::EntitySnapshot* entity = selected_entity(app);
  if (entity == nullptr) {
    return;
  }
  const proto::BodySnapshot* parent = hud::find_body(snapshot, entity->parent);
  const double radius = parent != nullptr && parent->radius > 0.0 ? parent->radius
                                                                  : entity->periapsis;
  const proto::Vec3d at = render::root_position(*entity);
  app.camera.frame(at, radius > 0.0 ? radius : 1.0e7, 0.25);
  // The glide is cancelled rather than left to travel: "reset the view" that
  // then spent a second drifting to where it was asked to be would be a strange
  // thing to watch.
  app.ease.snap(at);
}

void select_id(App& app, std::uint64_t id) {
  app.host.send(proto::Command::select(id));
  const std::vector<hud::EntityLine> lines = hud::entity_list(app.host.snapshot());
  for (std::size_t i = 0; i < lines.size(); ++i) {
    if (lines[i].id == id) {
      app.list_cursor = static_cast<int>(i);
      break;
    }
  }
}

void adjust_throttle(App& app, double steps) {
  const proto::EntitySnapshot* entity = selected_entity(app);
  if (entity == nullptr || entity->stage_count == 0) {
    return;
  }
  const double next = entity->throttle + 0.25 * steps;
  app.host.send(proto::Command::set_throttle(std::min(1.0, std::max(0.0, next))));
}

/// The entity nearest a click, within a few pixels so a small marker can be hit
/// without pixel hunting.
///
/// Nearest in *depth* among those under the pointer, once the view is tilted:
/// two markers can overlap on screen with one behind the other, and the one a
/// person means is the one they can see. With the pitch at zero every depth is
/// a tie and this is the plain distance, exactly as in the flat map.
[[nodiscard]] std::uint64_t entity_at(const App& app, double x, double y) {
  const proto::Snapshot& snapshot = app.host.snapshot();
  const render::View view = app.camera.view();
  std::uint64_t best = 0;
  double best_distance = 12.0;
  double best_depth = 0.0;

  for (std::uint32_t i = 0; i < snapshot.entity_count && i < proto::kMaxEntities; ++i) {
    const proto::Vec3d root = render::root_position(snapshot.entities[i]);
    const render::ScreenPoint at = view.project(root);
    const double distance = std::hypot(at.x - x, at.y - y);
    if (distance > 12.0) {
      continue;
    }
    const double depth = view.depth(root);
    // Strictly nearer, or as near and closer to the pointer: without the second
    // half, a marker exactly on top of another would win on storage order.
    const bool better = best == 0 || distance < best_distance ||
                        (distance == best_distance && depth < best_depth);
    if (better) {
      best_distance = distance;
      best_depth = depth;
      best = snapshot.entities[i].id;
    }
  }
  return best;
}

/// Turns a drag into a motion of the view centre, in the plane the screen is
/// showing. Written against the camera's own axes rather than against X and Y,
/// so a tilted view pans along the ground it is looking at instead of along the
/// ecliptic, which would slide everything sideways as the tilt changed.
void pan_by_pixels(App& app, double dx, double dy) {
  const render::View view = app.camera.view();
  const double aspect = view.cell_aspect > 0.0 ? view.cell_aspect : 1.0;
  const double across = dx * app.camera.metres_per_pixel / aspect;
  const double along = -dy * app.camera.metres_per_pixel;
  app.camera.center.x -= view.right.x * across + view.up.x * along;
  app.camera.center.y -= view.right.y * across + view.up.y * along;
  app.camera.center.z -= view.right.z * across + view.up.z * along;
  app.ease.snap(app.camera.center);
}

/// Stops following, saying so once. Both the pan and the orbit need this, and
/// the message is the only place the client explains why the camera stopped
/// tracking.
void release_follow(App& app) {
  if (app.camera.following) {
    app.camera.following = false;
    app.status = "free view (f to follow again)";
  }
}

/// One key, one action. Reads better as a table than as a chain of `if`s, and it
/// is the whole of the client's input handling outside the map itself.
struct KeyBinding {
  ImGuiKey key;
  const char* key_name;
  const char* action;
};

constexpr KeyBinding kBindings[] = {
    {ImGuiKey_Space, "space", "pause"},
    {ImGuiKey_LeftBracket, "[", "warp down"},
    {ImGuiKey_RightBracket, "]", "warp up"},
    {ImGuiKey_Tab, "tab", "next target"},
    {ImGuiKey_X, "x", "drop the stage"},
    {ImGuiKey_Z, "z", "throttle down"},
    {ImGuiKey_C, "c", "throttle up"},
    {ImGuiKey_A, "a", "flight computers on/off"},
    {ImGuiKey_D, "d", "dock the nearest vessel"},
    {ImGuiKey_E, "e", "assembly editor"},
    {ImGuiKey_V, "v", "flat / tilted view"},
    {ImGuiKey_F, "f", "follow the selection"},
    {ImGuiKey_G, "g", "grid"},
    {ImGuiKey_T, "t", "trajectory"},
    {ImGuiKey_L, "l", "labels"},
    {ImGuiKey_R, "r", "reset the view"},
};

/// Docks the nearest vessel the rules allow onto the selection.
///
/// `hud::nearest_dock` picks the candidate, so this key offers the same vessel
/// the terminal client's does. The client answers *whether*; what a docking does
/// to the world is the host's question, and this sends it only an id.
void dock_nearest(App& app) {
  const proto::Snapshot& snapshot = app.host.snapshot();
  const hud::DockOffer offer = hud::nearest_dock(snapshot, snapshot.selected);
  if (offer.id != 0) {
    app.host.send(proto::Command::dock(offer.id));
    app.status = "docking with " + offer.name;
    return;
  }
  if (offer.name.empty()) {
    app.status = selected_entity(app) == nullptr ? "select a vessel before docking"
                                                 : "nothing else is in this frame";
    return;
  }
  app.status = "cannot dock with " + offer.name + ": " + core::to_string(offer.block);
}

/// Opens the editor on whatever is selected.
///
/// By name and not by id: an id is handed out when the world is built, so it
/// means nothing inside a document, and after a build every id is different
/// anyway.
void begin_editing(App& app) {
  const proto::EntitySnapshot* entity = selected_entity(app);
  if (entity == nullptr) {
    app.editor = hud::Editor{};
    app.editor.open = true;
    app.editor.status = "nothing is selected, so there is nothing to edit";
    return;
  }
  (void)app.editor.load(app.base_scenario, entity->name.view());
}

/// Rebuilds the host from the edited document.
///
/// The mission restarts: the world is recreated from the scenario, so the clock
/// returns to the epoch and every entity to its launch state. That is the honest
/// consequence of changing what a vessel is made of — a stack is part of the
/// launch, not something to swap out in flight — which is why it is a button
/// rather than something the editor does as it goes.
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
  // same stack.
  if (!editor.write(app.base_scenario)) {
    return;
  }
  const std::string name = editor.entity;

  // Everything the client had set on the host, to be put back on the new one.
  // Warp most of all: an edit made at a thousand times real time should not
  // silently drop the mission back to real time.
  const double warp = app.host.snapshot().warp;
  const bool computers = app.host.computers_enabled();

  try {
    app.host = simhost::LocalSimSource::from_scenario(app.base_scenario);
  } catch (const std::exception& failure) {
    editor.status = std::string("the rebuilt scenario will not load: ") + failure.what();
    return;
  }
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
/// The document, not the sky: saving does not build. That separation is the
/// point of having both — a design is worth writing down before it is worth
/// flying, and a save that also relaunched would make "keep this and try
/// something else" impossible.
void save_assembly(App& app) {
  hud::Editor& editor = app.editor;
  if (app.save_path.empty()) {
    editor.status = "no path to save to; start with --save <file>";
    return;
  }

  // Overwriting a file that is already there is not a one-keystroke action: the
  // target is usually the file the mission was loaded from.
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

/// True when a flight key was pressed, so the editor's own keys can be tried
/// first and the flight keys cannot fire underneath an open panel.
[[nodiscard]] bool editor_key(App& app) {
  hud::Editor& editor = app.editor;
  if (ImGui::IsKeyPressed(ImGuiKey_Escape, false) || ImGui::IsKeyPressed(ImGuiKey_E, false)) {
    editor.close();
    return true;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_Enter, false) || ImGui::IsKeyPressed(ImGuiKey_I, false)) {
    editor.insert(app.catalogue);
    return true;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_Delete, false) || ImGui::IsKeyPressed(ImGuiKey_Backspace, false)) {
    editor.erase();
    return true;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_UpArrow, false)) {
    editor.move_cursor(-1);
    return true;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_DownArrow, false)) {
    editor.move_cursor(+1);
    return true;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_LeftBracket, false)) {
    editor.restage(-1);
    return true;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_RightBracket, false)) {
    editor.restage(+1);
    return true;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_N, false)) {
    editor.pick(app.catalogue, +1);
    return true;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_P, false)) {
    editor.pick(app.catalogue, -1);
    return true;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_U, false)) {
    editor.auto_restage(app.catalogue);
    return true;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_B, false)) {
    apply_assembly(app);
    return true;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_S, false)) {
    save_assembly(app);
    return true;
  }
  return false;
}

void handle_keys(App& app) {
  // ImGui's own input goes through the same keys, so a shortcut is only taken
  // when nothing is being typed into.
  if (ImGui::GetIO().WantTextInput) {
    return;
  }

  // The editor has first refusal, so that a stack can be built without the
  // flight keys firing underneath the panel.
  if (app.editor.open && editor_key(app)) {
    return;
  }

  if (ImGui::IsKeyPressed(ImGuiKey_Q, false) || ImGui::IsKeyPressed(ImGuiKey_Escape, false)) {
    glfwSetWindowShouldClose(glfwGetCurrentContext(), GLFW_TRUE);
    return;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_H, false) || ImGui::IsKeyPressed(ImGuiKey_Slash, false)) {
    app.show_help = !app.show_help;
    return;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_Space, false)) {
    const double warp = app.host.snapshot().warp;
    app.host.send(proto::Command::set_warp(warp > 0.0 ? 0.0 : 1.0));
    return;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_LeftBracket, false)) {
    app.host.send(proto::Command::step_warp(-1));
    return;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_RightBracket, false)) {
    app.host.send(proto::Command::step_warp(+1));
    return;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_Tab, false)) {
    const bool back = ImGui::GetIO().KeyShift;
    app.host.send(proto::Command::cycle_target(back ? -1 : +1));
    return;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_X, false)) {
    app.host.send(proto::Command::stage());
    return;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_Z, false)) {
    adjust_throttle(app, -1.0);
    return;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_C, false)) {
    adjust_throttle(app, +1.0);
    return;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_A, false)) {
    app.host.set_computers_enabled(!app.host.computers_enabled());
    return;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_D, false)) {
    dock_nearest(app);
    return;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_E, false)) {
    begin_editing(app);
    return;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_V, false)) {
    // The flat map and the tilted view are one camera at two pitches, so
    // switching is not a change of renderer and nothing moves in the world.
    app.camera.pitch = app.camera.is_top_down() ? kTilt : 0.0;
    app.status = app.camera.is_top_down() ? "flat view" : "tilted view (drag the right button)";
    return;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_F, false)) {
    app.camera.following = !app.camera.following;
    if (app.camera.following) {
      // Resuming the follow picks up from wherever the camera was left, so the
      // ease is snapped to it rather than to the target: gliding *and* panning
      // at the same time is a movement with two causes and no obvious end.
      app.ease.snap(app.camera.center);
      app.status.clear();
    } else {
      app.status = "free view (f to follow again)";
    }
    return;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_G, false)) {
    app.scene_options.show_grid = !app.scene_options.show_grid;
    return;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_T, false)) {
    app.scene_options.show_trajectory = !app.scene_options.show_trajectory;
    return;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_L, false)) {
    app.scene_options.show_labels = !app.scene_options.show_labels;
    return;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_R, false)) {
    reset_view(app);
    return;
  }
}

/// Mouse handling for the map: wheel zooms about the cursor, left-drag pans,
/// right-drag orbits, a click selects. Only called while the pointer is over
/// the map region.
void handle_map_mouse(App& app, const ImVec2& origin) {
  ImGuiIO& io = ImGui::GetIO();

  if (io.MouseWheel != 0.0f) {
    // Zooming about the cursor rather than the centre: the object under the
    // pointer is the one being examined, and moving it out from under the
    // pointer to keep the centre still is the wrong trade.
    const ImVec2 mouse = ImVec2(io.MousePos.x - origin.x, io.MousePos.y - origin.y);
    const proto::Vec3d before = app.camera.unproject(mouse.x, mouse.y);
    app.camera.zoom_by(io.MouseWheel > 0.0f ? 1.0 / 1.25 : 1.25);
    release_follow(app);
    const proto::Vec3d after = app.camera.unproject(mouse.x, mouse.y);
    app.camera.center.x += before.x - after.x;
    app.camera.center.y += before.y - after.y;
    app.camera.center.z += before.z - after.z;
    app.ease.snap(app.camera.center);
  }

  if (ImGui::IsMouseDragging(ImGuiMouseButton_Right, 0.0f)) {
    // The right button turns the view and the left one moves it. Two gestures
    // rather than one, because a drag that both panned and rotated would make
    // "keep that object under the pointer" impossible to do.
    const ImVec2 delta = io.MouseDelta;
    if (delta.x != 0.0f || delta.y != 0.0f) {
      release_follow(app);
      app.camera.orbit(-static_cast<double>(delta.x) * kOrbitPerPixel,
                       static_cast<double>(delta.y) * kOrbitPerPixel);
    }
  } else if (ImGui::IsMouseDragging(ImGuiMouseButton_Left, 0.0f)) {
    const ImVec2 delta = io.MouseDelta;
    if (delta.x != 0.0f || delta.y != 0.0f) {
      release_follow(app);
      pan_by_pixels(app, static_cast<double>(delta.x), static_cast<double>(delta.y));
    }
  } else if (ImGui::IsMouseReleased(ImGuiMouseButton_Left) &&
             ImGui::GetIO().MouseDragMaxDistanceSqr[0] < 16.0f) {
    // A press that did not move is a selection.
    const ImVec2 mouse = ImVec2(io.MousePos.x - origin.x, io.MousePos.y - origin.y);
    const std::uint64_t hit = entity_at(app, mouse.x, mouse.y);
    if (hit != 0) {
      select_id(app, hit);
    }
  }
}

void draw_map(App& app, double wall_dt) {
  const proto::Snapshot& snapshot = app.host.snapshot();

  // Placed and sized explicitly, because ImGui's automatic layout would give
  // the map a default-sized floating window and the camera would then be framed
  // against that instead of the screen. The map takes everything the panel does
  // not.
  const ImVec2 display = ImGui::GetIO().DisplaySize;
  ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f));
  ImGui::SetNextWindowSize(ImVec2(std::max(1.0f, display.x - kPanelWidth), display.y));

  ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.047f, 0.055f, 0.078f, 1.0f));
  ImGui::Begin("map", nullptr,
               ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                   ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar |
                   ImGuiWindowFlags_NoScrollWithMouse | ImGuiWindowFlags_NoBringToFrontOnFocus);

  const ImVec2 size = ImGui::GetContentRegionAvail();
  const ImVec2 origin = ImGui::GetCursorScreenPos();
  app.map_width = std::max(1, static_cast<int>(size.x));
  app.map_height = std::max(1, static_cast<int>(size.y));

  if (ImGui::IsWindowHovered()) {
    handle_map_mouse(app, origin);
  }

  app.camera.width = app.map_width;
  app.camera.height = app.map_height;
  // Pixels are square, so the cell aspect that makes a terminal map come out
  // round is 1 here.
  app.camera.cell_aspect = 1.0;
  app.camera.target = snapshot.selected;

  if (!app.framed && app.map_width > 1 && app.map_height > 1) {
    reset_view(app);
    app.framed = true;
  }

  // The camera glides rather than cuts, at a rate measured in *wall* time. A
  // glide paced by the simulation clock would be instantaneous at 100000x, and
  // the whole point of easing a selection change is that a person watches it
  // happen.
  const proto::EntitySnapshot* entity = selected_entity(app);
  if (app.camera.following && entity != nullptr) {
    const proto::Vec3d want = render::root_position(*entity);
    app.camera.center_on(app.ease.step(want, wall_dt));
  }

  refresh_trajectory(app);
  render::build_scene(snapshot, app.camera.view(), app.scene_options, app.trajectory, app.scene);
  gui::paint(app.scene, ImGui::GetWindowDrawList(), origin);

  if (!app.camera.following) {
    ImGui::GetWindowDrawList()->AddText(ImVec2(origin.x + 8.0f, origin.y + 8.0f),
                                        IM_COL32(255, 220, 120, 255), "free view - f to follow");
  } else if (!app.camera.is_top_down()) {
    ImGui::GetWindowDrawList()->AddText(ImVec2(origin.x + 8.0f, origin.y + 8.0f),
                                        IM_COL32(150, 190, 255, 255), "tilted - v for the flat map");
  }

  ImGui::Dummy(size);
  ImGui::End();
  ImGui::PopStyleColor();
}

void draw_telemetry(App& app) {
  const proto::Snapshot& snapshot = app.host.snapshot();

  ImGui::SetNextWindowPos(ImVec2(static_cast<float>(ImGui::GetIO().DisplaySize.x) - kPanelWidth, 0.0f));
  ImGui::SetNextWindowSize(ImVec2(kPanelWidth, ImGui::GetIO().DisplaySize.y));
  ImGui::Begin("telemetry", nullptr,
               ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                   ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoBringToFrontOnFocus);

  ImGui::TextUnformatted(app.scenario_name.c_str());
  ImGui::SameLine();
  ImGui::TextDisabled("%s", core::format_met(snapshot.tdb).c_str());
  ImGui::SameLine();
  ImGui::TextDisabled("%zu entities", static_cast<std::size_t>(snapshot.entity_count));
  if (snapshot.warp > 0.0) {
    ImGui::TextColored(ImVec4(0.45f, 0.9f, 0.5f, 1.0f), "warp x%g", snapshot.warp);
  } else {
    ImGui::TextColored(ImVec4(0.95f, 0.85f, 0.35f, 1.0f), "PAUSED");
  }
  ImGui::SameLine();
  if (app.host.computers_enabled()) {
    ImGui::TextDisabled("%zu computers", app.host.computers().size());
  } else {
    ImGui::TextColored(ImVec4(0.85f, 0.45f, 0.95f, 1.0f), "AUTOPILOT OFF");
  }

  if (!app.status.empty()) {
    ImGui::TextDisabled("%s", app.status.c_str());
  }
  ImGui::Separator();

  const std::vector<hud::EntityLine> lines = hud::entity_list(snapshot);
  if (ImGui::BeginChild("entities", ImVec2(0.0f, 120.0f), ImGuiChildFlags_Border)) {
    for (std::size_t i = 0; i < lines.size(); ++i) {
      const hud::EntityLine& line = lines[i];
      if (ImGui::Selectable(
              std::format("{}{}{}", line.controllable ? "* " : "  ", line.name,
                          line.selected ? "  <-" : "")
                  .c_str(),
              line.selected)) {
        select_id(app, line.id);
      }
      if (static_cast<int>(i) == app.list_cursor) {
        ImGui::SetItemDefaultFocus();
      }
    }
  }
  ImGui::EndChild();

  const proto::EntitySnapshot* entity = selected_entity(app);
  if (entity == nullptr) {
    ImGui::TextDisabled("no selection - tab to pick one");
    ImGui::End();
    return;
  }

  ImGui::SeparatorText(std::string(entity->name.view()).c_str());
  for (const hud::Row& row : hud::entity_readout(snapshot, *entity)) {
    if (row.emphasised) {
      ImGui::TextColored(ImVec4(1.0f, 0.42f, 0.38f, 1.0f), "%-13s %s", row.label.c_str(),
                         row.value.c_str());
    } else {
      ImGui::TextDisabled("%-13s", row.label.c_str());
      ImGui::SameLine();
      ImGui::TextUnformatted(row.value.c_str());
    }
  }

  ImGui::End();
}

/// The assembly editor, as a window over the map.
///
/// Over rather than beside: the stack, the part picker and the stage table are
/// three tables a person reads against each other, and the map behind it is a
/// picture of a rocket that is not flying yet. The window is closed with `e`,
/// which is also what opened it.
void draw_assembly(App& app) {
  if (!app.editor.open) {
    return;
  }
  hud::Editor& editor = app.editor;

  ImGui::SetNextWindowPos(ImVec2(60.0f, 60.0f), ImGuiCond_Appearing);
  ImGui::SetNextWindowSize(ImVec2(640.0f, 520.0f), ImGuiCond_Appearing);
  ImGui::Begin("assembly", &editor.open);

  core::Vessel vessel;
  std::string error;
  const bool resolved = editor.resolve(app.catalogue, vessel, error);

  // The picker and the stack side by side, because a part is chosen and then
  // placed, and the two halves of that are what the eye moves between.
  const float half = (ImGui::GetContentRegionAvail().x - 12.0f) * 0.5f;

  ImGui::BeginChild("parts", ImVec2(half, 220.0f), ImGuiChildFlags_Border);
  ImGui::SeparatorText("parts");
  const std::vector<hud::PartLine> parts = hud::part_lines(app.catalogue);
  for (std::size_t i = 0; i < parts.size(); ++i) {
    const hud::PartLine& line = parts[i];
    if (ImGui::Selectable(std::format("{}  {}", line.name, line.mass).c_str(),
                          static_cast<int>(i) == editor.part_cursor)) {
      editor.part_cursor = static_cast<int>(i);
    }
    if (ImGui::IsItemHovered() && !line.detail.empty()) {
      ImGui::SetTooltip("%s", line.detail.c_str());
    }
  }
  ImGui::EndChild();

  ImGui::SameLine();
  ImGui::BeginChild("stack", ImVec2(0.0f, 220.0f), ImGuiChildFlags_Border);
  ImGui::SeparatorText(
      std::format("stack: {}", editor.assembly.name.empty() ? "untitled" : editor.assembly.name)
          .c_str());
  if (resolved) {
    const std::vector<hud::StackLine> lines = hud::stack_lines(vessel);
    if (lines.empty()) {
      ImGui::TextDisabled("empty - insert a part to start");
    }
    for (std::size_t i = 0; i < lines.size(); ++i) {
      const hud::StackLine& line = lines[i];
      if (ImGui::Selectable(std::format("st{:<2}  {}  {}", line.stage, line.part, line.kind).c_str(),
                            static_cast<int>(i) == editor.stack_cursor)) {
        editor.stack_cursor = static_cast<int>(i);
      }
    }
  } else {
    ImGui::TextColored(ImVec4(1.0f, 0.42f, 0.38f, 1.0f), "%s", error.c_str());
  }
  ImGui::EndChild();

  // The verbs, as buttons, so that a mouse alone can build a rocket. The keys
  // are on the labels because that is where a person looks first.
  if (ImGui::Button("insert (enter)")) {
    editor.insert(app.catalogue);
  }
  ImGui::SameLine();
  if (ImGui::Button("remove (del)")) {
    editor.erase();
  }
  ImGui::SameLine();
  if (ImGui::Button("move up")) {
    editor.shift(-1);
  }
  ImGui::SameLine();
  if (ImGui::Button("move down")) {
    editor.shift(+1);
  }
  ImGui::SameLine();
  if (ImGui::Button("stage -")) {
    editor.restage(-1);
  }
  ImGui::SameLine();
  if (ImGui::Button("stage +")) {
    editor.restage(+1);
  }
  ImGui::SameLine();
  if (ImGui::Button("auto-stage (u)")) {
    editor.auto_restage(app.catalogue);
  }

  if (resolved) {
    const float third = (ImGui::GetContentRegionAvail().x - 12.0f) / 3.0f;
    ImGui::BeginChild("weights", ImVec2(third, 150.0f), ImGuiChildFlags_Border);
    for (const hud::Row& row : hud::stack_summary(vessel)) {
      ImGui::TextDisabled("%s", row.label.c_str());
      ImGui::SameLine();
      ImGui::TextUnformatted(row.value.c_str());
    }
    ImGui::EndChild();

    ImGui::SameLine();
    ImGui::BeginChild("stages", ImVec2(0.0f, 150.0f), ImGuiChildFlags_Border);
    ImGui::SeparatorText("stages");
    for (const hud::Row& row : hud::stage_rows(vessel)) {
      if (row.emphasised) {
        // An engine with no propellant in its own stage is the one mistake this
        // table exists to catch, so it is the one row that shouts.
        ImGui::TextColored(ImVec4(1.0f, 0.42f, 0.38f, 1.0f), "%s  %s", row.label.c_str(),
                           row.value.c_str());
      } else {
        ImGui::TextDisabled("%s", row.label.c_str());
        ImGui::SameLine();
        ImGui::TextUnformatted(row.value.c_str());
      }
    }
    ImGui::EndChild();
  }

  ImGui::Separator();
  if (ImGui::Button("build and fly (b)")) {
    apply_assembly(app);
  }
  ImGui::SameLine();
  if (ImGui::Button("save (s)")) {
    save_assembly(app);
  }
  ImGui::SameLine();
  if (ImGui::Button("close (e)")) {
    editor.close();
  }

  if (!editor.status.empty()) {
    ImGui::TextColored(ImVec4(0.95f, 0.85f, 0.35f, 1.0f), "%s", editor.status.c_str());
  } else {
    ImGui::TextDisabled(
        "n p pick a part   enter insert it above the cursor   del remove   [ ] restage");
  }

  ImGui::End();
}

void draw_help(App& app) {  if (!app.show_help) {
    return;
  }
  ImGui::SetNextWindowPos(ImVec2(60.0f, 60.0f), ImGuiCond_Appearing);
  ImGui::Begin("keys", &app.show_help, ImGuiWindowFlags_AlwaysAutoResize);

  ImGui::TextUnformatted("map");
  ImGui::Separator();
  ImGui::BulletText("wheel zoom about the cursor");
  ImGui::BulletText("drag left pan (stops following)");
  ImGui::BulletText("drag right orbit the view");
  ImGui::BulletText("click select the nearest object");
  ImGui::Separator();
  ImGui::TextUnformatted("keys");
  ImGui::Separator();
  for (const KeyBinding& binding : kBindings) {
    ImGui::Text("%-6s %s", binding.key_name, binding.action);
  }
  ImGui::Text("%-6s %s", "h", "this window");
  ImGui::Text("%-6s %s", "q", "quit");
  ImGui::Separator();
  ImGui::TextUnformatted("assembly editor");
  ImGui::Separator();
  ImGui::BulletText("n p pick a part, enter insert it above the cursor");
  ImGui::BulletText("del remove, [ ] restage, u stage from the decouplers");
  ImGui::BulletText("b build and fly (restarts the mission), s save");
  ImGui::End();
}

/// Writes the framebuffer to a binary PPM.
///
/// The graphical client's answer to the SVG dump the terminal client can already
/// write: a picture of the window that a person or a script can look at without
/// a window. PPM because it needs no encoder — the format is a three-line header
/// and the raw bytes, which is one dependency fewer than a PNG writer and enough
/// to see whether the map drew.
[[nodiscard]] bool write_ppm(const char* path, int width, int height) {
  if (width <= 0 || height <= 0) {
    return false;
  }
  std::vector<unsigned char> pixels(static_cast<std::size_t>(width) *
                                    static_cast<std::size_t>(height) * 3U);
  // OpenGL's origin is bottom-left and an image's is top-left, so flip on the
  // way out rather than storing an upside-down picture nobody can read.
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());

  std::FILE* file = std::fopen(path, "wb");
  if (file == nullptr) {
    return false;
  }
  std::fprintf(file, "P6\n%d %d\n255\n", width, height);
  const std::size_t stride = static_cast<std::size_t>(width) * 3U;
  for (int row = height - 1; row >= 0; --row) {
    std::fwrite(pixels.data() + static_cast<std::size_t>(row) * stride, 1, stride, file);
  }
  std::fclose(file);
  return true;
}

/// The window, the frame loop, and nothing else.
int run(App& app, int frame_limit, const std::string& screenshot) {
  if (glfwInit() == GLFW_FALSE) {
    std::fprintf(stderr,
                 "rocketlab: cannot open a window. Is DISPLAY set? The terminal client "
                 "(`rocketlab-tui`) needs no display.\n");
    return EXIT_FAILURE;
  }

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  GLFWwindow* window = glfwCreateWindow(1400, 820, "rocketlab", nullptr, nullptr);
  if (window == nullptr) {
    std::fprintf(stderr, "rocketlab: cannot create a window (OpenGL 3.3 unavailable?)\n");
    glfwTerminate();
    return EXIT_FAILURE;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui::GetIO().IniFilename = nullptr;  // no imgui.ini beside the binaries
  ImGui::StyleColorsDark();
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init("#version 330");

  auto last = std::chrono::steady_clock::now();
  int frames = 0;
  while (glfwWindowShouldClose(window) == GLFW_FALSE) {
    glfwPollEvents();

    const auto now = std::chrono::steady_clock::now();
    const double wall_dt =
        std::chrono::duration<double>(now - last).count();
    last = now;

    // The simulation advances here and nowhere else. `pump(0.0)` lets the host's
    // own frame timer supply the elapsed span, which is the single place the
    // wall clock is allowed to reach the simulation.
    app.host.pump(0.0);

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    handle_keys(app);
    draw_map(app, wall_dt);
    draw_telemetry(app);
    draw_assembly(app);
    draw_help(app);

    ImGui::Render();
    int width = 0;
    int height = 0;
    glfwGetFramebufferSize(window, &width, &height);
    glViewport(0, 0, width, height);
    glClearColor(0.047f, 0.055f, 0.078f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window);

    ++frames;
    if (frame_limit > 0 && frames >= frame_limit) {
      if (!screenshot.empty() && !write_ppm(screenshot.c_str(), width, height)) {
        std::fprintf(stderr, "rocketlab: cannot write '%s'\n", screenshot.c_str());
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
        glfwDestroyWindow(window);
        glfwTerminate();
        return EXIT_FAILURE;
      }
      break;
    }
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  glfwDestroyWindow(window);
  glfwTerminate();
  return EXIT_SUCCESS;
}

void print_usage() {
  std::puts(R"(rocketlab-gui - graphical client for the space mission simulator

usage:
  rocketlab-gui [scenario.json] [options]

options:
  --warp <factor>     initial time warp (default 1)
  --entity <name>     what the camera starts on (default the first vessel)
  --save <file>       where the assembly editor writes (default the scenario
                      file that was loaded, if one was)
  --frames <n>        draw n frames and exit, for a smoke test
  --screenshot <file> write the last frame as a PPM image and exit
  --script <e>=<f>    attach the Lua file <f> to the entity named <e> (repeatable)

The window shows the same map the terminal client draws, from the same camera —
and at zero pitch it *is* the same camera, the flat map being this one untilted.
Without a file the built-in default scenario is used.
)");
}

[[nodiscard]] bool split_assignment(std::string_view text, std::string& name, std::string& path) {
  const std::size_t at = text.find('=');
  if (at == std::string_view::npos || at == 0 || at + 1 >= text.size()) {
    return false;
  }
  name = std::string(text.substr(0, at));
  path = std::string(text.substr(at + 1));
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  std::vector<std::string> args(argv + 1, argv + argc);

  try {
    for (const std::string& arg : args) {
      if (arg == "-h" || arg == "--help") {
        print_usage();
        return EXIT_SUCCESS;
      }
    }

    std::string scenario_path;
    std::string entity_name;
    std::string save_path;
    std::vector<std::string> assignments;
    double warp = 1.0;
    int frame_limit = 0;
    std::string screenshot;

    const auto value_after = [&args](std::size_t& index, const char* flag) -> std::string {
      if (index + 1 >= args.size()) {
        throw std::runtime_error(std::string(flag) + " needs a value");
      }
      return args[++index];
    };

    for (std::size_t i = 0; i < args.size(); ++i) {
      const std::string& arg = args[i];
      if (arg == "--warp") {
        warp = std::stod(value_after(i, "--warp"));
      } else if (arg == "--entity") {
        entity_name = value_after(i, "--entity");
      } else if (arg == "--save") {
        save_path = value_after(i, "--save");
      } else if (arg == "--frames") {
        frame_limit = std::stoi(value_after(i, "--frames"));
      } else if (arg == "--screenshot") {
        screenshot = value_after(i, "--screenshot");
        // A screenshot of a window that is about to be opened and closed again
        // before the layout settles is not worth taking; a few frames let ImGui
        // place its windows and the map frame itself against a real size.
        if (frame_limit == 0) {
          frame_limit = 3;
        }
      } else if (arg == "--script") {
        assignments.push_back(value_after(i, "--script"));
      } else if (!arg.empty() && arg[0] == '-') {
        throw std::runtime_error("unknown option '" + arg + "'");
      } else if (scenario_path.empty()) {
        scenario_path = arg;
      } else {
        throw std::runtime_error("more than one scenario given");
      }
    }

    core::Scenario scenario = scenario_path.empty() ? scenario_io::default_scenario()
                                                    : scenario_io::load_scenario_file(scenario_path);
    if (!(warp > 0.0)) {
      warp = 1.0;
    }

    for (const std::string& assignment : assignments) {
      std::string name;
      std::string path;
      if (!split_assignment(assignment, name, path)) {
        throw std::runtime_error("--script wants <entity>=<path>, not '" + assignment + "'");
      }
      bool attached = false;
      for (core::ScenarioEntity& entity : scenario.entities) {
        if (entity.name == name) {
          entity.script = path;
          attached = true;
        }
      }
      if (!attached) {
        throw std::runtime_error("no entity named '" + name + "' to attach '" + path + "' to");
      }
    }

    App app(simhost::LocalSimSource::from_scenario(scenario));
    app.scenario_name = scenario.name;
    // The document the editor works on, and where a save goes. Without --save
    // the loaded file is the target, so editing the scenario you started from
    // is one keystroke; a default scenario has no file behind it and saving
    // then says so rather than inventing a name.
    app.base_scenario = scenario;
    app.save_path = save_path.empty() ? scenario_path : save_path;
    app.camera.metres_per_pixel = 1.0e6;
    // The warp is sent as a command rather than written into the scenario: how
    // fast time is passing is something a viewer decides, not something a
    // mission file records.
    app.host.send(proto::Command::set_warp(warp));

    // `--entity` is applied as a command rather than as a scenario edit: which
    // object the camera watches is client state, and the world has no opinion.
    if (!entity_name.empty()) {
      bool found = false;
      for (std::uint32_t i = 0; i < app.host.snapshot().entity_count; ++i) {
        if (app.host.snapshot().entities[i].name.view() == entity_name) {
          select_id(app, app.host.snapshot().entities[i].id);
          found = true;
          break;
        }
      }
      if (!found) {
        throw std::runtime_error("no entity named '" + entity_name + "'");
      }
    }

    return run(app, frame_limit, screenshot);
  } catch (const std::exception& error) {
    std::fprintf(stderr, "rocketlab: %s\n", error.what());
    return EXIT_FAILURE;
  }
}
