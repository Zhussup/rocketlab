// Tests for the readouts and the character backend.
//
// Both of these are pure functions of a snapshot or a scene, which is the
// reason they live outside the terminal client: a formatter that can only be
// checked by looking at a screen is a formatter that silently drifts.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <limits>
#include <string>

#include "rocketlab/core/scenario.hpp"
#include "rocketlab/hud/readout.hpp"
#include "rocketlab/scenario/json_io.hpp"
#include "rocketlab/render/scene.hpp"
#include "rocketlab/render/text.hpp"
#include "rocketlab/simhost/local_sim_source.hpp"

using namespace rocketlab;
using Catch::Matchers::WithinAbs;

namespace {

namespace core = rocketlab::core;
namespace hud = rocketlab::hud;
namespace proto = rocketlab::proto;
namespace render = rocketlab::render;
namespace scenario_io = rocketlab::scenario;
namespace simhost = rocketlab::simhost;

const hud::Row* find_row(const std::vector<hud::Row>& rows, std::string_view label) {
  for (const hud::Row& row : rows) {
    if (row.label == label) {
      return &row;
    }
  }
  return nullptr;
}

}  // namespace

TEST_CASE("lengths and speeds are formatted at a readable scale", "[hud]") {
  // The unit changes at each step so the number stays in a comfortable range;
  // a telemetry panel is read at a glance, and scientific notation is not.
  // The precision follows the magnitude, so the number always carries about
  // three significant figures and never more than the data deserves.
  CHECK(hud::format_length(1234.0) == "1.23 km");
  CHECK(hud::format_length(400e3) == "400.0 km");
  CHECK(hud::format_length(384.4e6) == "384.4 Mm");
  CHECK(hud::format_length(1.496e11) == "149.6 Gm");
  CHECK(hud::format_length(45.0) == "45.00 m");
  CHECK(hud::format_length(-2500.0) == "-2.50 km");

  CHECK(hud::format_speed(7.66e3) == "7.66 km/s");
  CHECK(hud::format_speed(120.0) == "120.0 m/s");

  CHECK(hud::format_mass(420000.0) == "420.0 t");
  CHECK(hud::format_angle(1.5707963267948966) == "90.00°");
}

TEST_CASE("periods are readable and unbound orbits say so", "[hud]") {
  CHECK(hud::format_period(5545.0) == "1h 32m 25s");
  CHECK(hud::format_period(3.0 * 86400.0 + 7200.0) == "3d 02h 00m");
  CHECK(hud::format_period(45.0) == "45s");

  // A parabola has no period and a hyperbola has no meaningful one, so neither
  // may be printed as a number.
  CHECK(hud::format_period(std::numeric_limits<double>::infinity()) == "open");
  CHECK(hud::format_period(0.0) == "open");
  CHECK(hud::format_period(-1.0) == "open");
}

TEST_CASE("a readout describes the orbit a scenario asked for", "[hud]") {
  core::Scenario scenario;
  scenario.name = "hud";
  core::ScenarioEntity station;
  station.name = "Station";
  station.parent_body = "Earth";
  station.periapsis_altitude = 400e3;
  station.apoapsis_altitude = 400e3;
  station.inclination_deg = 51.6;
  station.mass = 420000.0;
  station.controllable = true;
  scenario.entities = {station};

  auto source = simhost::LocalSimSource::from_scenario(scenario);
  const proto::Snapshot& snapshot = source.snapshot();
  const proto::EntitySnapshot* entity = hud::find_entity(snapshot, snapshot.selected);
  REQUIRE(entity != nullptr);

  const std::vector<hud::Row> rows = hud::entity_readout(snapshot, *entity);

  REQUIRE(find_row(rows, "parent") != nullptr);
  CHECK(find_row(rows, "parent")->value == "Earth");
  CHECK(find_row(rows, "kind")->value == "vessel");
  CHECK(find_row(rows, "mass")->value == "420.0 t");

  // Altitude above the surface, not radius from the centre: this is the number
  // a mission planner reads, and getting it wrong by a planetary radius is the
  // classic version of this bug.
  REQUIRE(find_row(rows, "periapsis") != nullptr);
  CHECK(find_row(rows, "periapsis")->value == "400.0 km");
  CHECK(find_row(rows, "apoapsis")->value == "400.0 km");
  CHECK(find_row(rows, "inclination")->value == "51.60°");

  // A circular orbit has an eccentricity of exactly zero, so the check is
  // absolute rather than relative.
  CHECK_THAT(std::stod(find_row(rows, "eccentricity")->value), WithinAbs(0.0, 1e-6));

  // The instantaneous altitude must agree with the element set on a circular
  // orbit, which is a cross-check that both come from the same state vector.
  CHECK(find_row(rows, "altitude")->value == "400.0 km");
  CHECK_FALSE(find_row(rows, "altitude")->emphasised);
}

TEST_CASE("an escaping orbit is flagged rather than printed as a period", "[hud]") {
  core::Scenario scenario;
  scenario.name = "escape";
  core::ScenarioEntity probe;
  probe.name = "Probe";
  probe.parent_body = "Earth";
  probe.periapsis_altitude = 400e3;
  probe.apoapsis_altitude = 900e6;  // far past escape; the builder makes it a hyperbola
  probe.controllable = true;
  scenario.entities = {probe};

  auto source = simhost::LocalSimSource::from_scenario(scenario);
  const proto::Snapshot& snapshot = source.snapshot();
  const proto::EntitySnapshot* entity = hud::find_entity(snapshot, snapshot.selected);
  REQUIRE(entity != nullptr);

  const std::vector<hud::Row> rows = hud::entity_readout(snapshot, *entity);
  if (proto::has_flag(entity->flags, proto::Flags::Escaping)) {
    CHECK(find_row(rows, "period")->value == "open");
    CHECK(find_row(rows, "period")->emphasised);
    CHECK(find_row(rows, "apoapsis")->value == "escape");
  }
}

TEST_CASE("the entity list carries the selection and the controllable flag", "[hud]") {
  core::Scenario scenario;
  core::ScenarioEntity vessel;
  vessel.name = "Vessel";
  vessel.parent_body = "Earth";
  vessel.periapsis_altitude = 400e3;
  vessel.apoapsis_altitude = 400e3;
  vessel.controllable = true;

  core::ScenarioEntity debris = vessel;
  debris.name = "Stage";
  debris.controllable = false;
  scenario.entities = {vessel, debris};

  auto source = simhost::LocalSimSource::from_scenario(scenario);
  const std::vector<hud::EntityLine> lines = hud::entity_list(source.snapshot());
  REQUIRE(lines.size() == 2);

  CHECK(lines[0].name == "Vessel");
  CHECK(lines[0].controllable);
  CHECK(lines[0].parent == "Earth");
  CHECK(lines[1].name == "Stage");
  CHECK_FALSE(lines[1].controllable);

  // Exactly one line is the selection, and it is the controllable one the host
  // chose when it was built.
  CHECK(lines[0].selected);
  CHECK_FALSE(lines[1].selected);
}

TEST_CASE("the readout reports what the flight computer is doing", "[hud][flight]") {
  // The tug and the shipped transfer script, so the panel is exercised on the
  // mission a person would actually fly rather than on a hand-built snapshot.
  // A row that has never been rendered against a real host is a row that has
  // never been checked.
  core::Scenario scenario = scenario_io::load_scenario_file(
      std::string(ROCKETLAB_SOURCE_DIR) + "/scenarios/tug.json");
  bool attached = false;
  for (core::ScenarioEntity& entity : scenario.entities) {
    if (entity.name == "Tug") {
      entity.script = std::string(ROCKETLAB_SOURCE_DIR) + "/scripts/hohmann.lua";
      attached = true;
    }
  }
  REQUIRE(attached);

  simhost::LocalSimSource source = simhost::LocalSimSource::from_scenario(scenario);
  const proto::EntitySnapshot* tug =
      hud::find_entity(source.snapshot(), source.snapshot().selected);
  REQUIRE(tug != nullptr);

  SECTION("before the first tick the computer has done nothing") {
    const std::vector<hud::Row> rows = hud::entity_readout(source.snapshot(), *tug);
    REQUIRE(find_row(rows, "computer") != nullptr);
    CHECK(find_row(rows, "computer")->value == "idle");
    CHECK_FALSE(find_row(rows, "computer")->emphasised);
    // Nothing has been logged yet, so a "script" row would be an empty line
    // pretending to be information.
    CHECK(find_row(rows, "script") == nullptr);
  }

  SECTION("a running computer shows its state, its budget and its last words") {
    source.pump(1.0);
    const std::vector<hud::Row> rows = hud::entity_readout(source.snapshot(), *tug);
    REQUIRE(find_row(rows, "computer") != nullptr);
    CHECK(find_row(rows, "computer")->value == "running");
    REQUIRE(find_row(rows, "instructions") != nullptr);
    CHECK(find_row(rows, "instructions")->value != "0");
    REQUIRE(find_row(rows, "script") != nullptr);
    CHECK(find_row(rows, "script")->value == "raising apoapsis to 1200 km");
  }

  SECTION("a fault is emphasised and carries the script's own words") {
    proto::Snapshot snapshot{};
    snapshot.entity_count = 1;
    snapshot.entities[0].name.assign("Probe");
    snapshot.entities[0].computer = proto::ComputerState::Faulted;
    snapshot.entities[0].computer_message.assign(
        "hohmann.lua:41: attempt to index a nil value (global 'ship')");

    const std::vector<hud::Row> rows = hud::entity_readout(snapshot, snapshot.entities[0]);
    REQUIRE(find_row(rows, "computer") != nullptr);
    CHECK(find_row(rows, "computer")->value == "faulted");
    CHECK(find_row(rows, "computer")->emphasised);
    REQUIRE(find_row(rows, "script") != nullptr);
    CHECK(find_row(rows, "script")->value.find("nil value") != std::string::npos);
    CHECK(find_row(rows, "script")->emphasised);
  }

  SECTION("an entity with no computer gets no computer rows") {
    proto::Snapshot snapshot{};
    snapshot.entity_count = 1;
    snapshot.entities[0].name.assign("Bystander");
    const std::vector<hud::Row> rows = hud::entity_readout(snapshot, snapshot.entities[0]);
    CHECK(find_row(rows, "computer") == nullptr);
    CHECK(find_row(rows, "instructions") == nullptr);
  }
}

TEST_CASE("a scene rasterises to characters", "[render]") {
  core::Scenario scenario;
  core::ScenarioEntity station;
  station.name = "Station";
  station.parent_body = "Earth";
  station.periapsis_altitude = 400e3;
  station.apoapsis_altitude = 400e3;
  station.controllable = true;
  scenario.entities = {station};

  auto source = simhost::LocalSimSource::from_scenario(scenario);
  const proto::Snapshot& snapshot = source.snapshot();

  render::Camera2D camera;
  camera.width = 60;
  camera.height = 20;
  camera.cell_aspect = 2.0;
  camera.metres_per_pixel = 5.0e5;
  camera.target = snapshot.selected;
  camera.following = true;
  render::follow_target(snapshot, camera);

  std::vector<proto::Vec3d> path;
  REQUIRE(source.query_trajectory(snapshot.selected, 6000.0, path));

  render::Scene scene;
  render::build_scene(snapshot, camera, {}, path, scene);
  const std::string text = render::to_text(scene);

  // One line per row, and every line exactly the width asked for.
  std::size_t lines = 0;
  for (std::size_t start = 0; start < text.size();) {
    const std::size_t end = text.find('\n', start);
    REQUIRE(end != std::string::npos);
    CHECK(end - start == 60);
    ++lines;
    start = end + 1;
  }
  CHECK(lines == 20);

  // The Earth is a body, the station a vessel, and the orbit a path. Each must
  // survive the conversion, since a backend that quietly drops a layer is
  // exactly the failure this backend exists to catch.
  CHECK(text.find('#') != std::string::npos);
  CHECK(text.find('O') != std::string::npos);
  CHECK(text.find('*') != std::string::npos);

  SECTION("turning a layer off removes its glyph") {
    render::SceneOptions options;
    options.show_trajectory = false;
    options.show_labels = false;
    render::Scene bare;
    render::build_scene(snapshot, camera, options, path, bare);
    CHECK(render::to_text(bare).find('*') == std::string::npos);
  }

  SECTION("an empty scene is blank but still the right size") {
    render::Scene empty;
    empty.width = 10;
    empty.height = 3;
    const std::string blank = render::to_text(empty);
    CHECK(blank == "          \n          \n          \n");
  }
}
