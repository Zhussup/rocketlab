// Tests for the layer clients actually touch: the snapshot, the simulation
// host behind it, and the scene built from it. None of this needs a terminal,
// which is the point of keeping the draw list separate from the drawing.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <string>
#include <vector>

#include "rocketlab/core/constants.hpp"
#include "rocketlab/render/camera.hpp"
#include "rocketlab/render/scene.hpp"
#include "rocketlab/render/svg.hpp"
#include "rocketlab/scenario/json_io.hpp"
#include "rocketlab/simhost/local_sim_source.hpp"

using namespace rocketlab;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

namespace proto = rocketlab::proto;
namespace render = rocketlab::render;
namespace simhost = rocketlab::simhost;
namespace scenario_io = rocketlab::scenario;

/// The scenario the client tests fly: one station in low Earth orbit, one
/// piece of debris, so that selection has something to move between.
core::Scenario two_entity_scenario() {
  core::Scenario scenario;
  scenario.name = "client test";
  scenario.epoch = core::Instant{0.0};

  core::ScenarioEntity station;
  station.name = "Station";
  station.parent_body = "Earth";
  station.periapsis_altitude = 400e3;
  station.apoapsis_altitude = 400e3;
  station.inclination_deg = 51.6;
  station.controllable = true;

  core::ScenarioEntity debris = station;
  debris.name = "Stage";
  debris.controllable = false;
  // A different orbit, so the two entities are distinguishable on the map
  // rather than drawn on top of each other.
  debris.apoapsis_altitude = 900e3;

  scenario.entities = {station, debris};
  return scenario;
}

const render::Primitive* find_text(const render::Scene& scene, std::string_view text) {
  for (const render::Primitive& primitive : scene.primitives) {
    if (primitive.kind == render::PrimitiveKind::Text && primitive.text == text) {
      return &primitive;
    }
  }
  return nullptr;
}

std::size_t count_kind(const render::Scene& scene, render::PrimitiveKind kind) {
  std::size_t total = 0;
  for (const render::Primitive& primitive : scene.primitives) {
    if (primitive.kind == kind) {
      ++total;
    }
  }
  return total;
}

}  // namespace

TEST_CASE("the camera projects and unprojects consistently", "[render]") {
  render::Camera2D camera;
  camera.width = 80;
  camera.height = 24;
  camera.cell_aspect = 2.0;
  camera.metres_per_pixel = 1000.0;

  SECTION("the centre of the view lands on the centre of the screen") {
    camera.center_x = 5.0e5;
    camera.center_y = -2.0e5;
    const render::ScreenPoint at = camera.project(proto::Vec3d{5.0e5, -2.0e5, 0.0});
    CHECK_THAT(at.x, WithinAbs(39.5, 1e-9));
    CHECK_THAT(at.y, WithinAbs(11.5, 1e-9));
  }

  SECTION("the view is isotropic on a character grid") {
    camera.center_x = 0.0;
    camera.center_y = 0.0;
    // A point one kilometre to the east and one to the north must be the same
    // physical distance from the centre on screen once the cell aspect is
    // taken into account. Ignoring the aspect is the classic way a terminal
    // map comes out stretched.
    const render::ScreenPoint east = camera.project(proto::Vec3d{1000.0, 0.0, 0.0});
    const render::ScreenPoint north = camera.project(proto::Vec3d{0.0, 1000.0, 0.0});
    const double dx_cells = east.x - 39.5;
    const double dy_cells = 11.5 - north.y;
    CHECK_THAT(dx_cells / camera.cell_aspect, WithinRel(dy_cells, 1e-12));
  }

  SECTION("unprojection inverts projection") {
    camera.center_x = 1.0e7;
    camera.center_y = -3.0e6;
    camera.yaw = 0.7;
    const proto::Vec3d root{1.0001e7, -3.02e6, 0.0};
    const render::ScreenPoint at = camera.project(root);
    const proto::Vec3d back = camera.unproject(at.x, at.y);
    CHECK_THAT(back.x, WithinAbs(root.x, 1e-3));
    CHECK_THAT(back.y, WithinAbs(root.y, 1e-3));
  }

  SECTION("yaw rotates the view without changing what is at the centre") {
    camera.center_x = 100.0;
    camera.center_y = 50.0;
    camera.yaw = 1.234;
    const render::ScreenPoint at = camera.project(proto::Vec3d{100.0, 50.0, 0.0});
    CHECK_THAT(at.x, WithinAbs(39.5, 1e-9));
    CHECK_THAT(at.y, WithinAbs(11.5, 1e-9));
  }
}

TEST_CASE("the camera zoom stays inside its limits", "[render]") {
  render::Camera2D camera;

  camera.metres_per_pixel = 1.0;
  for (int i = 0; i < 200; ++i) {
    camera.zoom_by(0.5);
  }
  CHECK_THAT(camera.metres_per_pixel, WithinAbs(render::kMinMetresPerPixel, 1e-12));

  for (int i = 0; i < 200; ++i) {
    camera.zoom_by(2.0);
  }
  CHECK_THAT(camera.metres_per_pixel, WithinAbs(render::kMaxMetresPerPixel, 1e-3));

  // A nonsense factor is ignored rather than poisoning the scale with a NaN.
  camera.zoom_by(0.0);
  camera.zoom_by(-1.0);
  CHECK_THAT(camera.metres_per_pixel, WithinAbs(render::kMaxMetresPerPixel, 1e-3));
}

TEST_CASE("framing a body puts it at a readable size", "[render]") {
  render::Camera2D camera;
  camera.width = 80;
  camera.height = 40;

  const double earth_radius = core::kRadiusEarth;
  camera.frame(proto::Vec3d{1.0e11, 0.0, 0.0}, earth_radius, 0.25);

  CHECK_THAT(camera.center_x, WithinAbs(1.0e11, 1e-3));
  // A quarter of the shorter axis (40 rows) is ten rows, so the radius should
  // come out at about that many pixels.
  CHECK_THAT(camera.to_pixels(earth_radius), WithinRel(10.0, 1e-9));
}

TEST_CASE("the camera glides to a new target rather than cutting to it", "[render]") {
  // The one piece of animation in either client, and the only part of the
  // graphical one that can be checked without a window: a selection change is
  // worth watching happen, because the distance between the two objects is the
  // thing the cut would hide.
  render::CameraEase ease;
  const proto::Vec3d start{0.0, 0.0, 0.0};
  const proto::Vec3d target{1.0e7, 0.0, 0.0};

  SECTION("the first step goes straight to the target") {
    // There is no previous frame to glide away from, so a fresh ease has to
    // place the camera rather than start it somewhere arbitrary.
    CHECK(ease.step(target, 1.0 / 60.0).x == target.x);
    CHECK_FALSE(ease.fresh);
  }

  SECTION("it approaches without overshooting") {
    ease.snap(start);
    double previous = 0.0;
    for (int i = 0; i < 120; ++i) {
      const proto::Vec3d at = ease.step(target, 1.0 / 60.0);
      CHECK(at.x > previous);
      CHECK(at.x <= target.x);  // an exponential approach never passes the mark
      previous = at.x;
    }
    // Two seconds at four per second is exp(-8), a third of a percent of the
    // gap — 3 km out of 10 000, which at any zoom this map offers is under a
    // pixel. The glide is over well before a person stops watching it.
    CHECK_THAT(ease.step(target, 1.0 / 60.0).x, WithinRel(target.x, 5e-3));
  }

  SECTION("the glide does not depend on the frame rate") {
    // A glide whose speed depended on how fast the client happened to be
    // drawing would be a different glide on every machine and unrepeatable in
    // a test, which is the whole reason the step is exponential.
    render::CameraEase fast;
    render::CameraEase slow;
    fast.snap(start);
    slow.snap(start);
    for (int i = 0; i < 60; ++i) {
      static_cast<void>(fast.step(target, 1.0 / 60.0));
    }
    static_cast<void>(slow.step(target, 1.0));
    CHECK_THAT(slow.center.x, WithinRel(fast.center.x, 1e-12));
  }

  SECTION("a snap cancels a glide in progress") {
    ease.snap(start);
    static_cast<void>(ease.step(target, 1.0 / 60.0));
    ease.snap(start);
    CHECK(ease.center.x == start.x);
    // And it stays put: `fresh` is still false, so the next step eases again
    // rather than jumping.
    const proto::Vec3d at = ease.step(target, 1.0 / 60.0);
    CHECK(at.x > start.x);
    CHECK(at.x < target.x);
  }

  SECTION("a zero rate is a cut") {
    render::CameraEase instant;
    instant.rate = 0.0;
    CHECK(instant.step(target, 1.0 / 60.0).x == target.x);
  }
}

TEST_CASE("grid spacing follows a 1-2-5 sequence", "[render]") {
  // Whatever the zoom, neighbouring lines must be at least the requested
  // number of pixels apart, and the spacing must be a round number.
  for (double metres_per_pixel = 1.0; metres_per_pixel < 1.0e9; metres_per_pixel *= 3.7) {
    const double spacing = render::choose_grid_spacing(metres_per_pixel, 8.0);
    INFO("metres per pixel " << metres_per_pixel);

    CHECK(spacing >= metres_per_pixel * 8.0);
    // Never more than the factor to the next rung of the sequence, which for
    // 1-2-5 is at most 2.5.
    CHECK(spacing < metres_per_pixel * 8.0 * 2.5);

    const double mantissa = spacing / std::pow(10.0, std::floor(std::log10(spacing)));
    CHECK((std::abs(mantissa - 1.0) < 1e-9 || std::abs(mantissa - 2.0) < 1e-9 ||
           std::abs(mantissa - 5.0) < 1e-9));
  }
}

TEST_CASE("a scene contains the bodies, the vessels and a labelled selection", "[render]") {
  auto source = simhost::LocalSimSource::from_scenario(two_entity_scenario());
  const proto::Snapshot& snapshot = source.snapshot();

  render::Camera2D camera;
  camera.width = 80;
  camera.height = 24;
  camera.cell_aspect = 2.0;
  // A thousand kilometres a row: the Earth is a disc a few rows across and the
  // station's orbit fits inside the frame. The camera follows the selection
  // rather than sitting at the root-frame origin, which is the Sun and would
  // show nothing but empty space.
  camera.metres_per_pixel = 1.0e6;
  camera.following = true;
  camera.target = snapshot.selected;
  render::follow_target(snapshot, camera);

  render::Scene scene;
  render::SceneOptions options;
  render::build_scene(snapshot, camera, options, {}, scene);

  CHECK(scene.width == 80);
  CHECK(scene.height == 24);
  CHECK(scene.scale_bar_metres > 0.0);

  // The Earth is large at this zoom and must be drawn; the station is far
  // smaller than a cell and must still be drawn, at the size floor.
  CHECK(find_text(scene, "Earth") != nullptr);
  CHECK(find_text(scene, "Station") != nullptr);
  CHECK(find_text(scene, "Stage") != nullptr);

  // Two vessels plus the planets in frame, all as discs.
  CHECK(count_kind(scene, render::PrimitiveKind::Disc) >= 2);
  // A reticle marks the selection, and only the selection.
  CHECK(count_kind(scene, render::PrimitiveKind::Cross) == 1);

  SECTION("every primitive stays inside the viewport") {
    for (const render::Primitive& primitive : scene.primitives) {
      for (const render::ScreenPoint& point : primitive.points) {
        CHECK(point.x >= 0.0);
        CHECK(point.x <= 79.0);
        CHECK(point.y >= 0.0);
        CHECK(point.y <= 23.0);
      }
    }
  }
}

TEST_CASE("a body smaller than a cell is still visible", "[render]") {
  auto source = simhost::LocalSimSource::from_scenario(two_entity_scenario());

  render::Camera2D camera;
  camera.width = 80;
  camera.height = 24;
  camera.cell_aspect = 2.0;
  camera.metres_per_pixel = 1.0e10;  // an entire planetary system per screen

  render::Scene scene;
  render::SceneOptions options;
  render::build_scene(source.snapshot(), camera, options, {}, scene);

  for (const render::Primitive& primitive : scene.primitives) {
    if (primitive.kind != render::PrimitiveKind::Disc || primitive.points.empty()) {
      continue;
    }
    // No marker may be drawn smaller than the floor, or a planet would vanish
    // exactly when it is serving as a landmark.
    CHECK(primitive.radius_y >= options.min_body_radius - 1e-9);
  }
}

/// A scenario of `count` vessels all sitting in the same low Earth orbit, so
/// that at a coarse zoom they project into the same cell and their labels have
/// to fight over the same space. Names are short and start alike so that the
/// overlap is guaranteed rather than incidental.
core::Scenario crowded_scenario(int count) {
  core::Scenario scenario;
  scenario.name = "crowded";
  scenario.epoch = core::Instant{0.0};

  for (int i = 0; i < count; ++i) {
    core::ScenarioEntity entity;
    entity.name = "Vessel" + std::to_string(i);
    entity.parent_body = "Earth";
    entity.periapsis_altitude = 400e3;
    entity.apoapsis_altitude = 400e3;
    entity.inclination_deg = 51.6;
    entity.controllable = true;
    scenario.entities.push_back(entity);
  }
  return scenario;
}

/// Where a label was finally put, in cells.
struct PlacedText {
  int row{-1};
  int x0{0};
  int x1{0};
  std::string text;
};

std::vector<PlacedText> placed_texts(const render::Scene& scene) {
  std::vector<PlacedText> out;
  for (const render::Primitive& primitive : scene.primitives) {
    if (primitive.kind != render::PrimitiveKind::Text || primitive.points.empty()) {
      continue;
    }
    const render::ScreenPoint at = primitive.points[0];
    const int x0 = static_cast<int>(std::floor(at.x));
    out.push_back(PlacedText{static_cast<int>(std::floor(at.y + 0.5)), x0,
                             x0 + static_cast<int>(primitive.text.size()), primitive.text});
  }
  return out;
}

TEST_CASE("labels that want the same space are separated rather than interleaved", "[render]") {
  // Two markers in the same cell would otherwise print their names into one
  // unreadable word, which is what a decluttered map must never do.
  auto source = simhost::LocalSimSource::from_scenario(crowded_scenario(2));
  const proto::Snapshot& snapshot = source.snapshot();

  render::Camera2D camera;
  camera.width = 80;
  camera.height = 24;
  camera.cell_aspect = 2.0;
  camera.metres_per_pixel = 1.0e6;
  camera.following = true;
  camera.target = snapshot.selected;
  render::follow_target(snapshot, camera);

  render::Scene scene;
  render::SceneOptions options;
  render::build_scene(snapshot, camera, options, {}, scene);

  // Both names survive: there is room nearby, so neither should be dropped.
  const render::Primitive* first = find_text(scene, "Vessel0");
  const render::Primitive* second = find_text(scene, "Vessel1");
  REQUIRE(first != nullptr);
  REQUIRE(second != nullptr);

  // And they do not share a row, because they overlap horizontally.
  CHECK(first->points[0].y != second->points[0].y);

  SECTION("no two labels ever overlap") {
    const std::vector<PlacedText> texts = placed_texts(scene);
    for (std::size_t i = 0; i < texts.size(); ++i) {
      for (std::size_t j = i + 1; j < texts.size(); ++j) {
        const bool same_row = texts[i].row == texts[j].row;
        const bool overlap = texts[i].x0 <= texts[j].x1 && texts[j].x0 <= texts[i].x1;
        INFO(texts[i].text << " on row " << texts[i].row << " vs " << texts[j].text << " on row "
                           << texts[j].row);
        CHECK_FALSE((same_row && overlap));
      }
    }
  }
}

TEST_CASE("the tracked object keeps its name when labels compete", "[render]") {
  auto source = simhost::LocalSimSource::from_scenario(crowded_scenario(2));
  const proto::Snapshot& snapshot = source.snapshot();

  render::Camera2D camera;
  camera.width = 80;
  camera.height = 24;
  camera.cell_aspect = 2.0;
  camera.metres_per_pixel = 1.0e6;
  camera.following = true;
  camera.target = snapshot.selected;
  render::follow_target(snapshot, camera);

  render::Scene scene;
  render::SceneOptions options;
  render::build_scene(snapshot, camera, options, {}, scene);

  // The selection is placed first, so it gets the row its marker points at
  // rather than being pushed aside by scenery that was queued after it.
  const proto::EntitySnapshot* selected = nullptr;
  for (std::uint32_t i = 0; i < snapshot.entity_count; ++i) {
    if (snapshot.entities[i].id == snapshot.selected) {
      selected = &snapshot.entities[i];
    }
  }
  REQUIRE(selected != nullptr);

  const render::Primitive* label = find_text(scene, std::string(selected->name.view()));
  REQUIRE(label != nullptr);

  const double marker_y = camera.project(render::root_position(*selected)).y;
  CHECK_THAT(label->points[0].y, WithinAbs(std::floor(marker_y + 0.5), 1e-9));
}

TEST_CASE("a crowded marker loses its name only when every nearby row is taken", "[render]") {
  // Five candidate rows around a marker, so seven co-located vessels cannot all
  // be named. The point of the test is that the loss is by exhaustion and not
  // by the first collision.
  auto source = simhost::LocalSimSource::from_scenario(crowded_scenario(7));
  const proto::Snapshot& snapshot = source.snapshot();

  render::Camera2D camera;
  camera.width = 80;
  camera.height = 24;
  camera.cell_aspect = 2.0;
  camera.metres_per_pixel = 1.0e6;
  camera.following = true;
  camera.target = snapshot.selected;
  render::follow_target(snapshot, camera);

  render::Scene scene;
  render::SceneOptions options;
  render::build_scene(snapshot, camera, options, {}, scene);

  int named = 0;
  for (int i = 0; i < 7; ++i) {
    if (find_text(scene, "Vessel" + std::to_string(i)) != nullptr) {
      ++named;
    }
  }
  CHECK(named == 5);

  // Whatever survived must still be readable: a dropped name is acceptable, an
  // interleaved one is not.
  const std::vector<PlacedText> texts = placed_texts(scene);
  for (std::size_t i = 0; i < texts.size(); ++i) {
    for (std::size_t j = i + 1; j < texts.size(); ++j) {
      const bool same_row = texts[i].row == texts[j].row;
      const bool overlap = texts[i].x0 <= texts[j].x1 && texts[j].x0 <= texts[i].x1;
      INFO(texts[i].text << " vs " << texts[j].text);
      CHECK_FALSE((same_row && overlap));
    }
  }
}

TEST_CASE("a predicted path is drawn from the samples it was given", "[render]") {
  auto source = simhost::LocalSimSource::from_scenario(two_entity_scenario());
  const proto::Snapshot& snapshot = source.snapshot();
  const std::uint64_t station = snapshot.selected;
  REQUIRE(station != 0);

  std::vector<proto::Vec3d> path;
  REQUIRE(source.query_trajectory(station, 6000.0, path));
  REQUIRE(path.size() > 2);

  render::Camera2D camera;
  camera.width = 80;
  camera.height = 24;
  camera.cell_aspect = 2.0;
  // Zoomed so the orbit is larger than the frame on both axes: the path has to
  // be clipped, and it is the clipped case that is worth checking.
  camera.metres_per_pixel = 5.0e5;
  camera.following = true;
  camera.target = station;
  render::follow_target(snapshot, camera);

  render::Scene scene;
  render::SceneOptions options;
  render::build_scene(snapshot, camera, options, path, scene);

  // Clipping splits a path into segments, so the count is not one; what must
  // hold is that the trajectory colour appears at all and that the total
  // painted length is non-trivial.
  std::size_t segments = 0;
  for (const render::Primitive& primitive : scene.primitives) {
    if (primitive.kind == render::PrimitiveKind::Polyline &&
        primitive.color == render::colors::kTrajectory) {
      ++segments;
    }
  }
  CHECK(segments > 0);

  SECTION("turning trajectories off removes them") {
    render::Scene bare;
    options.show_trajectory = false;
    render::build_scene(snapshot, camera, options, path, bare);
    for (const render::Primitive& primitive : bare.primitives) {
      CHECK(primitive.color != render::colors::kTrajectory);
    }
  }
}

TEST_CASE("the scene serialises to SVG", "[render]") {
  auto source = simhost::LocalSimSource::from_scenario(two_entity_scenario());

  render::Camera2D camera;
  camera.width = 40;
  camera.height = 12;
  camera.cell_aspect = 2.0;
  camera.metres_per_pixel = 5.0e5;

  render::Scene scene;
  render::build_scene(source.snapshot(), camera, {}, {}, scene);
  const std::string svg = render::to_svg(scene);

  CHECK(svg.starts_with("<svg"));
  CHECK(svg.ends_with("</svg>"));
  CHECK(svg.find("</text>") != std::string::npos);
  // The aspect is applied by the viewBox, so the declared width is the cell
  // count scaled by the aspect and not the raw column count.
  CHECK(svg.find("viewBox=\"0 0 40 12\"") != std::string::npos);
  CHECK(svg.find("width=\"640\"") != std::string::npos);
}

TEST_CASE("the local sim source publishes a frame a client can use", "[simhost]") {
  auto source = simhost::LocalSimSource::from_scenario(two_entity_scenario());
  const proto::Snapshot& snapshot = source.snapshot();

  REQUIRE(snapshot.entity_count == 2);
  CHECK(snapshot.body_count >= 3);
  CHECK(snapshot.selected != 0);
  CHECK(source.describe() == "local");

  const proto::EntitySnapshot& station = snapshot.entities[0];
  CHECK(std::string_view(station.name.view()) == "Station");
  CHECK(proto::has_flag(station.flags, proto::Flags::Controllable));
  CHECK_FALSE(proto::has_flag(station.flags, proto::Flags::Escaping));
  CHECK_THAT(station.periapsis, WithinRel(core::kRadiusEarth + 400e3, 1e-3));

  SECTION("the parent position matches the catalogue, so a client can compose the root state") {
    const core::BodySystem system = core::BodySystem::solar_system();
    const core::StateVector earth = system.root_state(*system.find("Earth"), snapshot.tdb);
    CHECK_THAT(station.parent_position.x, WithinAbs(earth.r.x, 1.0));
    CHECK_THAT(station.parent_position.y, WithinAbs(earth.r.y, 1.0));
    CHECK_THAT(station.parent_position.z, WithinAbs(earth.r.z, 1.0));
  }
}

TEST_CASE("commands change the warp and the selection", "[simhost]") {
  auto source = simhost::LocalSimSource::from_scenario(two_entity_scenario());

  SECTION("warp zero pauses the clock and a positive warp moves it") {
    source.send(proto::Command::set_warp(0.0));
    source.pump(1.0);
    const double paused = source.snapshot().tdb;

    source.pump(1.0);
    CHECK_THAT(source.snapshot().tdb, WithinAbs(paused, 1e-12));

    source.send(proto::Command::set_warp(100.0));
    source.pump(1.0);
    CHECK_THAT(source.snapshot().tdb, WithinAbs(paused + 100.0, 1e-6));
  }

  SECTION("stepping the warp walks the ladder one rung at a time") {
    source.send(proto::Command::set_warp(1.0));
    source.pump(0.0);
    source.send(proto::Command::step_warp(1));
    source.pump(0.0);
    CHECK_THAT(source.snapshot().warp, WithinRel(5.0, 1e-12));
    source.send(proto::Command::step_warp(-1));
    source.pump(0.0);
    CHECK_THAT(source.snapshot().warp, WithinRel(1.0, 1e-12));
  }

  SECTION("cycling the selection stays among controllable entities and wraps") {
    const std::uint64_t first = source.snapshot().selected;
    REQUIRE(first != 0);
    // Only the station is controllable, so cycling must come back to it rather
    // than landing on the debris.
    source.send(proto::Command::cycle_target(1));
    source.pump(0.0);
    CHECK(source.snapshot().selected == first);
  }

  SECTION("selecting an unknown entity clears the selection instead of failing") {
    source.send(proto::Command::select(9999));
    source.pump(0.0);
    CHECK(source.snapshot().selected == 0);
  }

  SECTION("removing the selected entity moves the selection on") {
    const std::uint64_t first = source.snapshot().selected;
    source.send(proto::Command{proto::CommandKind::Remove, {}, 0.0, first});
    source.pump(0.0);
    CHECK(source.snapshot().entity_count == 1);
    CHECK(source.snapshot().selected != first);
  }
}

TEST_CASE("a trajectory query predicts where the simulation will actually be", "[simhost]") {
  core::Scenario scenario = two_entity_scenario();
  // Start somewhere other than the epoch so the prediction has to cope with
  // bodies that are themselves moving.
  scenario.epoch = core::Instant{1234.0 * core::kSecondsPerDay};
  auto source = simhost::LocalSimSource::from_scenario(scenario);

  const proto::Snapshot& snapshot = source.snapshot();
  const std::uint64_t station_id = snapshot.selected;


  const double period = 5545.0;  // about one orbit at 400 km
  std::vector<proto::Vec3d> path;
  REQUIRE(source.query_trajectory(station_id, period, path));
  REQUIRE(path.size() == simhost::LocalSimSource::kTrajectorySamples + 1);

  const core::BodySystem system = core::BodySystem::solar_system();
  const core::BodyId earth = *system.find("Earth");

  SECTION("every sample sits at the orbital radius, in the parent's frame") {
    // Because the path comes back relative to the Earth, the check is the one
    // the orbit actually makes: constant distance, no ephemeris needed. If the
    // query ever regressed to root-frame points, the Earth's own 166,000 km of
    // travel during one orbit would blow this apart immediately.
    //
    // The station is in air, so the radius is not quite constant: over this one
    // orbit drag takes about seven metres off it. The tolerance is that decay,
    // and the two-sided check below pins the decay down — a frame mistake
    // misses by thousands of kilometres, and drag that had stopped working
    // would leave the radius exactly flat.
    const double expected = core::kRadiusEarth + 400e3;
    double lowest = expected;
    for (std::size_t i = 0; i < path.size(); ++i) {
      const double distance =
          std::sqrt(path[i].x * path[i].x + path[i].y * path[i].y + path[i].z * path[i].z);
      INFO("sample " << i);
      CHECK_THAT(distance, WithinAbs(expected, 100.0));
      if (distance < lowest) {
        lowest = distance;
      }
    }
    INFO("the station fell " << expected - lowest << " m over the orbit");
    CHECK(expected - lowest > 1.0);
    CHECK(expected - lowest < 100.0);
  }

  SECTION("the path closes after one period") {
    const double first_x = path.front().x;
    const double first_y = path.front().y;
    const double first_z = path.front().z;
    const double last_x = path.back().x;
    const double last_y = path.back().y;
    const double last_z = path.back().z;
    const double closure = std::sqrt(std::pow(last_x - first_x, 2.0) +
                                     std::pow(last_y - first_y, 2.0) +
                                     std::pow(last_z - first_z, 2.0));
    // Analytic propagation over exactly one period must land back on the start
    // to within the sampling of the last step, not merely somewhere nearby.
    CHECK_THAT(closure, WithinAbs(0.0, 1.0e4));
  }

  SECTION("the first sample is the current position, not a step ahead of it") {
    const proto::EntitySnapshot& station = snapshot.entities[0];
    CHECK_THAT(path.front().x, WithinAbs(station.position.x, 1e-6));
    CHECK_THAT(path.front().y, WithinAbs(station.position.y, 1e-6));
    CHECK_THAT(path.front().z, WithinAbs(station.position.z, 1e-6));
  }

  SECTION("the predicted endpoint is exactly where the simulation ends up") {
    // The whole point of routing prediction through the host is that there is
    // one answer, not two. So: fly the same world for the same span and require
    // the queried endpoint to match the state it actually arrives at. A second,
    // silently drifting implementation of the propagation is what this catches.
    core::World world = core::World::from_scenario(scenario);
    world.advance(period, 60.0);
    const core::Entity* after = world.find(station_id);
    REQUIRE(after != nullptr);

    // Both sides are in the parent frame, which is the whole reason they can be
    // compared without touching the ephemeris at all.
    CHECK_THAT(path.back().x, WithinAbs(after->state.r.x, 1.0e3));
    CHECK_THAT(path.back().y, WithinAbs(after->state.r.y, 1.0e3));
    CHECK_THAT(path.back().z, WithinAbs(after->state.r.z, 1.0e3));
  }

  SECTION("the frame convention is the parent's, not the root's") {
    // Composing with the parent position from the snapshot gives the path as it
    // sits around the Earth *now* — which is what a map should draw. Composing
    // with the parent position at the sample's own time gives the true future
    // root position. Both are checked, because confusing the two is the
    // easiest mistake to make here and it is invisible in a picture.
    const proto::EntitySnapshot& station = snapshot.entities[0];

    const core::Vec3 earth_now = system.root_state(earth, snapshot.tdb).r;
    CHECK_THAT(station.parent_position.x + path.front().x,
               WithinAbs(earth_now.x + station.position.x, 1.0e3));

    const core::Vec3 earth_then = system.root_state(earth, snapshot.tdb + period).r;
    core::World world = core::World::from_scenario(scenario);
    world.advance(period, 60.0);
    const core::StateVector root_after = world.root_state(*world.find(station_id));
    CHECK_THAT(earth_then.x + path.back().x, WithinAbs(root_after.r.x, 1.0e4));
    CHECK_THAT(earth_then.y + path.back().y, WithinAbs(root_after.r.y, 1.0e4));
    CHECK_THAT(earth_then.z + path.back().z, WithinAbs(root_after.r.z, 1.0e4));
  }

  SECTION("a query for an unknown entity fails rather than inventing a path") {
    std::vector<proto::Vec3d> other;
    CHECK_FALSE(source.query_trajectory(9999, period, other));
    CHECK(other.empty());
  }

  SECTION("a zero horizon still succeeds and yields the current position alone") {
    std::vector<proto::Vec3d> single;
    CHECK(source.query_trajectory(station_id, 0.0, single));
    CHECK(single.empty());
  }
}
