// Tests for the 3D camera.
//
// Two claims carry the design and are what these check. First, that the flat
// map is not a second projection beside this one but this one at pitch zero —
// so the two cannot disagree about where anything is. Second, that the basis
// never collapses, at any yaw or pitch, because it is built from an axis that
// always lies in the reference plane. The rest is ordering: a tilted view has
// to paint far to near, and a flat one must not sort at all.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <limits>
#include <string_view>
#include <vector>

#include "rocketlab/proto/snapshot.hpp"
#include "rocketlab/render/camera.hpp"
#include "rocketlab/render/camera3d.hpp"
#include "rocketlab/render/scene.hpp"

using namespace rocketlab;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

namespace proto = rocketlab::proto;
namespace render = rocketlab::render;

[[nodiscard]] double length(const proto::Vec3d& v) noexcept {
  return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

[[nodiscard]] double dot(const proto::Vec3d& a, const proto::Vec3d& b) noexcept {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

[[nodiscard]] proto::Vec3d cross(const proto::Vec3d& a, const proto::Vec3d& b) noexcept {
  return proto::Vec3d{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

[[nodiscard]] proto::Vec3d add(const proto::Vec3d& a, const proto::Vec3d& b) noexcept {
  return proto::Vec3d{a.x + b.x, a.y + b.y, a.z + b.z};
}

[[nodiscard]] proto::Vec3d scaled(const proto::Vec3d& v, double k) noexcept {
  return proto::Vec3d{v.x * k, v.y * k, v.z * k};
}

/// A camera of a size worth testing, with the aspect a terminal cell has.
[[nodiscard]] render::Camera3D a_camera() {
  render::Camera3D camera;
  camera.width = 80;
  camera.height = 24;
  camera.cell_aspect = 2.0;
  camera.metres_per_pixel = 1000.0;
  camera.center = proto::Vec3d{5.0e5, -2.0e5, 0.0};
  return camera;
}

/// An entity whose root position is `root`, in the root frame directly.
[[nodiscard]] proto::EntitySnapshot entity_at(std::uint64_t id, std::string_view name,
                                              const proto::Vec3d& root, double radius) {
  proto::EntitySnapshot entity;
  entity.id = id;
  entity.position = proto::Vec3d{0.0, 0.0, 0.0};
  entity.parent_position = root;
  entity.radius = radius;
  entity.name.assign(name);
  return entity;
}

[[nodiscard]] proto::BodySnapshot body_at(std::uint32_t id, std::string_view name,
                                          const proto::Vec3d& root, double radius) {
  proto::BodySnapshot body;
  body.id = id;
  body.position = root;
  body.radius = radius;
  body.name.assign(name);
  return body;
}

/// Draw options with everything off that would otherwise put primitives between
/// the markers and make an index-based assertion brittle.
[[nodiscard]] render::SceneOptions markers_only() {
  render::SceneOptions options;
  options.show_grid = false;
  options.show_trajectory = false;
  options.show_labels = false;
  options.show_scale_bar = false;
  return options;
}

}  // namespace

TEST_CASE("the 3D camera at zero pitch is the flat map", "[render][camera3d]") {
  const double yaws[] = {0.0, 0.7, -1.9, 3.0};

  for (const double yaw : yaws) {
    render::Camera3D camera = a_camera();
    camera.yaw = yaw;

    render::Camera2D flat;
    flat.width = camera.width;
    flat.height = camera.height;
    flat.cell_aspect = camera.cell_aspect;
    flat.metres_per_pixel = camera.metres_per_pixel;
    flat.center_x = camera.center.x;
    flat.center_y = camera.center.y;
    flat.yaw = yaw;

    // Points spread over the plane and off it: at zero pitch the height is
    // dropped, so an orbit above the ecliptic projects onto the same place as
    // its own shadow, which is exactly what the flat map has always drawn.
    const proto::Vec3d points[] = {{5.0e5, -2.0e5, 0.0},
                                   {5.3e5, -1.7e5, 0.0},
                                   {4.2e5, -2.4e5, 0.0},
                                   {5.1e5, -1.9e5, 8.0e6}};

    for (const proto::Vec3d& point : points) {
      const render::ScreenPoint here = camera.project(point);
      const render::ScreenPoint there = flat.project(point);
      CHECK_THAT(here.x, WithinAbs(there.x, 1e-9));
      CHECK_THAT(here.y, WithinAbs(there.y, 1e-9));
    }

    const proto::Vec3d back = camera.unproject(11.0, 7.0);
    const proto::Vec3d flat_back = flat.unproject(11.0, 7.0);
    CHECK_THAT(back.x, WithinAbs(flat_back.x, 1e-6));
    CHECK_THAT(back.y, WithinAbs(flat_back.y, 1e-6));
    CHECK_THAT(back.z, WithinAbs(0.0, 1e-9));
  }
}

TEST_CASE("the 3D basis is orthonormal at every orientation", "[render][camera3d]") {
  constexpr double kTwoPi = 6.283185307179586476925286766559;

  for (int yi = 0; yi < 8; ++yi) {
    for (int pi = -6; pi <= 6; ++pi) {
      render::Camera3D camera = a_camera();
      camera.yaw = kTwoPi * static_cast<double>(yi) / 8.0;
      camera.pitch = render::kMaxPitch * static_cast<double>(pi) / 6.0;

      const proto::Vec3d right = camera.right_axis();
      const proto::Vec3d up = camera.up_axis();
      const proto::Vec3d forward = camera.forward_axis();

      CHECK_THAT(length(right), WithinAbs(1.0, 1e-12));
      CHECK_THAT(length(up), WithinAbs(1.0, 1e-12));
      CHECK_THAT(length(forward), WithinAbs(1.0, 1e-12));

      CHECK_THAT(dot(right, up), WithinAbs(0.0, 1e-12));
      CHECK_THAT(dot(right, forward), WithinAbs(0.0, 1e-12));
      CHECK_THAT(dot(up, forward), WithinAbs(0.0, 1e-12));

      // Handedness: screen right cross screen up points back at the viewer,
      // which is the way `forward` does not. A basis that flipped sign here
      // would render the world mirrored, and it would do it only past the
      // pitch where the cross product changed sign — the kind of bug a test
      // that only looked at the zero-pitch view would never see.
      const proto::Vec3d handed = cross(right, up);
      CHECK_THAT(handed.x, WithinAbs(-forward.x, 1e-12));
      CHECK_THAT(handed.y, WithinAbs(-forward.y, 1e-12));
      CHECK_THAT(handed.z, WithinAbs(-forward.z, 1e-12));

      // The centre of the view is the centre of the screen, at every
      // orientation.
      const render::ScreenPoint middle = camera.project(camera.center);
      CHECK_THAT(middle.x, WithinAbs(0.5 * (camera.width - 1), 1e-9));
      CHECK_THAT(middle.y, WithinAbs(0.5 * (camera.height - 1), 1e-9));
    }
  }
}

TEST_CASE("no orientation collapses the view", "[render][camera3d]") {
  // The pole-ward limit is where the naive construction of a look-at basis
  // divides by zero. This one is built from an axis that lies in the reference
  // plane, so the limit is a perfectly ordinary view — and it is worth pinning
  // that, because the fix is easy to undo by "simplifying" the basis later.
  render::Camera3D camera = a_camera();
  camera.yaw = 0.9;
  camera.pitch = render::kMaxPitch;

  const render::ScreenPoint east = camera.project(add(camera.center, scaled(camera.right_axis(), 500.0)));
  const render::ScreenPoint north = camera.project(add(camera.center, scaled(camera.up_axis(), 500.0)));

  CHECK(std::isfinite(east.x));
  CHECK(std::isfinite(east.y));
  CHECK(std::isfinite(north.x));
  CHECK(std::isfinite(north.y));

  // Two axes that are perpendicular in space must land in different places,
  // half a pixel apart in the direction each one names.
  CHECK_THAT(east.x - north.x, WithinAbs(0.5 * camera.cell_aspect, 1e-9));
  CHECK_THAT(east.y - north.y, WithinAbs(0.5, 1e-9));

  // And the other pole is the same view mirrored, not a second singularity.
  camera.pitch = -render::kMaxPitch;
  const render::ScreenPoint down = camera.project(camera.center);
  CHECK(std::isfinite(down.x));
  CHECK(std::isfinite(down.y));
}

TEST_CASE("orbit turns the view and stops at the pole", "[render][camera3d]") {
  constexpr double kTwoPi = 6.283185307179586476925286766559;
  render::Camera3D camera = a_camera();

  SECTION("a pitch beyond the limit is clamped, not wrapped") {
    camera.orbit(0.0, 10.0);
    CHECK_THAT(camera.pitch, WithinAbs(render::kMaxPitch, 1e-12));
    camera.orbit(0.0, -10.0);
    CHECK_THAT(camera.pitch, WithinAbs(-render::kMaxPitch, 1e-12));

    // Clamped, not held at the limit forever: a drag back the other way moves
    // it immediately, which a wrap would not.
    camera.orbit(0.0, 0.25);
    CHECK_THAT(camera.pitch, WithinAbs(-render::kMaxPitch + 0.25, 1e-12));
  }

  SECTION("yaw wraps into [0, 2pi) whichever way the drag goes") {
    camera.orbit(7.0, 0.0);
    CHECK_THAT(camera.yaw, WithinAbs(7.0 - kTwoPi, 1e-12));

    camera.yaw = 0.0;
    camera.orbit(-1.0, 0.0);
    CHECK_THAT(camera.yaw, WithinAbs(kTwoPi - 1.0, 1e-12));

    for (int i = 0; i < 50; ++i) {
      camera.orbit(-3.3, 0.0);
      CHECK(camera.yaw >= 0.0);
      CHECK(camera.yaw < kTwoPi);
    }
  }

  SECTION("a non-finite drag is ignored rather than poisoning the camera") {
    const double yaw = camera.yaw;
    const double pitch = camera.pitch;
    camera.orbit(std::nan(""), 0.1);
    camera.orbit(0.1, std::nan(""));
    camera.orbit(0.1, std::numeric_limits<double>::infinity());
    CHECK_THAT(camera.yaw, WithinAbs(yaw, 1e-15));
    CHECK_THAT(camera.pitch, WithinAbs(pitch, 1e-15));
  }
}

TEST_CASE("depth grows away from the viewer", "[render][camera3d]") {
  render::Camera3D camera = a_camera();
  camera.yaw = 0.3;
  camera.pitch = 0.5;

  const render::View view = camera.view();
  const proto::Vec3d forward = camera.forward_axis();

  CHECK_THAT(view.depth(camera.center), WithinAbs(0.0, 1e-9));
  CHECK_THAT(view.depth(add(camera.center, scaled(forward, 1000.0))), WithinAbs(1000.0, 1e-9));
  CHECK_THAT(view.depth(add(camera.center, scaled(forward, -1000.0))), WithinAbs(-1000.0, 1e-9));

  // Away from the viewer must be away from the viewer, whatever the angles: a
  // point further along the view axis is never scored nearer.
  const proto::Vec3d near_point = add(camera.center, scaled(forward, -500.0));
  const proto::Vec3d far_point = add(camera.center, scaled(forward, 500.0));
  CHECK(view.depth(far_point) > view.depth(near_point));

  // A top-down camera looks down the pole, so "away" is the direction the
  // pole points, which is the sense the flat map has always used.
  render::Camera3D top_down = a_camera();
  const render::View down = top_down.view();
  CHECK_THAT(down.depth(add(top_down.center, proto::Vec3d{0.0, 0.0, -1000.0})),
             WithinAbs(1000.0, 1e-9));
  CHECK_THAT(down.depth(add(top_down.center, proto::Vec3d{0.0, 0.0, 1000.0})),
             WithinAbs(-1000.0, 1e-9));
}

TEST_CASE("unprojection returns a point in the view plane", "[render][camera3d]") {
  render::Camera3D camera = a_camera();
  camera.yaw = 1.1;
  camera.pitch = -0.6;

  const render::View view = camera.view();
  const proto::Vec3d point = view.unproject(13.5, 4.25);

  CHECK_THAT(view.depth(point), WithinAbs(0.0, 1e-6));

  const render::ScreenPoint back = camera.project(point);
  CHECK_THAT(back.x, WithinAbs(13.5, 1e-6));
  CHECK_THAT(back.y, WithinAbs(4.25, 1e-6));

  // Round-tripping a point that is not in the plane drops only its height
  // above the plane, and keeps everything else.
  const proto::Vec3d off_plane =
      add(camera.center, add(scaled(camera.right_axis(), 250.0), scaled(camera.up_axis(), -175.0)));
  const proto::Vec3d projected = view.unproject(camera.project(off_plane).x,
                                                camera.project(off_plane).y);
  CHECK_THAT(dot(projected, camera.right_axis()),
             WithinAbs(dot(off_plane, camera.right_axis()), 1e-6));
  CHECK_THAT(dot(projected, camera.up_axis()), WithinAbs(dot(off_plane, camera.up_axis()), 1e-6));
}

TEST_CASE("a flat 3D view draws the same picture as the 2D camera", "[render][camera3d]") {
  proto::Snapshot snapshot;
  snapshot.body_count = 2;
  snapshot.bodies[0] = body_at(1, "Earth", proto::Vec3d{0.0, 0.0, 0.0}, 6.371e6);
  snapshot.bodies[1] = body_at(2, "Moon", proto::Vec3d{3.844e8, 0.0, 1.0e7}, 1.737e6);
  snapshot.entity_count = 2;
  snapshot.entities[0] = entity_at(10, "Station", proto::Vec3d{7.0e6, 1.0e6, 0.0}, 20.0);
  snapshot.entities[1] = entity_at(11, "Stage", proto::Vec3d{7.2e6, 1.4e6, 4.0e5}, 3.0);
  snapshot.selected = 10;

  render::Camera2D flat;
  flat.width = 80;
  flat.height = 24;
  flat.cell_aspect = 2.0;
  flat.metres_per_pixel = 2.0e5;
  flat.center_x = 1.0e6;
  flat.center_y = 5.0e5;
  flat.yaw = 0.4;

  render::Camera3D camera;
  camera.width = flat.width;
  camera.height = flat.height;
  camera.cell_aspect = flat.cell_aspect;
  camera.metres_per_pixel = flat.metres_per_pixel;
  camera.center = proto::Vec3d{flat.center_x, flat.center_y, 0.0};
  camera.yaw = flat.yaw;

  const render::SceneOptions options;
  render::Scene from_flat;
  render::Scene from_3d;
  render::build_scene(snapshot, flat, options, {}, from_flat);
  render::build_scene(snapshot, camera.view(), options, {}, from_3d);

  REQUIRE(from_flat.primitives.size() == from_3d.primitives.size());
  for (std::size_t i = 0; i < from_flat.primitives.size(); ++i) {
    const render::Primitive& a = from_flat.primitives[i];
    const render::Primitive& b = from_3d.primitives[i];
    CHECK(a.kind == b.kind);
    CHECK(a.text == b.text);
    REQUIRE(a.points.size() == b.points.size());
    for (std::size_t j = 0; j < a.points.size(); ++j) {
      CHECK_THAT(a.points[j].x, WithinAbs(b.points[j].x, 1e-9));
      CHECK_THAT(a.points[j].y, WithinAbs(b.points[j].y, 1e-9));
    }
  }
}

TEST_CASE("a tilted view paints far to near", "[render][camera3d]") {
  render::Camera3D camera;
  camera.width = 80;
  camera.height = 24;
  camera.cell_aspect = 1.0;
  camera.metres_per_pixel = 1.0;
  camera.center = proto::Vec3d{0.0, 0.0, 0.0};
  camera.yaw = 0.0;
  camera.pitch = 0.5;
  camera.following = false;

  // Four markers spread along the view axis at depths +3, +1, -1, -3, each
  // nudged sideways so they land in different columns and the order they were
  // painted in can be read off the result. Two are bodies and two are
  // entities, which is what proves the two lists are merged rather than sorted
  // one after the other.
  const proto::Vec3d forward = camera.forward_axis();
  const proto::Vec3d right = camera.right_axis();
  const auto placed = [&](double depth, double across) {
    return add(add(camera.center, scaled(forward, depth)), scaled(right, across));
  };

  proto::Snapshot snapshot;
  snapshot.body_count = 2;
  snapshot.bodies[0] = body_at(1, "Far", placed(3.0, 0.0), 0.5);
  snapshot.bodies[1] = body_at(2, "Near", placed(-1.0, 20.0), 0.5);
  snapshot.entity_count = 2;
  snapshot.entities[0] = entity_at(10, "Middle", placed(1.0, 10.0), 0.5);
  snapshot.entities[1] = entity_at(11, "Nearest", placed(-3.0, 30.0), 0.5);
  snapshot.selected = 10;

  const render::SceneOptions options = markers_only();

  // Screen column of a marker placed `across` metres along the right axis:
  // half a viewport in, plus one column per metre, since the aspect is 1.0 and
  // the scale is 1 m/unit. Written out so the two orders below are exact
  // numbers rather than a comparison that would pass either way round.
  const auto column_of = [](double across) { return 39.5 + across; };

  SECTION("the tilted view sorts the merged list far to near") {
    render::Scene scene;
    render::build_scene(snapshot, camera.view(), options, {}, scene);

    std::vector<double> columns;
    for (const render::Primitive& primitive : scene.primitives) {
      if (primitive.kind == render::PrimitiveKind::Disc) {
        columns.push_back(primitive.points[0].x);
      }
    }
    REQUIRE(columns.size() == 4);
    // Far, middle-near, nearer, nearest: the order the depths ask for, with
    // the two bodies and the two entities interleaved rather than grouped.
    CHECK_THAT(columns[0], WithinAbs(column_of(0.0), 1e-9));
    CHECK_THAT(columns[1], WithinAbs(column_of(10.0), 1e-9));
    CHECK_THAT(columns[2], WithinAbs(column_of(20.0), 1e-9));
    CHECK_THAT(columns[3], WithinAbs(column_of(30.0), 1e-9));
  }

  SECTION("the flat view keeps the order the snapshot was written in") {
    camera.pitch = 0.0;
    render::Scene scene;
    render::build_scene(snapshot, camera.view(), options, {}, scene);

    std::vector<double> columns;
    for (const render::Primitive& primitive : scene.primitives) {
      if (primitive.kind == render::PrimitiveKind::Disc) {
        columns.push_back(primitive.points[0].x);
      }
    }
    REQUIRE(columns.size() == 4);
    // Bodies first in index order, then entities: the picture the map has
    // always drawn, and the third column sitting left of the second is the
    // signature of it — a sorted view would have put entity 0 before body 1.
    CHECK_THAT(columns[0], WithinAbs(column_of(0.0), 1e-9));
    CHECK_THAT(columns[1], WithinAbs(column_of(20.0), 1e-9));
    CHECK_THAT(columns[2], WithinAbs(column_of(10.0), 1e-9));
    CHECK_THAT(columns[3], WithinAbs(column_of(30.0), 1e-9));
  }
}
