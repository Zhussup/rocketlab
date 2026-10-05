// Tests for the Lua flight computer, and for the host that flies it.
//
// The three properties the milestone is named after — sandboxed, deterministic,
// budgeted — are each a claim that would be easy to make and hard to notice
// being false, so each one is checked here rather than asserted in a comment.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "rocketlab/core/body.hpp"
#include "rocketlab/core/orbital.hpp"
#include "rocketlab/core/scenario.hpp"
#include "rocketlab/flight/lua_program.hpp"
#include "rocketlab/scenario/json_io.hpp"
#include "rocketlab/simhost/local_sim_source.hpp"

using namespace rocketlab;
using Catch::Matchers::WithinAbs;

namespace {

namespace flight = rocketlab::flight;
namespace proto = rocketlab::proto;
namespace simhost = rocketlab::simhost;

/// Where the source tree is, so a test can name the files a person would name
/// instead of whatever directory ctest happens to run in.
[[nodiscard]] std::string source_path(std::string_view relative) {
  return std::string(ROCKETLAB_SOURCE_DIR) + "/" + std::string(relative);
}

[[nodiscard]] const proto::EntitySnapshot* find_entity(const proto::Snapshot& snapshot,
                                                       std::string_view name) {
  for (std::uint32_t i = 0; i < snapshot.entity_count; ++i) {
    if (snapshot.entities[i].name.view() == name) {
      return &snapshot.entities[i];
    }
  }
  return nullptr;
}

/// The radius of the body an entity is orbiting, taken off the wire rather than
/// spelled out, because the snapshot publishes it and a test with 6371 km in it
/// is a test with a copy of the body table in it.
[[nodiscard]] double body_radius(const proto::Snapshot& snapshot, std::uint32_t parent) {
  for (std::uint32_t i = 0; i < snapshot.body_count; ++i) {
    if (snapshot.bodies[i].id == parent) {
      return snapshot.bodies[i].radius;
    }
  }
  return 0.0;
}

/// Compiles a chunk, failing the test with the compiler's own message if it
/// will not load. Almost every test below is about what a script does when it
/// runs, so a load failure here is a bug in the test rather than a result.
[[nodiscard]] std::unique_ptr<flight::LuaProgram> compile(
    const std::string& source, flight::LuaProgram::Limits limits = {}) {
  std::string error;
  std::unique_ptr<flight::LuaProgram> program =
      flight::LuaProgram::compile(source, "test chunk", limits, error);
  if (program == nullptr) {
    FAIL("the script did not compile: " << error);
  }
  return program;
}

/// A view of a vessel in a 400 km circular equatorial orbit, which is the
/// background every flight test runs against.
[[nodiscard]] flight::Input orbit_input(double met = 0.0) {
  const core::BodySystem system = core::BodySystem::solar_system();
  const core::CelestialBody& earth = system.body(*system.find("Earth"));

  flight::Input in;
  in.met = met;
  in.mu = earth.mu;
  in.body_radius = earth.radius;
  in.state = core::state_from_altitudes(earth, 400e3, 400e3, 0.0, 0.0, 0.0, 0.0);
  in.elements = core::rv_to_elements(in.state, earth.mu);
  in.altitude = core::norm(in.state.r) - earth.radius;
  in.time_to_apoapsis = core::time_to_apoapsis(in.elements, earth.mu);
  in.time_to_periapsis = core::time_to_periapsis(in.elements, earth.mu);
  return in;
}

/// A 5.09 t single-stage vessel in a 400 km circular orbit: probe, one FL-100
/// tank and a Terrier. The numbers are the catalogue's, not the test's.
[[nodiscard]] core::Scenario scripted_scenario() {
  core::Scenario scenario;
  scenario.name = "scripted";
  scenario.epoch = core::Instant{0.0};

  core::ScenarioEntity tug;
  tug.name = "Tug";
  tug.parent_body = "Earth";
  tug.periapsis_altitude = 400e3;
  tug.apoapsis_altitude = 400e3;
  tug.controllable = true;
  tug.parts = {{"probe.core", 0}, {"tank.fl100", 0}, {"engine.terrier", 0}};

  core::ScenarioEntity bystander = tug;
  bystander.name = "Bystander";
  bystander.true_anomaly_deg = 180.0;
  bystander.parts.clear();

  scenario.entities = {tug, bystander};
  return scenario;
}

/// A script on disk, so the tests go through the same file-loading path a user
/// does rather than calling the compiler directly.
class TempScript {
 public:
  TempScript(const std::string& name, std::string source)
      : path_("/tmp/rocketlab-" + name + ".lua") {
    std::ofstream out(path_);
    out << source;
  }
  ~TempScript() { std::remove(path_.c_str()); }
  TempScript(const TempScript&) = delete;
  TempScript& operator=(const TempScript&) = delete;

  [[nodiscard]] const std::string& path() const { return path_; }

 private:
  std::string path_;
};

}  // namespace

TEST_CASE("a script reads the telemetry and asks for a throttle", "[flight]") {
  const auto program = compile(R"(
    function update()
      ship.set_throttle(0.25)
    end
  )");

  const flight::Input in = orbit_input(120.0);
  CHECK(program->status() == flight::Status::Idle);

  flight::Output out;
  REQUIRE(program->update(in, out));
  CHECK(program->status() == flight::Status::Running);
  CHECK_THAT(out.throttle, WithinAbs(0.25, 1e-12));
  CHECK_FALSE(out.stage);
}

TEST_CASE("the telemetry a script sees is the state it was handed", "[flight]") {
  // Every other test here would pass on a `ship` table that was never filled
  // in, so this one reads the numbers back and checks them against the conic
  // they came from.
  const auto program = compile(R"(
    function update()
      if math.abs(ship.radius - (ship.body_radius + 400000.0)) > 1.0 then
        ship.abort("radius is not the distance from the centre")
        return
      end
      if math.abs(ship.radius - ship.body_radius - ship.altitude) > 1.0 then
        ship.abort("altitude and radius disagree")
        return
      end
      if not ship.elements_valid then
        ship.abort("a circular orbit was called degenerate")
        return
      end
      -- Circular at 6771 km: sqrt(mu/r) = 7672.6 m/s.
      if math.abs(ship.speed - 7672.6) > 1.0 then
        ship.abort(string.format("speed is %.1f", ship.speed))
        return
      end
      if math.abs(ship.apoapsis - ship.periapsis) > 1.0 then
        ship.abort("a circular orbit has two different apsides")
        return
      end
      ship.log("telemetry agrees")
    end
  )");

  flight::Output out;
  const bool alive = program->update(orbit_input(), out);
  INFO(program->message());
  REQUIRE(alive);
  CHECK(program->message() == "telemetry agrees");
}

TEST_CASE("the sandbox has no filesystem, no clock and no randomness", "[flight]") {
  const auto program = compile(R"(
    local forbidden = {
      "io", "os", "package", "debug", "coroutine",
      "dofile", "loadfile", "load", "require",
      "print", "collectgarbage", "warn",
    }
    function update()
      for _, name in ipairs(forbidden) do
        if _G[name] ~= nil then
          ship.abort("the global " .. name .. " is reachable")
          return
        end
      end
      if math.random ~= nil or math.randomseed ~= nil then
        ship.abort("the interpreter can roll dice")
        return
      end
      -- What is left has to still work, or the sandbox would be an empty box.
      if type(string.format) ~= "function" or math.sqrt(4) ~= 2 then
        ship.abort("the sandbox took too much")
        return
      end
      ship.log("clean")
    end
  )");

  const flight::Input in = orbit_input();
  flight::Output out;
  for (int i = 0; i < 3; ++i) {
    REQUIRE(program->update(in, out));
  }
  INFO(program->message());
  CHECK(program->message() == "clean");
}

TEST_CASE("a runaway script is stopped by the instruction budget", "[flight]") {
  flight::LuaProgram::Limits limits;
  limits.instructions_per_tick = 50000;

  const auto program = compile(R"(
    function update()
      local n = 0
      while true do n = n + 1 end
    end
  )",
                               limits);

  flight::Output out;
  CHECK_FALSE(program->update(orbit_input(), out));
  CHECK(program->status() == flight::Status::Faulted);

  // The fault has to say what happened, because "the autopilot stopped" is the
  // single least useful thing a readout can print.
  INFO(program->message());
  CHECK(program->message().find("budget") != std::string::npos);
  CHECK(program->message().find("50000") != std::string::npos);

  // And having faulted it stays faulted: a program allowed another try would be
  // a way to spend the budget over and over.
  CHECK_FALSE(program->update(orbit_input(), out));
}

TEST_CASE("a script that raises is faulted, not fatal", "[flight]") {
  const auto program = compile(R"(
    function update()
      error("the guidance law is upside down")
    end
  )");

  flight::Output out;
  CHECK_FALSE(program->update(orbit_input(), out));
  CHECK(program->status() == flight::Status::Faulted);
  INFO(program->message());
  CHECK(program->message().find("upside down") != std::string::npos);
  // With a traceback, because which line of a control law went wrong is the
  // first thing its author wants to know.
  CHECK(program->message().find("test chunk") != std::string::npos);
}

TEST_CASE("a script with no update function will not load", "[flight]") {
  std::string error;
  const auto program = flight::LuaProgram::compile("local x = 1\n", "empty", {}, error);
  CHECK(program == nullptr);
  INFO(error);
  CHECK(error.find("update") != std::string::npos);
}

TEST_CASE("a syntax error is reported at load, not at flight time", "[flight]") {
  std::string error;
  const auto program = flight::LuaProgram::compile("function update( end\n", "broken", {}, error);
  CHECK(program == nullptr);
  CHECK_FALSE(error.empty());
}

TEST_CASE("a chunk that spins at load time pays the same budget", "[flight]") {
  // The top level of a Lua file is code like any other, and a file that spins
  // there would hang the loader rather than the flight.
  flight::LuaProgram::Limits limits;
  limits.instructions_per_tick = 20000;

  std::string error;
  const auto program =
      flight::LuaProgram::compile("local n = 0\nwhile true do n = n + 1 end\nfunction update() end\n",
                                  "spinner", limits, error);
  CHECK(program == nullptr);
  INFO(error);
  CHECK(error.find("budget") != std::string::npos);
}

TEST_CASE("the memory cap stops a script that allocates without end", "[flight]") {
  flight::LuaProgram::Limits limits;
  limits.memory_bytes = 256U << 10U;
  // A hundred thousand instructions would not fill a megabyte; one `string.rep`
  // of a gigabyte does, and costs the script a handful of instructions to ask
  // for. Without the cap this would ask the host for the gigabyte instead.
  const auto program = compile(R"(
    function update()
      local big = string.rep("x", 1000000000)
      ship.log(#big)
    end
  )",
                               limits);

  flight::Output out;
  CHECK_FALSE(program->update(orbit_input(), out));
  CHECK(program->status() == flight::Status::Faulted);
  INFO(program->message());
  CHECK(program->message().find("memory") != std::string::npos);
}

TEST_CASE("the budget is per tick, so a long mission is not a long run", "[flight]") {
  flight::LuaProgram::Limits limits;
  limits.instructions_per_tick = 20000;

  // A few thousand instructions every tick for a hundred ticks. The total is
  // well past the per-tick budget, which is the point: the budget bounds the
  // cost of a tick, not the length of a mission.
  const auto program = compile(R"(
    local total = 0
    function update()
      for i = 1, 300 do total = total + i end
      ship.set_throttle(0)
    end
  )",
                               limits);

  const flight::Input in = orbit_input();
  flight::Output out;
  for (int i = 0; i < 100; ++i) {
    REQUIRE(program->update(in, out));
  }
  CHECK(program->instructions() > static_cast<std::uint64_t>(limits.instructions_per_tick));
}

TEST_CASE("an attitude request resolves against the state the tick began with", "[flight]") {
  const flight::Input in = orbit_input();

  SECTION("prograde is passed on as the prograde convention") {
    const auto program = compile(R"(
      function update() ship.point_prograde() end
    )");
    flight::Output out;
    REQUIRE(program->update(in, out));
    // Exactly zero, because zero is what the integrator reads as "follow the
    // velocity" rather than "hold this direction".
    CHECK(core::norm_squared(out.thrust_dir) == 0.0);
    CHECK(out.set_attitude);
  }

  SECTION("retrograde is the velocity reversed") {
    const auto program = compile(R"(
      function update() ship.point_retrograde() end
    )");
    flight::Output out;
    REQUIRE(program->update(in, out));
    CHECK(core::norm(out.thrust_dir - (-core::normalized(in.state.v))) < 1e-12);
  }

  SECTION("radial is away from the body and normal is the orbit's") {
    const auto radial = compile(R"(
      function update() ship.point_radial() end
    )");
    const auto normal = compile(R"(
      function update() ship.point_normal() end
    )");

    flight::Output out;
    REQUIRE(radial->update(in, out));
    CHECK(core::norm(out.thrust_dir - core::normalized(in.state.r)) < 1e-12);
    // Radial thrust is perpendicular to the velocity on a circular orbit.
    CHECK_THAT(core::dot(out.thrust_dir, in.state.v), WithinAbs(0.0, 1e-6));

    REQUIRE(normal->update(in, out));
    const core::Vec3 expected = core::normalized(core::cross(in.state.r, in.state.v));
    CHECK(core::norm(out.thrust_dir - expected) < 1e-12);
    // An equatorial orbit has its normal along +z.
    CHECK_THAT(out.thrust_dir.z, WithinAbs(1.0, 1e-9));
  }

  SECTION("an explicit direction is taken as given") {
    const auto program = compile(R"(
      function update() ship.point(0, 1, 0) end
    )");
    flight::Output out;
    REQUIRE(program->update(in, out));
    CHECK_THAT(out.thrust_dir.x, WithinAbs(0.0, 1e-12));
    CHECK_THAT(out.thrust_dir.y, WithinAbs(1.0, 1e-12));
    CHECK_THAT(out.thrust_dir.z, WithinAbs(0.0, 1e-12));
  }

  SECTION("a script that only sets the throttle says nothing about attitude") {
    // Not the same as asking for prograde. A control law that lined the vessel
    // up on an earlier tick and now only wants to throttle must not find itself
    // swung back to prograde underneath, and zero `thrust_dir` cannot say that
    // because zero already means prograde.
    const auto program = compile(R"(
      function update() ship.set_throttle(1) end
    )");
    flight::Output out;
    REQUIRE(program->update(in, out));
    CHECK_FALSE(out.set_attitude);
    CHECK_THAT(out.throttle, WithinAbs(1.0, 1e-12));
  }

  SECTION("a script that does steer is taken at its word") {
    const auto program = compile(R"(
      function update() ship.point_antinormal() end
    )");
    flight::Output out;
    REQUIRE(program->update(in, out));
    CHECK(out.set_attitude);
    CHECK_THAT(out.thrust_dir.z, WithinAbs(-1.0, 1e-9));
  }
}

TEST_CASE("a program runs the same way twice from the same state", "[flight]") {
  // Determinism, checked the only way it can be: run it twice and compare
  // everything that is observable.
  const char* source = R"(
    local tick = 0
    function update()
      tick = tick + 1
      ship.point_prograde()
      ship.set_throttle((tick % 7) / 7)
      if tick % 13 == 0 then ship.jettison() end
      ship.log(tick, ship.apoapsis, math.sqrt(ship.speed))
    end
  )";

  const flight::Input in = orbit_input();

  struct Run {
    std::vector<double> throttles;
    std::uint64_t instructions{0};
    std::string message;
  };
  const auto once = [&]() {
    Run run;
    {
      const auto program = compile(source);
      flight::Output out;
      for (int i = 0; i < 200; ++i) {
        program->update(in, out);
        run.throttles.push_back(out.throttle);
      }
      run.instructions = program->instructions();
      run.message = std::string(program->message());
    }
    return run;
  };

  const Run first = once();
  const Run second = once();

  CHECK(first.throttles == second.throttles);
  CHECK(first.instructions == second.instructions);
  CHECK(first.message == second.message);
}

TEST_CASE("a clone starts where the program started", "[flight]") {
  const auto program = compile(R"(
    local tick = 0
    function update()
      tick = tick + 1
      ship.set_throttle(tick > 5 and 1 or 0)
      ship.point_radial()
    end
  )");

  const flight::Input in = orbit_input();
  flight::Output out;
  for (int i = 0; i < 7; ++i) {
    program->update(in, out);
  }
  CHECK_THAT(out.throttle, WithinAbs(1.0, 1e-12));

  // The clone rewinds to the beginning, not to where the original has got to.
  // That is exactly what predicting a trajectory needs: the prediction starts
  // from the live state and flies the plan forward from there.
  const std::unique_ptr<flight::Program> copy = program->clone();
  REQUIRE(copy != nullptr);
  CHECK(copy->status() == flight::Status::Idle);

  flight::Output from_clone;
  for (int i = 0; i < 7; ++i) {
    REQUIRE(copy->update(in, from_clone));
  }
  CHECK_THAT(from_clone.throttle, WithinAbs(1.0, 1e-12));
  CHECK(copy->instructions() == program->instructions());
}

// --- the host ------------------------------------------------------------------

TEST_CASE("a scenario can name a script and the host flies it", "[flight][simhost]") {
  const TempScript script("raise", R"(
    function update()
      ship.point_prograde()
      ship.set_throttle(1)
      if ship.apoapsis > ship.body_radius + 500000.0 then
        ship.set_throttle(0)
        ship.abort("apoapsis raised")
      end
    end
  )");

  core::Scenario scenario = scripted_scenario();
  scenario.entities[0].script = script.path();

  simhost::LocalSimSource source = simhost::LocalSimSource::from_scenario(scenario);
  REQUIRE(source.computers().size() == 1);
  CHECK(source.computers().front().path == script.path());

  const std::uint64_t tug = source.snapshot().selected;
  REQUIRE(tug != 0);
  const double apoapsis_before = find_entity(source.snapshot(), "Tug")->apoapsis;

  for (int i = 0; i < 120; ++i) {
    source.pump(1.0);
  }

  const proto::EntitySnapshot* published = find_entity(source.snapshot(), "Tug");
  REQUIRE(published != nullptr);
  REQUIRE(published->computer == proto::ComputerState::Finished);
  INFO(std::string(published->computer_message.view()));
  CHECK(published->computer_message.view() == "apoapsis raised");

  // The script aims at 500 km above the body's surface, and `ship.apoapsis` is a
  // radius from the centre, so the target is a radius too. Both ends are pinned:
  // the lower one says the burn happened, and the upper one says it *stopped*.
  // Without that upper bound this test passed for a year with the engine still
  // lit — `ship.abort()` cut the throttle, the host threw the final tick's output
  // away, and the vessel burned on until the assertion below was met by luck.
  const double target = body_radius(source.snapshot(), published->parent) + 500e3;
  INFO("apoapsis " << published->apoapsis << " m against a target of " << target << " m");
  CHECK(published->apoapsis > target);
  CHECK(published->apoapsis < target + 100e3);
  CHECK(published->propellant < published->propellant_capacity);
  // The bystander, on the same orbit with nothing flying it, has not moved.
  CHECK(find_entity(source.snapshot(), "Bystander")->apoapsis < apoapsis_before + 1.0);
}

TEST_CASE("the host keeps flying a scripted vessel between control ticks", "[flight][simhost]") {
  // A script that asks for full throttle and never lets go. If the throttle it
  // set were dropped between control ticks the vessel would burn for one second
  // in sixty, which is the kind of bug that looks like a weak engine.
  const TempScript script("hold", R"(
    function update()
      ship.point_prograde()
      ship.set_throttle(1)
    end
  )");

  core::Scenario scenario = scripted_scenario();
  scenario.entities[0].script = script.path();

  simhost::LocalSimSource source = simhost::LocalSimSource::from_scenario(scenario);
  source.set_max_step(60.0);
  source.set_control_period(1.0);

  const proto::EntitySnapshot* start = find_entity(source.snapshot(), "Tug");
  REQUIRE(start != nullptr);
  const double wet_mass = start->mass;

  for (int i = 0; i < 60; ++i) {
    source.pump(1.0);
  }

  const proto::EntitySnapshot* published = find_entity(source.snapshot(), "Tug");
  REQUIRE(published != nullptr);

  // Sixty seconds at 60 kN and 345 s of specific impulse: m_dot = T / (Isp g0)
  // = 17.735 kg/s, so 1064 kg. Constant thrust at constant Isp makes the mass
  // flow constant, so the figure does not depend on the mass falling as it
  // burns and the bound can be tight.
  const double burned = wet_mass - published->mass;
  INFO("burned " << burned << " kg");
  CHECK_THAT(burned, WithinAbs(1064.1, 1.0));
}

TEST_CASE("a script that faults leaves a vessel that can still be flown", "[flight][simhost]") {
  const TempScript script("fault", R"(
    function update()
      error("guidance failure")
    end
  )");

  core::Scenario scenario = scripted_scenario();
  scenario.entities[0].script = script.path();

  simhost::LocalSimSource source = simhost::LocalSimSource::from_scenario(scenario);
  const std::uint64_t tug = source.snapshot().selected;

  for (int i = 0; i < 10; ++i) {
    source.pump(1.0);
  }

  const proto::EntitySnapshot* published = find_entity(source.snapshot(), "Tug");
  REQUIRE(published != nullptr);
  CHECK(published->computer == proto::ComputerState::Faulted);
  INFO(std::string(published->computer_message.view()));
  CHECK(published->computer_message.view().find("guidance failure") != std::string_view::npos);

  // The vessel is still there and still answers the controls a client has: a
  // fault takes the autopilot out, not the ship.
  const double before = published->mass;
  source.send(proto::Command::set_throttle(1.0, tug));
  source.pump(1.0);
  CHECK(find_entity(source.snapshot(), "Tug")->mass < before);
}

TEST_CASE("a computer that faults under thrust does not leave the engine lit", "[flight][simhost]") {
  // A fault is not a tidy ending: the script may have been at full throttle on
  // the tick before it died. If the host left that command on the books the
  // vessel would burn until someone noticed, which is precisely the quiet
  // disaster a flight computer is supposed to prevent.
  const TempScript script("fault-thrust", R"(
    local countdown = 3
    function update()
      ship.point_prograde()
      ship.set_throttle(1)
      countdown = countdown - 1
      if countdown <= 0 then
        error("guidance failure under thrust")
      end
    end
  )");

  core::Scenario scenario = scripted_scenario();
  scenario.entities[0].script = script.path();

  simhost::LocalSimSource source = simhost::LocalSimSource::from_scenario(scenario);

  // Three ticks of thrust, then the fault. One second of control period, so one
  // tick per pump.
  for (int i = 0; i < 3; ++i) {
    source.pump(1.0);
  }
  const proto::EntitySnapshot* faulted = find_entity(source.snapshot(), "Tug");
  REQUIRE(faulted != nullptr);
  REQUIRE(faulted->computer == proto::ComputerState::Faulted);
  CHECK_THAT(faulted->throttle, WithinAbs(0.0, 1e-9));

  // And it stays off: a stopped program is not run again, and nothing puts the
  // throttle back in its hand.
  const double mass = faulted->mass;
  for (int i = 0; i < 20; ++i) {
    source.pump(1.0);
  }
  const proto::EntitySnapshot* after = find_entity(source.snapshot(), "Tug");
  REQUIRE(after != nullptr);
  CHECK_THAT(after->mass, WithinAbs(mass, 1e-9));
}

TEST_CASE("a script that will not load stops the scenario loading", "[flight][simhost]") {
  core::Scenario missing = scripted_scenario();
  missing.entities[0].script = "/tmp/rocketlab-no-such-script.lua";
  CHECK_THROWS(simhost::LocalSimSource::from_scenario(missing));

  const TempScript broken("broken", "function update( end");
  core::Scenario syntax = scripted_scenario();
  syntax.entities[0].script = broken.path();
  CHECK_THROWS(simhost::LocalSimSource::from_scenario(syntax));
}

TEST_CASE("a predicted path flies the script, not a coast", "[flight][simhost]") {
  const TempScript script("predict", R"(
    function update()
      ship.point_prograde()
      ship.set_throttle(1)
    end
  )");

  core::Scenario scenario = scripted_scenario();
  scenario.entities[0].script = script.path();

  simhost::LocalSimSource source = simhost::LocalSimSource::from_scenario(scenario);
  const std::uint64_t tug = source.snapshot().selected;
  const std::uint64_t bystander = find_entity(source.snapshot(), "Bystander")->id;

  std::vector<proto::Vec3d> path;
  REQUIRE(source.query_trajectory(tug, 600.0, path));
  REQUIRE(path.size() > 2);

  std::vector<proto::Vec3d> coast;
  REQUIRE(source.query_trajectory(bystander, 600.0, coast));
  REQUIRE(coast.size() == path.size());

  const auto radius_of = [](const proto::Vec3d& p) {
    return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
  };
  const double scripted_radius = radius_of(path.back());
  const double coast_radius = radius_of(coast.back());
  INFO("scripted " << scripted_radius << " vs coasting " << coast_radius);
  // Ten minutes of full throttle on a 5 t vessel is 5 km/s, past escape
  // velocity, so the far end of the scripted path is not merely higher — it is
  // in a different orbit entirely. A coast would have come back round to the
  // radius it started at.
  CHECK(scripted_radius > coast_radius + 1000.0);
}

TEST_CASE("predicting a trajectory does not disturb the program it clones", "[flight][simhost]") {
  // Cloning is what makes the prediction honest, and the price of getting it
  // wrong is that asking for a picture of the future changes the future.
  const TempScript script("count", R"(
    local tick = 0
    function update()
      tick = tick + 1
      ship.set_throttle(1)
      ship.log(tick)
    end
  )");

  core::Scenario scenario = scripted_scenario();
  scenario.entities[0].script = script.path();

  simhost::LocalSimSource source = simhost::LocalSimSource::from_scenario(scenario);
  const std::uint64_t tug = source.snapshot().selected;

  source.pump(1.0);
  source.pump(1.0);
  const auto before =
      static_cast<std::uint64_t>(find_entity(source.snapshot(), "Tug")->computer_instructions);

  std::vector<proto::Vec3d> path;
  for (int i = 0; i < 5; ++i) {
    REQUIRE(source.query_trajectory(tug, 600.0, path));
  }

  source.pump(1.0);
  const auto after =
      static_cast<std::uint64_t>(find_entity(source.snapshot(), "Tug")->computer_instructions);

  // Two ticks have run, so one tick's worth is about half of `before`, and then
  // the live program has run exactly one more. A clone that leaked into the
  // counter would show up here as five predictions' worth of instructions.
  const std::uint64_t one_tick = before / 2;
  INFO("before " << before << ", after " << after << ", one tick " << one_tick);
  CHECK(after >= before + one_tick);
  CHECK(after <= before + 2 * one_tick);
}

TEST_CASE("a computer can be switched off without being unloaded", "[flight][simhost]") {
  const TempScript script("toggle", R"(
    function update() ship.set_throttle(1) end
  )");

  core::Scenario scenario = scripted_scenario();
  scenario.entities[0].script = script.path();

  simhost::LocalSimSource source = simhost::LocalSimSource::from_scenario(scenario);
  const std::uint64_t tug = source.snapshot().selected;

  source.pump(1.0);
  const double after_one = find_entity(source.snapshot(), "Tug")->propellant;

  // Switched off, and with the throttle taken back by hand, nothing should move.
  // This is the case time warp needs: a daemon stepping hours at a time cannot
  // afford to run Lua on every second of it.
  source.send(proto::Command::set_computer(false, tug));
  source.send(proto::Command::set_throttle(0.0, tug));
  for (int i = 0; i < 5; ++i) {
    source.pump(1.0);
  }
  const double while_off = find_entity(source.snapshot(), "Tug")->propellant;
  CHECK_THAT(while_off, WithinAbs(after_one, 1e-9));

  source.send(proto::Command::set_computer(true, tug));
  source.pump(1.0);
  CHECK(find_entity(source.snapshot(), "Tug")->propellant < while_off);
}

TEST_CASE("the shipped autopilot flies a Hohmann transfer", "[flight][simhost]") {
  // The end-to-end check: the file in `scripts/`, on the tug in `scenarios/`,
  // flown by the host. Every other test here proves one piece; this one is the
  // only one that proves they fit together, and it is also the thing a person
  // would actually do.
  core::Scenario scripted = scenario::load_scenario_file(source_path("scenarios/tug.json"));
  bool attached = false;
  for (core::ScenarioEntity& entity : scripted.entities) {
    if (entity.name == "Tug") {
      entity.script = source_path("scripts/hohmann.lua");
      attached = true;
    }
  }
  REQUIRE(attached);

  simhost::LocalSimSource source = simhost::LocalSimSource::from_scenario(scripted);

  // A one-second control loop lets the circularisation burn overshoot by a whole
  // second's worth of thrust, which at 650 kN on a 40 t stack raises the
  // periapsis by some 60 km in the last tick. A fifth of a second holds that to
  // about 12 km, which makes the assertions below be about the transfer rather
  // than about the tick size.
  source.set_control_period(0.2);

  const std::uint64_t tug = source.snapshot().selected;
  REQUIRE(tug != 0);
  const double earth_radius =
      body_radius(source.snapshot(), find_entity(source.snapshot(), "Tug")->parent);
  REQUIRE(earth_radius > 6.0e6);

  // The whole transfer is about fifty minutes: a thirteen second raise, a fifty
  // minute coast, and a thirteen second circularisation. Four hours is a
  // generous ceiling, and the loop stops early once the script reports it is
  // done.
  int pumps = 0;
  for (; pumps < 4 * 3600; ++pumps) {
    source.pump(1.0);
    if (find_entity(source.snapshot(), "Tug")->computer == proto::ComputerState::Finished) {
      break;
    }
  }

  const proto::EntitySnapshot* now = find_entity(source.snapshot(), "Tug");
  REQUIRE(now != nullptr);
  INFO(std::string(now->computer_message.view()));
  INFO("pumps " << pumps);

  REQUIRE(pumps < 4 * 3600);
  CHECK(now->computer == proto::ComputerState::Finished);

  const double periapsis_km = (now->periapsis - earth_radius) / 1000.0;
  const double apoapsis_km = (now->apoapsis - earth_radius) / 1000.0;
  INFO("periapsis " << periapsis_km << " km, apoapsis " << apoapsis_km << " km, e "
                    << now->eccentricity);

  // Circular at 1200 km: a Hohmann transfer from 400 km is 417 m/s, and it takes
  // a few minutes of thrusting a 40 t stack to find it. Both apsides overshoot a
  // little, and for two different reasons: the raise burn is cut on the tick
  // *after* it sees the apoapsis cross the target, and the circularisation starts
  // a fixed 25 s early because the script is not clever enough to work out the
  // burn time. Sixty kilometres on a six-hour transfer is the script's aim, not
  // the sandbox's precision, which is what this test is really about.
  CHECK_THAT(periapsis_km, WithinAbs(1200.0, 60.0));
  CHECK_THAT(apoapsis_km, WithinAbs(1200.0, 60.0));
  CHECK(now->eccentricity < 0.01);
  // And it got there by burning, not by drifting: most of the stage is still
  // there, but not all of it.
  CHECK(now->propellant < now->propellant_capacity);
  CHECK(now->propellant > 0.0);
}
