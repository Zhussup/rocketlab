// Headless driver for the simulation core.
//
// This is the M0 deliverable and the thing the test suite cannot be: a way to
// actually fly a scenario and look at the numbers. It deliberately links no
// UI library, so it stays usable over ssh and inside the test harness.

#include <cstdio>
#include <cstdlib>
#include <format>
#include <fstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "rocketlab/core/world.hpp"
#include "rocketlab/flight/program.hpp"
#include "rocketlab/hud/readout.hpp"
#include "rocketlab/render/scene.hpp"
#include "rocketlab/render/svg.hpp"
#include "rocketlab/render/text.hpp"
#include "rocketlab/scenario/json_io.hpp"
#include "rocketlab/simhost/local_sim_source.hpp"

namespace {

using namespace rocketlab;

constexpr double kRadiansToDegrees = 57.29577951308232;

void print_usage() {
  std::puts(R"(rocketlab - space mission simulator, headless driver

usage:
  rocketlab bodies                     list the body tree and its spheres of influence
  rocketlab parts                      list the part catalogue
  rocketlab scenario [file.json]      load a scenario and print the initial orbits
  rocketlab run [file.json] [opts]    propagate and print the resulting orbits
  rocketlab vessel [file.json] [opts] print the stack and the stage table
  rocketlab map [file.json] [opts]    draw the map the terminal client would show
  rocketlab script <file.lua> [opts]   attach a Lua flight computer and run one control tick
  rocketlab fly [file.json] [opts]    fly a scenario with its Lua flight computers running

options for `run`:
  --duration <time>   how much simulation time to cover (default 1d)
  --step <time>       largest step between event checks (default 60s)
  --warp <factor>     reported alongside, does not change the result

options for `vessel`:
  --entity <name>     which vessel to describe (default the first one with parts)
  --burn              fly it until it runs dry, then report what it achieved

options for `map`:
  --width <cells>     canvas width (default 96)
  --height <cells>    canvas height (default 32)
  --zoom <m/cell>     metres to a character cell (default 1e6)
  --entity <name>     what the camera centres on (default the first vessel)
  --svg <file>        also write the scene as SVG

options for `script`:
  --scenario <f.json> which scenario to attach it to (default the built-in one)
  --entity <name>     which entity to attach it to (default the first with parts)
  --control <time>    how often the computer is run (default 1s)

options for `fly`:
  --script <e>=<f>    attach the Lua file <f> to the entity named <e> (repeatable)
  --duration <time>   how much simulation time to fly (default 30m)
  --control <time>    how often the computers are run (default 1s)
  --report <time>     how often to print a status line (default 2m)

A script may also be named in the scenario itself with a `script` field, which
`fly` combines with anything given on the command line.

durations accept a unit suffix: 90s, 45m, 6h, 3d, or a bare number of seconds.
without a file the built-in default scenario is used.
)");
}

/// Parses "90", "90s", "45m", "6h", "3d". Returns seconds.
[[nodiscard]] double parse_duration(std::string_view text) {
  if (text.empty()) {
    throw std::runtime_error("empty duration");
  }
  std::size_t consumed = 0;
  double value = 0.0;
  try {
    value = std::stod(std::string(text), &consumed);
  } catch (const std::exception&) {
    throw std::runtime_error("cannot read a number from duration '" + std::string(text) + "'");
  }

  const std::string_view suffix = text.substr(consumed);
  if (suffix.empty() || suffix == "s") {
    return value;
  }
  if (suffix == "m" || suffix == "min") {
    return value * 60.0;
  }
  if (suffix == "h") {
    return value * 3600.0;
  }
  if (suffix == "d") {
    return value * core::kSecondsPerDay;
  }
  throw std::runtime_error("unknown duration unit '" + std::string(suffix) + "'");
}

struct OrbitSummary {
  double periapsis_altitude{0.0};
  double apoapsis_altitude{0.0};
  double inclination_deg{0.0};
  double eccentricity{0.0};
  double period_seconds{0.0};
};

[[nodiscard]] OrbitSummary summarise(const core::StateVector& state,
                                     const core::CelestialBody& body) {
  const core::OrbitalElements el = core::rv_to_elements(state, body.mu);
  OrbitSummary summary;
  if (el.degenerate) {
    return summary;
  }
  summary.periapsis_altitude = el.periapsis() - body.radius;
  summary.apoapsis_altitude = el.apoapsis() - body.radius;
  summary.inclination_deg = el.i * kRadiansToDegrees;
  summary.eccentricity = el.e;
  summary.period_seconds = el.period(body.mu);
  return summary;
}

[[nodiscard]] std::string format_altitude(double metres) {
  if (metres >= 1e9) {
    return std::format("{:.3f} Gm", metres / 1e9);
  }
  if (metres >= 1e6) {
    return std::format("{:.3f} Mm", metres / 1e6);
  }
  if (metres >= 1e3) {
    return std::format("{:.3f} km", metres / 1e3);
  }
  return std::format("{:.1f} m", metres);
}

[[nodiscard]] std::string format_period(double seconds) {
  if (!(seconds > 0.0) || seconds > 1e12) {
    return "open";
  }
  return core::format_duration(seconds);
}

int command_bodies() {
  const core::BodySystem system = core::BodySystem::solar_system();
  std::puts(std::format("{:<10} {:>14} {:>16} {:>14}", "body", "mu [m3/s2]", "radius", "SOI").c_str());
  for (const core::CelestialBody& body : system.bodies()) {
    const std::string parent =
        body.is_root() ? "-" : system.body(body.parent).name;
    std::printf("%-10s %14.4e %16s %14s   (around %s)\n", body.name.c_str(), body.mu,
                format_altitude(body.radius).c_str(),
                body.is_root() ? "-" : format_altitude(body.soi_radius).c_str(), parent.c_str());
  }
  return EXIT_SUCCESS;
}

int command_parts() {
  const core::PartCatalogue catalogue = core::PartCatalogue::stock();
  std::puts(std::format("{:<18} {:<10} {:>10} {:>12} {:>10} {:>7} {:>8} {:>8}", "part", "kind",
                        "dry [kg]", "prop [kg]", "thrust", "isp [s]", "radius", "length")
                .c_str());
  for (const core::Part& part : catalogue.parts()) {
    const std::string thrust =
        part.thrust > 0.0 ? hud::format_thrust(part.thrust) : std::string("-");
    const std::string isp = part.isp > 0.0 ? std::format("{:.0f}", part.isp) : std::string("-");
    std::printf("%-18s %-10s %10.1f %12.1f %10s %7s %8.2f %8.2f\n", part.name.c_str(),
                core::to_string(part.kind), part.dry_mass, part.propellant, thrust.c_str(),
                isp.c_str(), part.radius, part.length);
  }
  std::printf("\n%zu parts\n", catalogue.size());
  return EXIT_SUCCESS;
}

/// The stack and the stage table, which is the whole point of M2: the delta-v a
/// design is worth, before anyone has flown it.
int command_vessel(const core::Scenario& scenario, const std::string& entity_name, bool burn) {
  core::World world = core::World::from_scenario(scenario);

  const core::Entity* chosen = nullptr;
  for (const core::Entity& entity : world.entities()) {
    if (entity.vessel.empty()) {
      continue;
    }
    if (entity_name.empty() || entity.name == entity_name) {
      chosen = &entity;
      break;
    }
  }
  if (chosen == nullptr) {
    std::fprintf(stderr, "rocketlab: no vessel with parts%s%s\n",
                 entity_name.empty() ? "" : " named ", entity_name.c_str());
    return EXIT_FAILURE;
  }

  const core::EntityId id = chosen->id;
  std::printf("vessel '%s' in '%s', %zu parts\n\n", chosen->name.c_str(), scenario.name.c_str(),
              chosen->vessel.parts.size());

  const auto print_stack = [](const core::Entity& entity) {
    std::puts(std::format("{:<3} {:<18} {:<10} {:>11} {:>11}", "stg", "part", "kind", "dry [kg]",
                          "prop [kg]")
                  .c_str());
    for (std::size_t i = 0; i < entity.vessel.parts.size(); ++i) {
      const core::Part& part = entity.vessel.parts[i];
      std::printf("%-3d %-18s %-10s %11.1f %11.1f\n", entity.vessel.stage[i], part.name.c_str(),
                  core::to_string(part.kind), part.dry_mass, entity.vessel.propellant[i]);
    }
    std::printf("%-3s %-18s %-10s %11.1f %11.1f\n", "", "total", "", entity.vessel.mass(),
                entity.vessel.propellant_left());
  };

  const auto print_table = [](const core::Vessel& vessel, std::string_view heading) {
    std::printf("\n%s\n", std::string(heading).c_str());
    std::puts(std::format("{:<5} {:>12} {:>12} {:>12} {:>10} {:>7} {:>12} {:>12}", "stage",
                          "m0 [kg]", "m1 [kg]", "prop [kg]", "thrust", "isp [s]", "burn",
                          "delta-v")
                  .c_str());
    for (const core::StageReport& stage : core::stage_table(vessel)) {
      const std::string burn_time = stage.burn_time > 0.0 && stage.burn_time < 1e9
                                        ? core::format_duration(stage.burn_time)
                                        : std::string("no engine");
      std::printf("%-5d %12.0f %12.0f %12.0f %10s %7.0f %12s %12s\n", stage.index,
                  stage.ignition_mass, stage.final_mass, stage.propellant,
                  hud::format_thrust(stage.thrust).c_str(), stage.isp, burn_time.c_str(),
                  hud::format_speed(stage.delta_v).c_str());
    }
    std::printf("total delta-v %s\n", hud::format_speed(core::total_delta_v(vessel)).c_str());
  };

  print_stack(*chosen);
  print_table(chosen->vessel, "stage table, full tanks");

  if (!burn) {
    std::printf("\ndelta-v remaining %s\n",
                hud::format_speed(core::remaining_delta_v(chosen->vessel)).c_str());
    return EXIT_SUCCESS;
  }

  // Fly it. The burn is the check on the table above: a stage table is a
  // prediction, and a prediction nobody ever lands on is a guess.
  const double before = core::norm(world.find(id)->state.v);
  const core::Vec3 before_v = world.find(id)->state.v;
  world.find(id)->vessel.throttle = 1.0;

  double elapsed = 0.0;
  while (elapsed < 4.0 * 3600.0) {
    core::Entity* entity = world.find(id);
    if (entity == nullptr || !entity->vessel.can_thrust()) {
      break;
    }
    // One-second steps so that "how long did the burn take" is answered to
    // within a second of the truth rather than within a step of it.
    world.advance(1.0, 1.0);
    elapsed += 1.0;
  }

  const core::Entity* after = world.find(id);
  if (after == nullptr) {
    std::fprintf(stderr, "rocketlab: the vessel disappeared during the burn\n");
    return EXIT_FAILURE;
  }
  const double delta_v = core::norm(after->state.v - before_v);
  std::printf("\nburn for %s on the current stage\n", core::format_duration(elapsed).c_str());
  std::printf("  speed %s -> %s  (%s of it)\n", hud::format_speed(before).c_str(),
              hud::format_speed(core::norm(after->state.v)).c_str(),
              hud::format_speed(delta_v).c_str());
  std::printf("  mass %s, propellant left %s, delta-v left %s\n",
              hud::format_mass(after->mass).c_str(),
              hud::format_mass(after->vessel.propellant_left()).c_str(),
              hud::format_speed(core::remaining_delta_v(after->vessel)).c_str());
  return EXIT_SUCCESS;
}

void print_orbits(const core::World& world, std::string_view heading) {
  std::puts(std::format("\n{}  ({})", heading, core::format_met(world.clock().tdb())).c_str());
  std::puts(std::format("{:<12} {:<8} {:>14} {:>14} {:>8} {:>12}", "entity", "parent", "periapsis",
                        "apoapsis", "inc", "period")
                .c_str());
  for (const core::Entity& entity : world.entities()) {
    const core::CelestialBody& body = world.bodies().body(entity.parent);
    const OrbitSummary orbit = summarise(entity.state, body);
    std::printf("%-12s %-8s %14s %14s %7.2f° %12s\n", entity.name.c_str(), body.name.c_str(),
                format_altitude(orbit.periapsis_altitude).c_str(),
                format_altitude(orbit.apoapsis_altitude).c_str(), orbit.inclination_deg,
                format_period(orbit.period_seconds).c_str());
  }
}

int command_run(const core::Scenario& scenario, double duration, double step, double warp) {
  core::World world = core::World::from_scenario(scenario);
  std::printf("scenario '%s' at %s, %zu entities\n", scenario.name.c_str(),
              core::format_met(scenario.epoch.tdb).c_str(), scenario.entities.size());
  print_orbits(world, "initial state");

  world.advance(duration, step);
  print_orbits(world, std::format("after {}", core::format_duration(duration)));

  // Sanity line: with analytic propagation a bound orbit must come back to
  // where it started after a whole number of periods, and an unperturbed one
  // must not drift in shape. Printing the error makes that visible.
  for (const core::Entity& entity : world.entities()) {
    const core::StateVector root = world.root_state(entity);
    std::printf("%-12s root-frame |r| = %s\n", entity.name.c_str(),
                format_altitude(core::norm(root.r)).c_str());
  }
  if (warp != 1.0) {
    std::printf("(reported at warp x%.0f; analytic propagation makes the result independent of it)\n",
                warp);
  }
  return EXIT_SUCCESS;
}

/// Draws the map the terminal client would show, as characters.
///
/// This is the same scene the TUI paints: one camera, one set of primitives,
/// two backends. It exists so the map can be looked at without a terminal, and
/// so the drawing path has a check that does not depend on someone noticing a
/// picture is wrong.
int command_map(const core::Scenario& scenario, int width, int height, double metres_per_pixel,
                const std::string& entity_name, const std::string& svg_path) {
  simhost::LocalSimSource host = simhost::LocalSimSource::from_scenario(scenario);
  const proto::Snapshot& snapshot = host.snapshot();

  // Pick the target: by name if asked, otherwise the host's own choice.
  std::uint64_t target = snapshot.selected;
  if (!entity_name.empty()) {
    target = 0;
    for (std::uint32_t i = 0; i < snapshot.entity_count; ++i) {
      if (snapshot.entities[i].name.view() == entity_name) {
        target = snapshot.entities[i].id;
        break;
      }
    }
    if (target == 0) {
      std::fprintf(stderr, "rocketlab: no entity named '%s'\n", entity_name.c_str());
      return EXIT_FAILURE;
    }
  }

  render::Camera2D camera;
  camera.width = width;
  camera.height = height;
  camera.cell_aspect = 2.0;
  camera.metres_per_pixel = metres_per_pixel;
  camera.following = true;
  camera.target = target;

  const proto::EntitySnapshot* entity = hud::find_entity(snapshot, target);
  if (entity != nullptr) {
    render::follow_target(snapshot, camera);
  }

  std::vector<proto::Vec3d> trajectory;
  double horizon = entity != nullptr ? entity->period : 0.0;
  if (!(horizon > 0.0) || horizon > 4.0e7) {
    horizon = 3600.0;
  }
  if (target != 0 && !host.query_trajectory(target, horizon, trajectory)) {
    trajectory.clear();
  }

  render::Scene scene;
  render::build_scene(snapshot, camera, {}, trajectory, scene);

  const hud::EntityLine* chosen = nullptr;
  const std::vector<hud::EntityLine> lines = hud::entity_list(snapshot);
  for (const hud::EntityLine& line : lines) {
    if (line.id == target) {
      chosen = &line;
    }
  }
  std::printf("map: %s at %s, %.6g m per cell, centred on %s\n", scenario.name.c_str(),
              core::format_met(snapshot.tdb).c_str(), metres_per_pixel,
              chosen != nullptr ? chosen->name.c_str() : "(nothing)");
  std::printf("%s", render::to_text(scene).c_str());
  std::puts("(one '#' is a body, 'O' a vessel, 'x' debris, '*' the predicted path,\n"
            " '+' the selection, '.' the grid, and the scale bar sits bottom left)");

  if (!svg_path.empty()) {
    std::ofstream out(svg_path);
    if (!out) {
      std::fprintf(stderr, "rocketlab: cannot write '%s'\n", svg_path.c_str());
      return EXIT_FAILURE;
    }
    out << render::to_svg(scene);
    std::printf("wrote %s\n", svg_path.c_str());
  }
  return EXIT_SUCCESS;
}

/// Splits `<entity>=<path>`, the form `--script` takes.
[[nodiscard]] bool split_assignment(std::string_view text, std::string& name, std::string& path) {
  const std::size_t at = text.find('=');
  if (at == std::string_view::npos || at == 0 || at + 1 >= text.size()) {
    return false;
  }
  name = std::string(text.substr(0, at));
  path = std::string(text.substr(at + 1));
  return true;
}

/// The body an entity orbits, looked up by id in the snapshot's own body table.
///
/// Not hardcoded, because the altitude of an orbit is measured from that body's
/// surface and the table is right there on the wire.
[[nodiscard]] double body_radius_of(const proto::Snapshot& snapshot, std::uint32_t parent) {
  for (std::uint32_t i = 0; i < snapshot.body_count; ++i) {
    if (snapshot.bodies[i].id == parent) {
      return snapshot.bodies[i].radius;
    }
  }
  return 0.0;
}

[[nodiscard]] double altitude_of(const proto::Snapshot& snapshot,
                                 const proto::EntitySnapshot& entity) {
  const double radius = core::norm(core::Vec3{entity.position.x, entity.position.y,
                                              entity.position.z});
  return radius - body_radius_of(snapshot, entity.parent);
}

/// The orbit table, read off the snapshot the host publishes rather than off a
/// World, because that is what a client can see.
void print_snapshot_orbits(const proto::Snapshot& snapshot) {
  std::puts(std::format("\n{:<12} {:>14} {:>14} {:>8} {:>12} {:<10} {}", "entity", "periapsis",
                        "apoapsis", "inc", "period", "computer", "message")
                .c_str());
  for (std::uint32_t i = 0; i < snapshot.entity_count; ++i) {
    const proto::EntitySnapshot& entity = snapshot.entities[i];
    const double periapsis = entity.periapsis - body_radius_of(snapshot, entity.parent);
    const double apoapsis = entity.apoapsis - body_radius_of(snapshot, entity.parent);
    const double period = entity.period;
    std::printf("%-12s %14s %14s %7.2f° %12s %-10s %s\n",
                std::string(entity.name.view()).c_str(), format_altitude(periapsis).c_str(),
                format_altitude(apoapsis).c_str(), entity.inclination * kRadiansToDegrees,
                format_period(period).c_str(), hud::computer_state_name(entity.computer),
                std::string(entity.computer_message.view()).c_str());
  }
}

/// Flies a scenario with its flight computers running.
///
/// The scripts are named on the command line rather than only in the file, so
/// that the same mission can be flown with and without an autopilot without
/// editing the scenario — which is the only way to see what the autopilot
/// actually changed.
int command_fly(const core::Scenario& given, const std::vector<std::string>& assignments,
                double duration, double control, double report) {
  core::Scenario scenario = given;
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

  simhost::LocalSimSource host = simhost::LocalSimSource::from_scenario(scenario);
  host.set_control_period(control);

  std::printf("scenario '%s' at %s: %zu entities, %zu flight computers, control every %s\n",
              scenario.name.c_str(), core::format_met(scenario.epoch.tdb).c_str(),
              scenario.entities.size(), host.computers().size(),
              core::format_duration(control).c_str());
  print_snapshot_orbits(host.snapshot());

  // A script is otherwise invisible: without this a reader sees a vessel
  // steering with nobody's hands on the controls and has to take it on trust.
  std::vector<std::string> last_line(host.computers().size());
  const auto report_computers = [&]() {
    const proto::Snapshot& snapshot = host.snapshot();
    for (std::size_t i = 0; i < host.computers().size(); ++i) {
      const simhost::ComputerAttachment& attachment = host.computers()[i];
      std::string line = std::format("[{}] {}: {}", core::format_met(snapshot.tdb),
                                     attachment.path, flight::to_string(attachment.status));
      if (!attachment.message.empty()) {
        line += " — " + attachment.message;
      }
      if (line != last_line[i]) {
        std::puts(line.c_str());
        last_line[i] = std::move(line);
      }
    }
  };
  report_computers();

  for (double met = 0.0; met < duration;) {
    const double span = std::min(report, duration - met);
    host.pump(span);
    met += span;
    report_computers();

    const proto::Snapshot& snapshot = host.snapshot();
    const proto::EntitySnapshot* entity = hud::find_entity(snapshot, snapshot.selected);
    std::printf("  t+%-8s %-12s alt %14s  apo %14s  prop %10s  dv %10s\n",
                core::format_met(snapshot.tdb).c_str(),
                entity != nullptr ? std::string(entity->name.view()).c_str() : "-",
                entity != nullptr ? hud::format_length(altitude_of(snapshot, *entity)).c_str() : "-",
                entity != nullptr ? hud::format_length(entity->apoapsis - body_radius_of(snapshot, entity->parent)).c_str() : "-",
                entity != nullptr ? hud::format_mass(entity->propellant).c_str() : "-",
                entity != nullptr ? hud::format_speed(entity->delta_v).c_str() : "-");
  }

  print_snapshot_orbits(host.snapshot());
  return EXIT_SUCCESS;
}

/// Compile-checks a flight computer, then runs one tick of it.
///
/// Loading is the moment a mistake in a script should surface, not an hour into
/// a mission, and the second half answers the question that follows: what does
/// it actually do. The tick is run through the same host a client would use, so
/// what is printed is what the mission would see.
int command_script(const std::string& path, const core::Scenario& given,
                   const std::string& entity_name, double control) {
  core::Scenario scenario = given;

  // A script needs a vessel, and the default scenario is a station with no
  // parts. Attaching to the first entity that has some means the check works on
  // any scenario rather than only on ones written for it.
  core::ScenarioEntity* target = nullptr;
  for (core::ScenarioEntity& entity : scenario.entities) {
    if (!entity_name.empty()) {
      if (entity.name == entity_name) {
        target = &entity;
        break;
      }
      continue;
    }
    if (!entity.parts.empty()) {
      target = &entity;
      break;
    }
    if (target == nullptr) {
      target = &entity;
    }
  }
  if (target == nullptr) {
    throw std::runtime_error(entity_name.empty()
                                 ? "the scenario has no entity to attach a script to"
                                 : "no entity named '" + entity_name + "'");
  }
  target->script = path;

  simhost::LocalSimSource host = simhost::LocalSimSource::from_scenario(scenario);
  host.set_control_period(control);
  std::printf("'%s' loaded onto %s in '%s'\n", path.c_str(), target->name.c_str(),
              scenario.name.c_str());

  host.pump(control);

  const simhost::ComputerAttachment& attachment = host.computers().front();
  std::printf("  after one %s tick: %s, %s instructions\n", core::format_duration(control).c_str(),
              flight::to_string(attachment.status), std::to_string(attachment.instructions()).c_str());
  if (!attachment.message.empty()) {
    std::printf("  message: %s\n", attachment.message.c_str());
  }

  // Looked up by name rather than by `snapshot.selected`: the camera follows the
  // first controllable entity, which is not necessarily the one that was just
  // handed a script.
  const proto::Snapshot& snapshot = host.snapshot();
  const proto::EntitySnapshot* entity = nullptr;
  for (std::uint32_t i = 0; i < snapshot.entity_count; ++i) {
    if (snapshot.entities[i].name.view() == target->name) {
      entity = &snapshot.entities[i];
      break;
    }
  }
  if (entity != nullptr) {
    std::printf("  %s is at %s, %s, %s of delta-v left\n", std::string(entity->name.view()).c_str(),
                hud::format_length(altitude_of(snapshot, *entity)).c_str(),
                hud::format_speed(core::norm(core::Vec3{entity->velocity.x, entity->velocity.y,
                                                        entity->velocity.z}))
                    .c_str(),
                hud::format_speed(entity->delta_v).c_str());
  }
  return EXIT_SUCCESS;
}

[[nodiscard]] std::string value_after(const std::vector<std::string>& args, std::size_t& index,
                                      const char* flag) {
  if (index + 1 >= args.size()) {
    throw std::runtime_error(std::string(flag) + " needs a value");
  }
  return args[++index];
}

}  // namespace

int main(int argc, char** argv) {
  std::vector<std::string> args(argv + 1, argv + argc);
  if (args.empty() || args[0] == "-h" || args[0] == "--help") {
    print_usage();
    return args.empty() ? EXIT_FAILURE : EXIT_SUCCESS;
  }

  const std::string command = args[0];

  try {
    if (command == "bodies") {
      return command_bodies();
    }

    if (command == "parts") {
      return command_parts();
    }

    if (command == "scenario" || command == "run") {
      std::string path;
      double duration = core::kSecondsPerDay;
      double step = 60.0;
      double warp = 1.0;

      for (std::size_t i = 1; i < args.size(); ++i) {
        const std::string& arg = args[i];
        if (arg == "--duration") {
          duration = parse_duration(value_after(args, i, "--duration"));
        } else if (arg == "--step") {
          step = parse_duration(value_after(args, i, "--step"));
        } else if (arg == "--warp") {
          warp = std::stod(value_after(args, i, "--warp"));
        } else if (!arg.empty() && arg[0] == '-') {
          throw std::runtime_error("unknown option '" + arg + "'");
        } else if (path.empty()) {
          path = arg;
        } else {
          throw std::runtime_error("more than one scenario file given");
        }
      }

      const core::Scenario scenario =
          path.empty() ? scenario::default_scenario() : scenario::load_scenario_file(path);

      if (command == "scenario") {
        std::printf("%s", scenario::write_scenario(scenario).c_str());
        return EXIT_SUCCESS;
      }
      return command_run(scenario, duration, step, warp);
    }

    if (command == "vessel") {
      std::string path;
      std::string entity_name;
      bool burn = false;

      for (std::size_t i = 1; i < args.size(); ++i) {
        const std::string& arg = args[i];
        if (arg == "--entity") {
          entity_name = value_after(args, i, "--entity");
        } else if (arg == "--burn") {
          burn = true;
        } else if (!arg.empty() && arg[0] == '-') {
          throw std::runtime_error("unknown option '" + arg + "'");
        } else if (path.empty()) {
          path = arg;
        } else {
          throw std::runtime_error("more than one scenario file given");
        }
      }

      const core::Scenario scenario =
          path.empty() ? scenario::default_scenario() : scenario::load_scenario_file(path);
      return command_vessel(scenario, entity_name, burn);
    }

    if (command == "map") {
      std::string path;
      std::string entity_name;
      std::string svg_path;
      int width = 96;
      int height = 32;
      double zoom = 1.0e6;

      for (std::size_t i = 1; i < args.size(); ++i) {
        const std::string& arg = args[i];
        if (arg == "--width") {
          width = std::stoi(value_after(args, i, "--width"));
        } else if (arg == "--height") {
          height = std::stoi(value_after(args, i, "--height"));
        } else if (arg == "--zoom") {
          zoom = std::stod(value_after(args, i, "--zoom"));
        } else if (arg == "--entity") {
          entity_name = value_after(args, i, "--entity");
        } else if (arg == "--svg") {
          svg_path = value_after(args, i, "--svg");
        } else if (!arg.empty() && arg[0] == '-') {
          throw std::runtime_error("unknown option '" + arg + "'");
        } else if (path.empty()) {
          path = arg;
        } else {
          throw std::runtime_error("more than one scenario file given");
        }
      }

      if (width < 4 || height < 4) {
        throw std::runtime_error("--width and --height must be at least 4");
      }
      if (!(zoom > 0.0)) {
        throw std::runtime_error("--zoom must be positive");
      }

      const core::Scenario scenario =
          path.empty() ? scenario::default_scenario() : scenario::load_scenario_file(path);
      return command_map(scenario, width, height, zoom, entity_name, svg_path);
    }

    if (command == "script") {
      std::string script_path;
      std::string scenario_path;
      std::string entity_name;
      double control = 1.0;

      for (std::size_t i = 1; i < args.size(); ++i) {
        const std::string& arg = args[i];
        if (arg == "--control") {
          control = parse_duration(value_after(args, i, "--control"));
        } else if (arg == "--scenario") {
          scenario_path = value_after(args, i, "--scenario");
        } else if (arg == "--entity") {
          entity_name = value_after(args, i, "--entity");
        } else if (!arg.empty() && arg[0] == '-') {
          throw std::runtime_error("unknown option '" + arg + "'");
        } else if (script_path.empty()) {
          script_path = arg;
        } else {
          throw std::runtime_error("more than one script given");
        }
      }
      if (script_path.empty()) {
        throw std::runtime_error("script needs the path to a Lua file");
      }
      if (!(control > 0.0)) {
        throw std::runtime_error("--control must be positive");
      }

      const core::Scenario scenario = scenario_path.empty()
                                          ? scenario::default_scenario()
                                          : scenario::load_scenario_file(scenario_path);
      return command_script(script_path, scenario, entity_name, control);
    }

    if (command == "fly") {
      std::string path;
      std::vector<std::string> assignments;
      double duration = 30.0 * 60.0;
      double control = 1.0;
      double report = 2.0 * 60.0;

      for (std::size_t i = 1; i < args.size(); ++i) {
        const std::string& arg = args[i];
        if (arg == "--script") {
          assignments.push_back(value_after(args, i, "--script"));
        } else if (arg == "--duration") {
          duration = parse_duration(value_after(args, i, "--duration"));
        } else if (arg == "--control") {
          control = parse_duration(value_after(args, i, "--control"));
        } else if (arg == "--report") {
          report = parse_duration(value_after(args, i, "--report"));
        } else if (!arg.empty() && arg[0] == '-') {
          throw std::runtime_error("unknown option '" + arg + "'");
        } else if (path.empty()) {
          path = arg;
        } else {
          throw std::runtime_error("more than one scenario file given");
        }
      }
      if (!(control > 0.0) || !(report > 0.0) || !(duration > 0.0)) {
        throw std::runtime_error("--duration, --control and --report must be positive");
      }

      const core::Scenario scenario =
          path.empty() ? scenario::default_scenario() : scenario::load_scenario_file(path);
      return command_fly(scenario, assignments, duration, control, report);
    }

    std::fprintf(stderr, "rocketlab: unknown command '%s'\n\n", command.c_str());
    print_usage();
    return EXIT_FAILURE;
  } catch (const std::exception& error) {
    std::fprintf(stderr, "rocketlab: %s\n", error.what());
    return EXIT_FAILURE;
  }
}
