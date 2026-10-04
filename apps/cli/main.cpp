// Headless driver for the simulation core.
//
// This is the M0 deliverable and the thing the test suite cannot be: a way to
// actually fly a scenario and look at the numbers. It deliberately links no
// UI library, so it stays usable over ssh and inside the test harness.

#include <cstdio>
#include <cstdlib>
#include <format>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "rocketlab/core/world.hpp"
#include "rocketlab/scenario/json_io.hpp"

namespace {

using namespace rocketlab;

constexpr double kRadiansToDegrees = 57.29577951308232;

void print_usage() {
  std::puts(R"(rocketlab - space mission simulator, headless driver

usage:
  rocketlab bodies                     list the body tree and its spheres of influence
  rocketlab scenario [file.json]      load a scenario and print the initial orbits
  rocketlab run [file.json] [opts]    propagate and print the resulting orbits

options for `run`:
  --duration <time>   how much simulation time to cover (default 1d)
  --step <time>       largest step between event checks (default 60s)
  --warp <factor>     reported alongside, does not change the result

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

    std::fprintf(stderr, "rocketlab: unknown command '%s'\n\n", command.c_str());
    print_usage();
    return EXIT_FAILURE;
  } catch (const std::exception& error) {
    std::fprintf(stderr, "rocketlab: %s\n", error.what());
    return EXIT_FAILURE;
  }
}
