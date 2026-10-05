#include "rocketlab/scenario/json_io.hpp"

#include <cstdio>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "rocketlab/core/time.hpp"

namespace rocketlab::scenario {
namespace {

using nlohmann::json;

[[noreturn]] void fail(const std::string& what) {
  throw std::runtime_error("rocketlab: scenario: " + what);
}

[[nodiscard]] double read_number(const json& object, const char* key, double fallback) {
  const auto it = object.find(key);
  if (it == object.end() || it->is_null()) {
    return fallback;
  }
  if (!it->is_number()) {
    fail(std::string("field '") + key + "' must be a number");
  }
  return it->get<double>();
}

[[nodiscard]] int read_int(const json& object, const char* key, int fallback) {
  const auto it = object.find(key);
  if (it == object.end() || it->is_null()) {
    return fallback;
  }
  if (!it->is_number_integer()) {
    fail(std::string("field '") + key + "' must be an integer");
  }
  return it->get<int>();
}

[[nodiscard]] std::string read_string(const json& object, const char* key,
                                      const std::string& fallback) {
  const auto it = object.find(key);
  if (it == object.end() || it->is_null()) {
    return fallback;
  }
  if (!it->is_string()) {
    fail(std::string("field '") + key + "' must be a string");
  }
  return it->get<std::string>();
}

/// Parses `YYYY-MM-DDTHH:MM:SS`, where the time part may be omitted. A date
/// alone means midnight.
[[nodiscard]] core::Instant parse_iso_epoch(const std::string& text) {
  core::CalendarDate date;
  int hour = 0;
  int minute = 0;
  double second = 0.0;

  const int fields = std::sscanf(text.c_str(), "%d-%d-%dT%d:%d:%lf", &date.year, &date.month,
                                 &date.day, &hour, &minute, &second);
  if (fields < 3) {
    fail("epoch '" + text + "' is not a date of the form YYYY-MM-DDTHH:MM:SS");
  }
  if (fields < 4) {
    hour = 0;
  }
  if (fields < 5) {
    minute = 0;
  }
  if (fields < 6) {
    second = 0.0;
  }
  if (date.month < 1 || date.month > 12 || date.day < 1 || date.day > 31 || hour < 0 || hour > 23 ||
      minute < 0 || minute > 59 || second < 0.0 || second >= 61.0) {
    fail("epoch '" + text + "' has a field out of range");
  }

  date.hour = hour;
  date.minute = minute;
  date.second = second;
  return core::from_calendar(date);
}

[[nodiscard]] core::ScenarioEntity parse_entity(const json& node) {
  if (!node.is_object()) {
    fail("every entry of 'entities' must be an object");
  }

  core::ScenarioEntity entity;
  entity.name = read_string(node, "name", entity.name);
  entity.parent_body = read_string(node, "parent", entity.parent_body);
  entity.mass = read_number(node, "mass", entity.mass);
  entity.radius = read_number(node, "radius", entity.radius);

  const auto controllable = node.find("controllable");
  if (controllable != node.end() && controllable->is_boolean()) {
    entity.controllable = controllable->get<bool>();
  }

  const auto parts = node.find("parts");
  if (parts != node.end() && !parts->is_null()) {
    if (!parts->is_array()) {
      fail("'parts' must be an array");
    }
    for (const json& entry : *parts) {
      if (!entry.is_object()) {
        fail("every entry of 'parts' must be an object");
      }
      core::StackPart part;
      part.part = read_string(entry, "part", "");
      if (part.part.empty()) {
        fail("a 'parts' entry must name a 'part'");
      }
      part.stage = read_int(entry, "stage", 0);
      if (part.stage < 0) {
        fail("part '" + part.part + "' has a negative stage");
      }
      entity.parts.push_back(std::move(part));
    }
  }

  const auto orbit = node.find("orbit");
  if (orbit != node.end() && !orbit->is_null()) {
    if (!orbit->is_object()) {
      fail("'orbit' must be an object");
    }
    entity.periapsis_altitude = read_number(*orbit, "periapsis_altitude", entity.periapsis_altitude);
    entity.apoapsis_altitude = read_number(*orbit, "apoapsis_altitude", entity.apoapsis_altitude);
    entity.inclination_deg = read_number(*orbit, "inclination_deg", entity.inclination_deg);
    entity.raan_deg = read_number(*orbit, "raan_deg", entity.raan_deg);
    entity.argp_deg = read_number(*orbit, "argp_deg", entity.argp_deg);
    entity.true_anomaly_deg = read_number(*orbit, "true_anomaly_deg", entity.true_anomaly_deg);
  } else {
    // Flat fields are accepted too, which keeps hand-written snippets short.
    entity.periapsis_altitude = read_number(node, "periapsis_altitude", entity.periapsis_altitude);
    entity.apoapsis_altitude = read_number(node, "apoapsis_altitude", entity.apoapsis_altitude);
    entity.inclination_deg = read_number(node, "inclination_deg", entity.inclination_deg);
    entity.raan_deg = read_number(node, "raan_deg", entity.raan_deg);
    entity.argp_deg = read_number(node, "argp_deg", entity.argp_deg);
    entity.true_anomaly_deg = read_number(node, "true_anomaly_deg", entity.true_anomaly_deg);
  }

  if (entity.apoapsis_altitude < entity.periapsis_altitude) {
    fail("entity '" + entity.name + "' has an apoapsis below its periapsis");
  }
  return entity;
}

}  // namespace

core::Scenario parse_scenario(std::string_view json_text) {
  json document;
  try {
    document = json::parse(json_text);
  } catch (const json::parse_error& error) {
    fail(std::string("malformed JSON: ") + error.what());
  }
  if (!document.is_object()) {
    fail("the document must be an object");
  }

  core::Scenario scenario;
  scenario.name = read_string(document, "name", scenario.name);

  if (document.contains("epoch_tdb")) {
    scenario.epoch = core::Instant{read_number(document, "epoch_tdb", 0.0)};
  } else if (document.contains("epoch")) {
    scenario.epoch = parse_iso_epoch(read_string(document, "epoch", "2000-01-01T12:00:00"));
  }

  const auto entities = document.find("entities");
  if (entities != document.end()) {
    if (!entities->is_array()) {
      fail("'entities' must be an array");
    }
    for (const json& node : *entities) {
      scenario.entities.push_back(parse_entity(node));
    }
  }
  return scenario;
}

core::Scenario load_scenario_file(const std::filesystem::path& path) {
  std::ifstream stream(path);
  if (!stream) {
    fail("cannot open '" + path.string() + "'");
  }
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  try {
    return parse_scenario(buffer.str());
  } catch (const std::runtime_error& error) {
    fail(std::string(error.what()) + " (in " + path.string() + ")");
  }
}

std::string write_scenario(const core::Scenario& scenario) {
  json document;
  document["name"] = scenario.name;
  document["epoch_tdb"] = scenario.epoch.tdb;

  json entities = json::array();
  for (const core::ScenarioEntity& entity : scenario.entities) {
    json node;
    node["name"] = entity.name;
    node["parent"] = entity.parent_body;
    node["orbit"]["periapsis_altitude"] = entity.periapsis_altitude;
    node["orbit"]["apoapsis_altitude"] = entity.apoapsis_altitude;
    node["orbit"]["inclination_deg"] = entity.inclination_deg;
    node["orbit"]["raan_deg"] = entity.raan_deg;
    node["orbit"]["argp_deg"] = entity.argp_deg;
    node["orbit"]["true_anomaly_deg"] = entity.true_anomaly_deg;
    if (entity.parts.empty()) {
      // A point mass carries its own figures. A vessel built from parts does
      // not: writing them too would put two answers in the file, and the one a
      // person edited would be the one that got ignored.
      node["mass"] = entity.mass;
      node["radius"] = entity.radius;
    } else {
      json parts = json::array();
      for (const core::StackPart& part : entity.parts) {
        json entry;
        entry["part"] = part.part;
        entry["stage"] = part.stage;
        parts.push_back(std::move(entry));
      }
      node["parts"] = std::move(parts);
    }
    node["controllable"] = entity.controllable;
    entities.push_back(std::move(node));
  }
  document["entities"] = std::move(entities);
  return document.dump(2) + "\n";
}

core::Scenario default_scenario() {
  core::Scenario scenario;
  scenario.name = "default";
  scenario.epoch = core::Instant{};

  core::ScenarioEntity station;
  station.name = "Station";
  station.parent_body = "Earth";
  station.periapsis_altitude = 400e3;
  station.apoapsis_altitude = 400e3;
  station.inclination_deg = 51.6;
  station.mass = 420000.0;
  station.radius = 60.0;
  scenario.entities.push_back(station);

  core::ScenarioEntity probe;
  probe.name = "Probe";
  probe.parent_body = "Earth";
  probe.periapsis_altitude = 250e3;
  probe.apoapsis_altitude = 35786e3;
  probe.inclination_deg = 28.5;
  probe.argp_deg = 180.0;
  probe.mass = 800.0;
  probe.radius = 3.0;
  scenario.entities.push_back(probe);

  return scenario;
}

}  // namespace rocketlab::scenario
