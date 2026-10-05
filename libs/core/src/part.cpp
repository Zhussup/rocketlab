#include "rocketlab/core/part.hpp"

#include <algorithm>
#include <utility>

namespace rocketlab::core {

const char* to_string(PartKind kind) noexcept {
  switch (kind) {
    case PartKind::CommandPod:
      return "pod";
    case PartKind::Tank:
      return "tank";
    case PartKind::Engine:
      return "engine";
    case PartKind::Decoupler:
      return "decoupler";
    case PartKind::Payload:
      return "payload";
    case PartKind::Structure:
      return "structure";
  }
  return "unknown";
}

PartCatalogue PartCatalogue::stock() {
  PartCatalogue catalogue;

  const auto add = [&catalogue](Part part) {
    catalogue.parts_.push_back(std::move(part));
  };

  // Capsules and probes. Their mass is the crew compartment and the avionics,
  // and `torque` is the reaction wheels a flight computer will steer with.
  add(Part{"pod.mk1", PartKind::CommandPod, 840.0, 0.0, 0.0, 0.0, 1.25, 1.20, 1500.0});
  add(Part{"probe.core", PartKind::CommandPod, 90.0, 0.0, 0.0, 0.0, 0.60, 0.30, 60.0});

  // Tanks. Dry mass runs at roughly an eighth of the propellant they hold,
  // which is where real structures land and is what makes the stage table on a
  // scenario like `leo.json` read plausibly.
  add(Part{"tank.fl100", PartKind::Tank, 500.0, 4000.0, 0.0, 0.0, 1.25, 1.50, 0.0});
  add(Part{"tank.fl400", PartKind::Tank, 4000.0, 32000.0, 0.0, 0.0, 2.50, 3.00, 0.0});
  add(Part{"tank.fl800", PartKind::Tank, 8000.0, 72000.0, 0.0, 0.0, 2.50, 6.00, 0.0});

  // Engines, by size. Thrust and Isp are vacuum figures at full throttle; a
  // throttled engine scales the first and leaves the second alone, which is
  // what makes a throttled burn cost the same propellant for the same impulse.
  add(Part{"engine.ant", PartKind::Engine, 40.0, 0.0, 2000.0, 315.0, 0.60, 0.30, 0.0});
  add(Part{"engine.terrier", PartKind::Engine, 500.0, 0.0, 60000.0, 345.0, 1.25, 1.50, 0.0});
  add(Part{"engine.skipper", PartKind::Engine, 3000.0, 0.0, 650000.0, 320.0, 2.50, 3.00, 0.0});
  add(Part{"engine.mainsail", PartKind::Engine, 6000.0, 0.0, 1500000.0, 285.0, 2.50, 3.00, 0.0});

  add(Part{"decoupler.stack", PartKind::Decoupler, 50.0, 0.0, 0.0, 0.0, 1.25, 0.30, 0.0});
  add(Part{"decoupler.large", PartKind::Decoupler, 200.0, 0.0, 0.0, 0.0, 2.50, 0.40, 0.0});

  add(Part{"lab.science", PartKind::Payload, 400.0, 0.0, 0.0, 0.0, 1.25, 1.00, 0.0});
  add(Part{"cargo.supply", PartKind::Payload, 250.0, 0.0, 0.0, 0.0, 1.25, 1.20, 0.0});
  add(Part{"antenna.hg", PartKind::Structure, 30.0, 0.0, 0.0, 0.0, 0.60, 0.50, 0.0});
  add(Part{"fins.basic", PartKind::Structure, 80.0, 0.0, 0.0, 0.0, 1.25, 0.40, 0.0});

  return catalogue;
}

const Part* PartCatalogue::find(std::string_view name) const noexcept {
  const auto it = std::find_if(parts_.begin(), parts_.end(),
                               [name](const Part& part) { return part.name == name; });
  return it == parts_.end() ? nullptr : &*it;
}

}  // namespace rocketlab::core
