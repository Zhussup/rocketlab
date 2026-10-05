#include "rocketlab/core/docking.hpp"

#include <cmath>

namespace rocketlab::core {

const char* to_string(DockBlock block) noexcept {
  switch (block) {
    case DockBlock::None:
      return "none";
    case DockBlock::SameEntity:
      return "an entity cannot dock with itself";
    case DockBlock::NoParts:
      return "both vessels must be built from parts";
    case DockBlock::DifferentParent:
      return "the two vessels are around different bodies";
    case DockBlock::TooFar:
      return "too far apart";
    case DockBlock::TooFast:
      return "closing too fast";
  }
  return "unknown";
}

DockingGeometry docking_geometry(const StateVector& target, const StateVector& absorbed,
                                 double target_radius, double absorbed_radius) noexcept {
  DockingGeometry geometry;
  geometry.separation = norm(target.r - absorbed.r);
  geometry.combined_radius = target_radius + absorbed_radius;
  geometry.closing_speed = norm(target.v - absorbed.v);
  return geometry;
}

DockingGeometry docking_geometry(const Entity& target, const Entity& absorbed) noexcept {
  return docking_geometry(target.state, absorbed.state, target.radius, absorbed.radius);
}

DockBlock dock_block(const DockingGeometry& geometry, const DockingLimits& limits) noexcept {
  // A gap, not a centre distance: two large vessels whose centres are a hundred
  // metres apart are touching, and two small ones are not.
  const double gap = geometry.separation - geometry.combined_radius;

  // A NaN separation is a state that has gone wrong; refusing is the only
  // answer that does not propagate the NaN into a merge. `!(a <= b)` rather
  // than `a > b` so that a NaN takes this branch.
  if (!(gap <= limits.surface_clearance)) {
    return DockBlock::TooFar;
  }
  if (!(geometry.closing_speed <= limits.max_closing_speed)) {
    return DockBlock::TooFast;
  }
  return DockBlock::None;
}

bool can_dock(const DockingGeometry& geometry, const DockingLimits& limits) noexcept {
  return dock_block(geometry, limits) == DockBlock::None;
}

DockBlock dock_block(const Entity& target, const Entity& absorbed,
                     const DockingLimits& limits) noexcept {
  if (target.id == absorbed.id) {
    return DockBlock::SameEntity;
  }
  if (target.vessel.empty() || absorbed.vessel.empty()) {
    return DockBlock::NoParts;
  }
  if (target.parent != absorbed.parent) {
    return DockBlock::DifferentParent;
  }
  return dock_block(docking_geometry(target, absorbed), limits);
}

}  // namespace rocketlab::core
