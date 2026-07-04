//
//  backends_internal.h
//  engine::physics (internal)
//
//  Per-backend factory makers, kept out of the public header. `createPhysicsWorld` (defined in the
//  realtime TU) dispatches on `Backend` to one of these. Each backend TU defines its own maker.
//

#pragma once

#include <memory>

#include "engine/physics/world.h"

namespace engine::physics {

// Reduced-coordinate Featherstone/ABA backend (Phase E). Defined in reduced/featherstone_world.cpp.
std::unique_ptr<PhysicsWorld> createFeatherstoneWorld(const WorldDef& def);

} // namespace engine::physics
