//
//  entity.h
//  engine::ecs
//
//  An entity is a generational handle (shared core::Handle). The World maps its index to a
//  location {archetype, row}; the generation invalidates stale handles after destroy.
//

#pragma once

#include "engine/core/memory/handle.h"

namespace engine::ecs {

struct EntityTag;
using Entity = engine::core::Handle<EntityTag>;

} // namespace engine::ecs
