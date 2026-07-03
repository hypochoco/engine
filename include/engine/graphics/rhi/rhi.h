//
//  rhi.h
//  engine::graphics / rhi
//
//  Umbrella include for the backend-agnostic Render Hardware Interface.
//  Design: notes/investigations/2026-07-02-rhi-interface-plan.md
//

#pragma once

#include "engine/graphics/rhi/types.h"
#include "engine/graphics/rhi/resources.h"
#include "engine/graphics/rhi/pipeline.h"
#include "engine/graphics/rhi/command_list.h"
#include "engine/graphics/rhi/device.h"

namespace engine::rhi {

// Which backend this binary was compiled with (for logging/diagnostics).
const char* backendName();

} // namespace engine::rhi
