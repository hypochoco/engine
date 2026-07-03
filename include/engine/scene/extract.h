//
//  extract.h
//  engine::scene
//
//  The render-extraction system: turns ECS world state into render lists (RenderItem[] +
//  InstanceData[]) grouped by mesh, ready to drop into a render::RenderView. This is the
//  ECS-facing bridge the RHI plan describes — the Renderer consumes the result, unaware of
//  the ECS.
//

#pragma once

#include <vector>

#include "engine/ecs/world.h"
#include "engine/graphics/render/render_view.h"
#include "engine/graphics/rhi/types.h"
#include "engine/scene/render_components.h"

namespace engine::scene {

struct ExtractedScene {
    std::vector<render::InstanceData> instances;
    std::vector<render::RenderItem>   items;   // one per (mesh) batch; contiguous instance runs
};

// Queries <Transform, RenderMesh, RenderMaterial>, buckets instances by mesh, and fills `out`
// with one RenderItem per mesh + a contiguous InstanceData run. `pipeline` is applied to all
// items (single-pipeline scene for now). Deterministic order (mesh id).
void extract(ecs::World& world, rhi::PipelineHandle pipeline, ExtractedScene& out);

} // namespace engine::scene
