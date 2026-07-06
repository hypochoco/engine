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
#include "engine/graphics/view/render_view.h"
#include "engine/graphics/rhi/types.h"
#include "engine/scene/render_components.h"

namespace engine::scene {

struct ExtractedScene {
    std::vector<render::InstanceData> instances;
    std::vector<render::RenderItem>   items;   // one per (mesh) batch; contiguous instance runs
};

// Queries <Transform, RenderMesh, RenderMaterial>, buckets instances by mesh, and fills `out`
// with one RenderItem per mesh + a contiguous InstanceData run. Deterministic order (mesh id).
// Pipeline-free: how the items are drawn (the mesh pipeline) is the consuming renderer's concern
// (see Renderer::setMeshPipeline), not part of the extracted scene.
void extract(ecs::World& world, ExtractedScene& out);

// Builds one render::RenderView per camera entity (<Transform, engine::Camera>): the view
// matrix is the inverse of the camera's pose, the projection comes from the Camera (aspect =
// width/height), and Background/SceneLighting resources are applied (see environment.h). The
// items/instances spans point into `scene` (which must outlive the views); the caller fills in
// each view's `materials` and `target` (e.g. the per-frame swapchain image). No camera entity
// → no view.
void extractViews(ecs::World& world, const ExtractedScene& scene,
                  std::vector<render::RenderView>& outViews,
                  uint32_t width, uint32_t height);

} // namespace engine::scene
