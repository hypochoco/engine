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

#include <span>
#include <vector>

#include "engine/core/math/bounds.h"
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

// Frustum-culls an extracted scene: keeps only instances whose world-space AABB intersects the
// frustum, writing a compacted ExtractedScene (its own contiguous item + instance runs) into `out`.
// `localBoundsPerItem` is parallel to `in.items` — the LOCAL-space AABB of each item's mesh (see
// core::computeBounds); each instance's world AABB is that box transformed by InstanceData.model.
// Opt-in and lossless in the visible set: a RenderView pointed at `out` renders identically to one
// pointed at `in`, minus off-screen instances. Empty `localBoundsPerItem` ⇒ no culling (copy).
void cullToFrustum(const core::Frustum& frustum, const ExtractedScene& in,
                   std::span<const core::Aabb> localBoundsPerItem, ExtractedScene& out);

// Convenience: the world-view-projection frustum of a render view (proj * view).
inline core::Frustum viewFrustum(const render::RenderView& v) {
    return core::Frustum::fromViewProj(v.proj * v.view);
}

} // namespace engine::scene
