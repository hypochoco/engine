//
//  extract.cpp
//  engine::scene
//

#include "engine/scene/extract.h"

#include <map>
#include <span>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "engine/core/math/camera.h"
#include "engine/core/math/transform.h"
#include "engine/ecs/query.h"
#include "engine/scene/environment.h"

namespace engine::scene {

void extract(ecs::World& world, ExtractedScene& out) {
    out.instances.clear();
    out.items.clear();

    // Bucket instances by mesh so each mesh becomes one contiguous instanced draw.
    // (ordered map → deterministic item order; a flat radix/sort is the scaling follow-up.)
    std::map<uint32_t, std::vector<render::InstanceData>> byMesh;
    std::map<uint32_t, render::MeshHandle>                meshOf;

    world.query<engine::Transform, RenderMesh, RenderMaterial>().each(
        [&](ecs::Entity, engine::Transform& t, RenderMesh& rm, RenderMaterial& mat) {
            render::InstanceData d;
            d.model = t.matrix();
            d.normalModel = d.model;   // TODO: transpose(inverse) for non-uniform scale
            d.materialIndex = mat.materialIndex;
            byMesh[rm.mesh.index].push_back(d);
            meshOf[rm.mesh.index] = rm.mesh;
        });

    for (auto& [meshIdx, insts] : byMesh) {
        render::RenderItem item;
        item.mesh = meshOf[meshIdx];
        item.firstInstance = static_cast<uint32_t>(out.instances.size());
        item.instanceCount = static_cast<uint32_t>(insts.size());
        out.items.push_back(item);
        out.instances.insert(out.instances.end(), insts.begin(), insts.end());
    }
}

void extractViews(ecs::World& world, const ExtractedScene& scene,
                  std::vector<render::RenderView>& outViews,
                  uint32_t width, uint32_t height) {
    outViews.clear();
    const float aspect = height ? static_cast<float>(width) / static_cast<float>(height) : 1.0f;

    world.query<engine::Transform, engine::Camera>().each(
        [&](ecs::Entity, engine::Transform& t, engine::Camera& cam) {
            render::RenderView v;
            // View = inverse of the rigid pose (position + rotation; scale ignored for a camera).
            const glm::mat4 pose = glm::translate(glm::mat4(1.0f), t.position) * glm::mat4_cast(t.rotation);
            v.view   = glm::inverse(pose);
            v.proj   = cam.projectionMatrix(aspect);
            v.width  = width;
            v.height = height;
            applyEnvironment(world, v);   // background + lighting from ECS resources
            v.items     = std::span<const render::RenderItem>(scene.items);
            v.instances = std::span<const render::InstanceData>(scene.instances);
            outViews.push_back(v);
        });
}

void cullToFrustum(const core::Frustum& frustum, const ExtractedScene& in,
                   std::span<const core::Aabb> localBoundsPerItem, ExtractedScene& out) {
    out.instances.clear();
    out.items.clear();

    // No bounds provided ⇒ don't cull (straight copy), so callers can enable culling per view.
    if (localBoundsPerItem.size() != in.items.size()) {
        out.instances = in.instances;
        out.items     = in.items;
        return;
    }

    for (std::size_t i = 0; i < in.items.size(); ++i) {
        const render::RenderItem& item = in.items[i];
        const core::Aabb& local = localBoundsPerItem[i];

        const uint32_t first = static_cast<uint32_t>(out.instances.size());
        for (uint32_t k = 0; k < item.instanceCount; ++k) {
            const render::InstanceData& inst = in.instances[item.firstInstance + k];
            if (frustum.intersects(local.transformed(inst.model)))
                out.instances.push_back(inst);
        }
        const uint32_t kept = static_cast<uint32_t>(out.instances.size()) - first;
        if (kept == 0) continue;   // whole batch culled → drop the item

        render::RenderItem culled = item;
        culled.firstInstance = first;
        culled.instanceCount = kept;
        out.items.push_back(culled);
    }
}

} // namespace engine::scene