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

void extract(ecs::World& world, rhi::PipelineHandle pipeline, ExtractedScene& out) {
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
        item.pipeline = pipeline;
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

} // namespace engine::scene
