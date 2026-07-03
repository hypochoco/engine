//
//  extract.cpp
//  engine::scene
//

#include "engine/scene/extract.h"

#include <map>
#include <vector>

#include "engine/core/math/transform.h"
#include "engine/ecs/query.h"

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

} // namespace engine::scene
