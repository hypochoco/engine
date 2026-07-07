//
//  geometry_catalog.h
//  engine::core / geometry
//
//  The authoritative CPU-side geometry residency. Owns MeshData and hands out stable, backend-
//  agnostic MeshIds. This is the neutral home that render/pathtracer heads (and their ECS bridges,
//  and a future BVH builder) source geometry from — so geometry identity lives in `core`, not in
//  any renderer. See notes/investigations/path-tracing/2026-07-07-pathtracer-dependency-model.md.
//
//  Today `core::MeshData` was transient loader output uploaded straight into the realtime
//  GeometryStore; the catalog makes the CPU copy authoritative so more than one consumer (the path
//  tracer, later a BVH) can read it without reaching through the realtime renderer.
//

#pragma once

#include <cstddef>
#include <utility>
#include <vector>

#include "engine/core/geometry/mesh.h"
#include "engine/core/memory/handle.h"

namespace engine {

// Stable identity of a mesh within a GeometryCatalog (backend-agnostic).
struct MeshCatalogTag;
using MeshId = core::Handle<MeshCatalogTag>;

// Append-only CPU geometry store. Value-type owned by whoever assembles a scene; both renderer
// heads read `MeshData` from it by `MeshId`.
class GeometryCatalog {
public:
    // Adds a mesh (by move) and returns its stable id. Ids are never reused (append-only).
    MeshId add(MeshData mesh) {
        const uint32_t index = static_cast<uint32_t>(meshes_.size());
        meshes_.push_back(std::move(mesh));
        return MeshId{ index, 0 };
    }

    bool valid(MeshId id) const { return id.valid() && id.index < meshes_.size(); }

    const MeshData& mesh(MeshId id) const { return meshes_[id.index]; }

    std::size_t size() const { return meshes_.size(); }

private:
    std::vector<MeshData> meshes_;
};

} // namespace engine
