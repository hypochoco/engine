//
//  obj_loader.h
//  engine::core / geometry
//
//  Wavefront .obj mesh loader (via tinyobjloader) → core::ModelData, plus tangent-frame
//  computation for normal mapping. Declarations are dependency-free; the implementation is
//  compiled only in a full engine build (ENGINE_ASSET_LOADERS) — a headless training build
//  returns an empty ModelData rather than linking a mesh decoder.
//

#pragma once

#include <string_view>

#include "engine/core/geometry/mesh.h"
#include "engine/core/geometry/model.h"

namespace engine::geometry {

// Loads a Wavefront .obj (triangulated) into a ModelData: one submesh per material (faces are
// grouped by material id; faces with no material go into a default material). Vertex positions,
// normals (generated flat if absent), and UVs are filled from the file, and a tangent frame is
// computed per submesh (see computeTangents). Materials carry baseColorFactor from the .mtl
// diffuse (Kd); texture binding is layered on separately. Returns an empty ModelData on failure.
//
// `flipV` flips the V texture coordinate (obj/GL bottom-up → top-left origin), matching how
// images are uploaded; on by default.
ModelData loadObj(std::string_view path, bool flipV = true);

// Computes a per-vertex tangent frame from positions, UVs, and normals: tangent.xyz is aligned
// with +U and orthonormalized against the normal; tangent.w is the ±1 handedness so the
// bitangent is w * cross(normal, tangent). Vertices with degenerate/missing UVs keep w = 0
// ("no tangent frame"). Operates in place; the mesh must be an indexed triangle list.
void computeTangents(MeshData& mesh);

} // namespace engine::geometry
