//
//  obj_loader.cpp
//  engine::core / geometry
//
//  tinyobjloader-backed .obj loader + tangent computation. Does NOT define
//  TINYOBJLOADER_IMPLEMENTATION — the external tinyobjloader target already compiles it; here we
//  only include the header and link the library. Gated on ENGINE_ASSET_LOADERS so a headless
//  training build (no tinyobj) still compiles (loadObj returns an empty ModelData there).
//

#include "engine/core/geometry/obj_loader.h"

#if defined(ENGINE_ASSET_LOADERS)

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

#include <glm/glm.hpp>
#include <tiny_obj_loader.h>

namespace engine::geometry {

namespace {

// Uniqueness key for an OBJ face-corner: the triple of tinyobj indices. Two corners with the same
// (position, normal, texcoord) indices share a vertex.
struct Corner {
    int v = -1, n = -1, t = -1;
    bool operator==(const Corner& o) const { return v == o.v && n == o.n && t == o.t; }
};
struct CornerHash {
    std::size_t operator()(const Corner& c) const {
        std::size_t h = static_cast<std::size_t>(c.v) * 73856093u;
        h ^= static_cast<std::size_t>(c.n) * 19349663u + 0x9e3779b9u + (h << 6) + (h >> 2);
        h ^= static_cast<std::size_t>(c.t) * 83492791u + 0x9e3779b9u + (h << 6) + (h >> 2);
        return h;
    }
};

} // namespace

void computeTangents(MeshData& mesh) {
    const std::size_t n = mesh.vertices.size();
    if (n == 0 || mesh.indices.size() < 3) return;

    std::vector<glm::vec3> tan(n, glm::vec3(0.0f));
    std::vector<glm::vec3> bitan(n, glm::vec3(0.0f));

    for (std::size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        const uint32_t i0 = mesh.indices[i], i1 = mesh.indices[i + 1], i2 = mesh.indices[i + 2];
        const Vertex& a = mesh.vertices[i0];
        const Vertex& b = mesh.vertices[i1];
        const Vertex& c = mesh.vertices[i2];

        const glm::vec3 e1 = b.position - a.position;
        const glm::vec3 e2 = c.position - a.position;
        const glm::vec2 d1 = b.uv - a.uv;
        const glm::vec2 d2 = c.uv - a.uv;

        const float denom = d1.x * d2.y - d2.x * d1.y;
        if (std::abs(denom) < 1e-12f) continue;   // degenerate UVs — leave tangent unset (w=0)
        const float r = 1.0f / denom;
        const glm::vec3 t = (e1 * d2.y - e2 * d1.y) * r;
        const glm::vec3 bt = (e2 * d1.x - e1 * d2.x) * r;

        tan[i0] += t; tan[i1] += t; tan[i2] += t;
        bitan[i0] += bt; bitan[i1] += bt; bitan[i2] += bt;
    }

    for (std::size_t i = 0; i < n; ++i) {
        const glm::vec3 nrm = mesh.vertices[i].normal;
        const glm::vec3 t = tan[i];
        if (glm::dot(t, t) < 1e-16f) { mesh.vertices[i].tangent = glm::vec4(0.0f); continue; }
        // Gram-Schmidt orthonormalize against the normal.
        glm::vec3 ot = t - nrm * glm::dot(nrm, t);
        const float len2 = glm::dot(ot, ot);
        if (len2 < 1e-16f) { mesh.vertices[i].tangent = glm::vec4(0.0f); continue; }
        ot *= 1.0f / std::sqrt(len2);
        const float w = (glm::dot(glm::cross(nrm, ot), bitan[i]) < 0.0f) ? -1.0f : 1.0f;
        mesh.vertices[i].tangent = glm::vec4(ot, w);
    }
}

ModelData loadObj(std::string_view path, bool flipV) {
    ModelData model;

    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> objMaterials;
    std::string warn, err;

    const std::string pathStr(path);
    const std::string baseDir = std::filesystem::path(pathStr).parent_path().string();
    const bool ok = tinyobj::LoadObj(&attrib, &shapes, &objMaterials, &warn, &err,
                                     pathStr.c_str(),
                                     baseDir.empty() ? nullptr : (baseDir + "/").c_str(),
                                     /*triangulate=*/true);
    if (!ok) return {};

    // Materials: index 0..M-1 map to the file's materials; a trailing "default" material catches
    // faces with material id -1.
    model.materials.reserve(objMaterials.size() + 1);
    for (const auto& m : objMaterials) {
        Material mat;
        mat.baseColorFactor = glm::vec4(m.diffuse[0], m.diffuse[1], m.diffuse[2], m.dissolve);
        mat.emissiveFactor  = glm::vec3(m.emission[0], m.emission[1], m.emission[2]);
        // tinyobj defaults roughness/metallic to 0 when the .mtl has no PBR extension; only adopt
        // them when specified, otherwise keep the plain-Lambert default (roughnessFactor = 1).
        if (m.roughness > 0.0f || m.metallic > 0.0f) {
            mat.roughnessFactor = m.roughness;
            mat.metallicFactor  = m.metallic;
        }
        // Texture references (m.diffuse_texname etc.) are resolved to bindless slots by the caller;
        // the loader only fills factors (it does not own a texture cache).
        model.materials.push_back(mat);
    }
    const int defaultMatIndex = static_cast<int>(model.materials.size());   // may be unused
    bool usedDefault = false;

    // One MeshData per material index actually referenced. `buckets[matIdx]` accumulates geometry;
    // `dedup[matIdx]` maps a face-corner to its vertex index within that bucket.
    std::unordered_map<int, MeshData> buckets;
    std::unordered_map<int, std::unordered_map<Corner, uint32_t, CornerHash>> dedup;

    for (const auto& shape : shapes) {
        const auto& m = shape.mesh;
        std::size_t indexOffset = 0;
        for (std::size_t f = 0; f < m.num_face_vertices.size(); ++f) {
            const int fv = m.num_face_vertices[f];   // 3 (triangulated)
            int matId = m.material_ids.empty() ? -1 : m.material_ids[f];
            if (matId < 0 || matId >= static_cast<int>(objMaterials.size())) {
                matId = defaultMatIndex;
                usedDefault = true;
            }

            MeshData& bucket = buckets[matId];
            auto& cornerMap = dedup[matId];

            for (int vi = 0; vi < fv; ++vi) {
                const tinyobj::index_t idx = m.indices[indexOffset + vi];
                const Corner key{ idx.vertex_index, idx.normal_index, idx.texcoord_index };

                auto it = cornerMap.find(key);
                if (it != cornerMap.end()) {
                    bucket.indices.push_back(it->second);
                    continue;
                }

                Vertex vert;
                vert.position = { attrib.vertices[3 * idx.vertex_index + 0],
                                  attrib.vertices[3 * idx.vertex_index + 1],
                                  attrib.vertices[3 * idx.vertex_index + 2] };
                if (idx.normal_index >= 0) {
                    vert.normal = { attrib.normals[3 * idx.normal_index + 0],
                                    attrib.normals[3 * idx.normal_index + 1],
                                    attrib.normals[3 * idx.normal_index + 2] };
                }
                if (idx.texcoord_index >= 0) {
                    const float u = attrib.texcoords[2 * idx.texcoord_index + 0];
                    const float v = attrib.texcoords[2 * idx.texcoord_index + 1];
                    vert.uv = { u, flipV ? (1.0f - v) : v };
                }
                vert.color = { 1.0f, 1.0f, 1.0f };

                const uint32_t newIndex = static_cast<uint32_t>(bucket.vertices.size());
                bucket.vertices.push_back(vert);
                bucket.indices.push_back(newIndex);
                cornerMap.emplace(key, newIndex);
            }
            indexOffset += static_cast<std::size_t>(fv);
        }
    }

    if (usedDefault && defaultMatIndex == static_cast<int>(model.materials.size())) {
        model.materials.push_back(Material{});   // white default
    }

    // Flatten buckets into ModelData (deterministic order by material index).
    std::vector<int> matIds;
    matIds.reserve(buckets.size());
    for (auto& [mid, _] : buckets) matIds.push_back(mid);
    std::sort(matIds.begin(), matIds.end());

    // If any faces lacked normals, generate flat normals so tangents (and shading) are sane.
    auto ensureNormals = [](MeshData& mesh) {
        bool anyMissing = false;
        for (const auto& v : mesh.vertices)
            if (glm::dot(v.normal, v.normal) < 1e-12f) { anyMissing = true; break; }
        if (!anyMissing) return;
        for (auto& v : mesh.vertices) v.normal = glm::vec3(0.0f);
        for (std::size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
            const uint32_t a = mesh.indices[i], b = mesh.indices[i + 1], c = mesh.indices[i + 2];
            const glm::vec3 fn = glm::cross(mesh.vertices[b].position - mesh.vertices[a].position,
                                            mesh.vertices[c].position - mesh.vertices[a].position);
            mesh.vertices[a].normal += fn; mesh.vertices[b].normal += fn; mesh.vertices[c].normal += fn;
        }
        for (auto& v : mesh.vertices) {
            const float l2 = glm::dot(v.normal, v.normal);
            v.normal = (l2 > 1e-12f) ? v.normal * (1.0f / std::sqrt(l2)) : glm::vec3(0, 1, 0);
        }
    };

    for (int mid : matIds) {
        MeshData mesh = std::move(buckets[mid]);
        ensureNormals(mesh);
        computeTangents(mesh);
        model.meshes.push_back(std::move(mesh));
        model.meshMaterial.push_back(static_cast<uint32_t>(mid));
    }

    return model;
}

} // namespace engine::geometry

#else   // !ENGINE_ASSET_LOADERS — headless training build, no mesh decoder linked.

namespace engine::geometry {
ModelData loadObj(std::string_view, bool) { return {}; }
void computeTangents(MeshData&) {}
} // namespace engine::geometry

#endif
