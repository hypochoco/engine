#include "harness/harness.h"
//
//  obj_loader.cpp
//  engine::tst — core / unit
//
//  Verifies engine::geometry::loadObj (tinyobj → ModelData) + tangent computation. Writes a small
//  textured quad .obj (positions + UVs + a +Z normal) to a temp file, loads it, and checks: one
//  submesh, 6 indices, positions/UVs preserved, and a valid tangent frame (unit tangent aligned
//  with +U ≈ +X, orthogonal to the +Z normal, |w| == 1). Also checks computeTangents directly and
//  the degenerate-UV (no tangent) case.
//

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#include <glm/glm.hpp>

#include "engine/core/geometry/obj_loader.h"

using namespace engine;

TST_CASE(core, unit, obj_loader_quad) {
    // A unit quad in the z=0 plane, normal +Z, UVs mapping U→+X, V→+Y. Two triangles.
    const char* obj =
        "v -1 -1 0\n" "v 1 -1 0\n" "v 1 1 0\n" "v -1 1 0\n"
        "vt 0 0\n" "vt 1 0\n" "vt 1 1\n" "vt 0 1\n"
        "vn 0 0 1\n"
        "f 1/1/1 2/2/1 3/3/1\n"
        "f 1/1/1 3/3/1 4/4/1\n";

    const auto path = (std::filesystem::temp_directory_path() / "engine_quad_test.obj").string();
    { std::ofstream f(path); f << obj; }

    ModelData model = geometry::loadObj(path);
    TST_REQUIRE_MSG(model.meshes.size() == 1, "expected a single submesh");
    TST_REQUIRE_MSG(model.meshMaterial.size() == 1, "expected one mesh→material mapping");

    const MeshData& m = model.meshes[0];
    std::printf("obj_loader: verts=%zu indices=%zu materials=%zu\n",
                m.vertices.size(), m.indices.size(), model.materials.size());
    TST_REQUIRE_MSG(m.indices.size() == 6, "quad should triangulate to 6 indices");
    TST_REQUIRE_MSG(m.vertices.size() == 4, "quad corners should dedup to 4 unique vertices");

    // Every vertex: +Z normal, and a valid tangent frame aligned with +X (UV U axis).
    for (const Vertex& v : m.vertices) {
        TST_APPROX(v.normal.z, 1.0f, 1e-4);
        const glm::vec3 t = glm::vec3(v.tangent);
        TST_REQUIRE_MSG(std::abs(glm::length(t) - 1.0f) < 1e-3f, "tangent should be unit length");
        TST_REQUIRE_MSG(std::abs(glm::dot(t, v.normal)) < 1e-3f, "tangent should be orthogonal to normal");
        TST_REQUIRE_MSG(std::abs(t.x) > 0.99f && std::abs(t.y) < 1e-2f && std::abs(t.z) < 1e-2f,
                        "tangent should align with +U (world +X) for this quad");
        TST_REQUIRE_MSG(std::abs(std::abs(v.tangent.w) - 1.0f) < 1e-4f, "handedness w should be ±1");
    }

    // Degenerate UVs (all zero) → computeTangents leaves w = 0 (no tangent frame).
    MeshData degen;
    degen.vertices = { {}, {}, {} };
    for (auto& v : degen.vertices) { v.normal = {0, 0, 1}; v.uv = {0, 0}; }
    degen.vertices[0].position = {0, 0, 0};
    degen.vertices[1].position = {1, 0, 0};
    degen.vertices[2].position = {0, 1, 0};
    degen.indices = {0, 1, 2};
    geometry::computeTangents(degen);
    for (const Vertex& v : degen.vertices)
        TST_REQUIRE_MSG(v.tangent == glm::vec4(0.0f), "degenerate UVs should yield no tangent (w=0)");

    std::error_code ec;
    std::filesystem::remove(path, ec);
    std::printf("obj loader ok\n");
}
