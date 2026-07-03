//
//  test.cpp
//  engine::tst
//
//  Driver test for engine::core geometry. Builds meshes via the primitive generators
//  and checks their vertex/index counts. Links engine::core only (no graphics backend).
//

#include "engine/core/core.h"

#include "harness/harness.h"
#include <cstdint>
#include <iostream>

TST_CASE(core, unit, geometry) {
    using namespace engine;

    // Plane: (subdivisions+1)^2 vertices, subdivisions^2 * 6 indices.
    const MeshData plane = primitives::makePlane(10.0f, 1);
    std::cout << "plane:  " << plane.vertices.size() << " verts, "
              << plane.indices.size() << " indices\n";
    TST_REQUIRE(plane.vertices.size() == 4);
    TST_REQUIRE(plane.indices.size() == 6);

    // Sphere: (rings+1)*(sectors+1) vertices, rings*sectors*6 indices.
    const uint32_t rings = 16, sectors = 32;
    const MeshData sphere = primitives::makeSphere(0.5f, rings, sectors);
    std::cout << "sphere: " << sphere.vertices.size() << " verts, "
              << sphere.indices.size() << " indices\n";
    TST_REQUIRE(sphere.vertices.size() == static_cast<size_t>(rings + 1) * (sectors + 1));
    TST_REQUIRE(sphere.indices.size() == static_cast<size_t>(rings) * sectors * 6);

    // Sanity: sphere positions sit on the radius (smooth normals point outward).
    for (const auto& v : sphere.vertices) {
        const float len = glm::length(v.position);
        TST_REQUIRE(len > 0.49f && len < 0.51f);
        (void)len;
    }

    // Quad + minimal model wiring compile & behave.
    const MeshData quad = primitives::makeQuad();
    TST_REQUIRE(quad.indices.size() == 6);

    ModelData model;
    model.meshes.push_back(sphere);
    model.materials.push_back(Material{});
    model.meshMaterial.push_back(0);
    TST_REQUIRE(model.meshes.size() == model.meshMaterial.size());

    std::cout << "core geometry ok\n";
}
