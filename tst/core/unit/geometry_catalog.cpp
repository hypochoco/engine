//
//  geometry_catalog.cpp
//  engine::tst / core / unit
//
//  The core CPU geometry residency: ids are stable + distinct, lookups return the stored data, and
//  invalid ids are rejected.
//

#include <cstdio>

#include "engine/core/geometry/geometry_catalog.h"
#include "engine/core/geometry/primitives.h"
#include "harness/harness.h"

using namespace engine;

TST_CASE(core, unit, geometry_catalog) {
    GeometryCatalog cat;
    TST_REQUIRE(cat.size() == 0);
    TST_REQUIRE_MSG(!cat.valid(MeshId{}), "default (invalid) MeshId must be rejected");

    MeshData box = primitives::makeBox(glm::vec3(0.5f));
    MeshData sphere = primitives::makeSphere(0.5f, 8, 12);
    const size_t boxVerts = box.vertices.size();
    const size_t sphereIdx = sphere.indices.size();

    const MeshId a = cat.add(std::move(box));
    const MeshId b = cat.add(std::move(sphere));

    // Ids are valid, distinct, and the catalog grew.
    TST_REQUIRE(cat.valid(a) && cat.valid(b));
    TST_REQUIRE_MSG(!(a == b), "distinct meshes get distinct ids");
    TST_REQUIRE(cat.size() == 2);

    // Lookups return the stored geometry (by id, order-stable).
    TST_REQUIRE_MSG(cat.mesh(a).vertices.size() == boxVerts, "mesh(a) returns the box data");
    TST_REQUIRE_MSG(cat.mesh(b).indices.size() == sphereIdx, "mesh(b) returns the sphere data");

    // Out-of-range id is invalid.
    TST_REQUIRE_MSG(!cat.valid(MeshId{ 99u, 0u }), "out-of-range id is invalid");

    std::printf("geometry_catalog ok: %zu meshes (box verts=%zu, sphere idx=%zu)\n",
                cat.size(), cat.mesh(a).vertices.size(), cat.mesh(b).indices.size());
}
