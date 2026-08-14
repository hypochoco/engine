#include "harness/harness.h"
//
//  bounds.cpp
//  engine::tst — core / unit
//
//  Tests core::Aabb (transform) + core::Frustum (extraction from a view-proj + AABB/sphere tests).
//

#include <cstdio>

#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "engine/core/math/bounds.h"

using namespace engine;

TST_CASE(core, unit, aabb_transform) {
    core::Aabb b{ glm::vec3(-1), glm::vec3(1) };
    TST_REQUIRE(!b.empty());
    TST_APPROX(b.center().x, 0.0f, 1e-6);
    TST_APPROX(b.extents().y, 1.0f, 1e-6);

    // Translate far along +X: the box moves with it.
    core::Aabb t = b.transformed(glm::translate(glm::mat4(1.0f), glm::vec3(100, 0, 0)));
    TST_APPROX(t.center().x, 100.0f, 1e-4);
    TST_APPROX(t.extents().x, 1.0f, 1e-4);

    // Uniform scale grows the extents.
    core::Aabb s = b.transformed(glm::scale(glm::mat4(1.0f), glm::vec3(3.0f)));
    TST_APPROX(s.extents().x, 3.0f, 1e-4);
}

TST_CASE(core, unit, frustum_cull) {
    // Camera at (0,0,5) looking toward -Z; fov 60, aspect 1, near 0.1, far 100.
    const glm::mat4 view = glm::lookAt(glm::vec3(0, 0, 5), glm::vec3(0), glm::vec3(0, 1, 0));
    const glm::mat4 proj = glm::perspective(glm::radians(60.0f), 1.0f, 0.1f, 100.0f);
    const core::Frustum f = core::Frustum::fromViewProj(proj * view);

    auto box = [](glm::vec3 c) { return core::Aabb{ c - glm::vec3(0.5f), c + glm::vec3(0.5f) }; };

    // In front of the camera near the origin → visible.
    TST_REQUIRE_MSG(f.intersects(box(glm::vec3(0, 0, 0))), "origin box should be inside");
    TST_REQUIRE_MSG(f.intersects(glm::vec3(0, 0, 0), 0.5f), "origin sphere should be inside");
    // Behind the camera (+Z beyond the eye) → culled.
    TST_REQUIRE_MSG(!f.intersects(box(glm::vec3(0, 0, 10))), "box behind the camera should be culled");
    TST_REQUIRE_MSG(!f.intersects(glm::vec3(0, 0, 10), 0.5f), "sphere behind the camera should be culled");
    // Far off to the side → culled.
    TST_REQUIRE_MSG(!f.intersects(box(glm::vec3(100, 0, 0))), "box far to the side should be culled");
    // Beyond the far plane (distance from eye ~205 > 100) → culled.
    TST_REQUIRE_MSG(!f.intersects(box(glm::vec3(0, 0, -200))), "box beyond far plane should be culled");

    std::printf("bounds/frustum ok\n");
}
