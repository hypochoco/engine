//
//  intersection.cpp
//  engine::tst / graphics / path-tracer / unit
//
//  Unit tests for ray-triangle intersection (Möller–Trumbore) and the Scene nearest-hit /
//  shadow-ray queries: hit/miss/parallel/behind cases + correct t and barycentrics, nearest-of-many
//  selection, and occlusion.
//

#include <cstdio>

#include <glm/glm.hpp>

#include "engine/pathtracer/sampling.h"
#include "engine/pathtracer/scene.h"
#include "harness/harness.h"

using namespace engine::pt;

namespace {
Triangle tri(glm::vec3 a, glm::vec3 b, glm::vec3 c, uint32_t mat = 0) {
    Triangle t; t.v0 = a; t.v1 = b; t.v2 = c;
    t.n0 = t.n1 = t.n2 = t.geoNormal(); t.material = mat;
    return t;
}
}

TST_CASE(pathtracer, unit, ray_triangle) {
    const glm::vec3 v0(0, 0, 0), v1(1, 0, 0), v2(0, 1, 0);   // z = 0 plane
    float t, u, v;

    // Hit: straight down at (0.25, 0.25) from z=1 → t=1, barycentrics u=v=0.25.
    TST_REQUIRE_MSG(intersectTriangle(v0, v1, v2, {0.25f, 0.25f, 1.0f}, {0, 0, -1}, 1e-4f, t, u, v), "should hit");
    TST_APPROX(t, 1.0f, 1e-4);
    TST_APPROX(u, 0.25f, 1e-4);
    TST_APPROX(v, 0.25f, 1e-4);

    // Miss: outside the triangle (x+y > 1).
    TST_REQUIRE_MSG(!intersectTriangle(v0, v1, v2, {0.9f, 0.9f, 1.0f}, {0, 0, -1}, 1e-4f, t, u, v), "should miss (outside)");

    // Parallel: ray direction lies in the triangle's plane.
    TST_REQUIRE_MSG(!intersectTriangle(v0, v1, v2, {0.25f, 0.25f, 1.0f}, {1, 0, 0}, 1e-4f, t, u, v), "parallel ray misses");

    // Behind: triangle is behind the ray origin (pointing away).
    TST_REQUIRE_MSG(!intersectTriangle(v0, v1, v2, {0.25f, 0.25f, 1.0f}, {0, 0, 1}, 1e-4f, t, u, v), "behind-origin misses");

    // tMin cull: a hit closer than tMin is rejected.
    TST_REQUIRE_MSG(!intersectTriangle(v0, v1, v2, {0.25f, 0.25f, 0.05f}, {0, 0, -1}, 0.1f, t, u, v), "hit within tMin is culled");
}

TST_CASE(pathtracer, unit, scene_nearest_and_occlusion) {
    Scene s;
    s.addMaterial({});                                   // material 0
    // Two parallel quads facing +z: near at z=0, far at z=-2.
    s.triangles.push_back(tri({-1,-1, 0}, { 1,-1, 0}, { 1, 1, 0}));
    s.triangles.push_back(tri({-1,-1, 0}, { 1, 1, 0}, {-1, 1, 0}));
    const uint32_t nearCount = 2;
    s.triangles.push_back(tri({-1,-1,-2}, { 1,-1,-2}, { 1, 1,-2}));
    s.triangles.push_back(tri({-1,-1,-2}, { 1, 1,-2}, {-1, 1,-2}));
    s.finalize();

    // Ray from z=3 toward -z hits the NEAR quad (t=3), not the far one.
    const Hit h = s.intersect({0, 0, 3}, {0, 0, -1});
    TST_REQUIRE_MSG(h.valid, "ray should hit");
    TST_APPROX(h.t, 3.0f, 1e-3);
    TST_REQUIRE_MSG(h.tri < nearCount, "nearest hit must be a near-quad triangle");
    TST_APPROX(h.p.z, 0.0f, 1e-3);
    TST_REQUIRE_MSG(std::fabs(glm::dot(h.n, glm::vec3(0, 0, 1))) > 0.99f, "hit normal should be ±z");

    // Occlusion: the near quad blocks the segment from z=3 to the far quad at z=-2 (dist 5).
    TST_REQUIRE_MSG(s.occluded({0, 0, 3}, {0, 0, -1}, 5.0f), "near quad should occlude");
    // A ray that misses both quads is unoccluded.
    TST_REQUIRE_MSG(!s.occluded({5, 5, 3}, {0, 0, -1}, 10.0f), "off-axis ray is unoccluded");
    // Clear line-of-sight shorter than the first hit is unoccluded.
    TST_REQUIRE_MSG(!s.occluded({0, 0, 3}, {0, 0, -1}, 2.0f), "segment ending before the quad is unoccluded");

    std::printf("scene intersect/occlusion ok (near t=%.2f tri=%u)\n", h.t, h.tri);
}
