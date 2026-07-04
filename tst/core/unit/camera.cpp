#include "harness/harness.h"
//
//  camera.cpp
//  engine::tst — core / unit
//
//  Tests engine::Camera projection matrices (perspective + orthographic), including the 0..1
//  depth convention (GLM_FORCE_DEPTH_ZERO_TO_ONE, set engine-wide).
//

#include <glm/glm.hpp>

#include "engine/core/math/camera.h"

using namespace engine;

TST_CASE(core, unit, camera_perspective) {
    Camera cam;   // perspective by default
    const glm::mat4 p = cam.projectionMatrix(16.0f / 9.0f);
    TST_REQUIRE(p != glm::mat4(1.0f));

    // A near-plane point at view-space (0,0,-nearZ) projects to clip w = nearZ.
    const glm::vec4 clip = p * glm::vec4(0.0f, 0.0f, -cam.nearZ, 1.0f);
    TST_APPROX(clip.w, cam.nearZ, 1e-4);
    // 0..1 depth: the near plane maps to z=0 (ndc z / w == 0).
    TST_APPROX(clip.z / clip.w, 0.0f, 1e-4);
}

TST_CASE(core, unit, camera_orthographic) {
    Camera cam;
    cam.projection = Camera::Projection::Orthographic;
    cam.orthoHeight = 10.0f;
    const glm::mat4 o = cam.projectionMatrix(1.0f);
    TST_REQUIRE(o != glm::mat4(1.0f));

    // With aspect 1 and height 10, the top edge (y=5) maps to ndc y = 1.
    const glm::vec4 top = o * glm::vec4(0.0f, 5.0f, -cam.nearZ, 1.0f);
    TST_APPROX(top.y, 1.0f, 1e-4);
}
