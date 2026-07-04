//
//  camera.h
//  engine::core / math
//
//  A render-agnostic camera *projection*. Pure math data — no render/RHI types — so it lives in
//  core alongside Transform and satisfies the ECS trivially-copyable rule. The camera's pose is
//  a separate `engine::Transform` on the same entity (the view matrix is the inverse of that
//  pose); this struct only describes how the scene is projected. `engine::scene::extractViews`
//  turns a <Transform, Camera> entity into a render::RenderView.
//

#pragma once

#include <cstdint>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace engine {

struct Camera {
    enum class Projection : uint8_t { Perspective, Orthographic };

    Projection projection = Projection::Perspective;
    float fovY        = glm::radians(60.0f);   // vertical field of view, radians (perspective)
    float nearZ       = 0.1f;
    float farZ        = 1000.0f;
    float orthoHeight = 10.0f;                  // vertical world extent (orthographic)

    // Projection matrix for a given viewport aspect (width / height). Depth range is 0..1
    // (GLM_FORCE_DEPTH_ZERO_TO_ONE is set engine-wide to match Metal/Vulkan clip space).
    glm::mat4 projectionMatrix(float aspect) const {
        if (projection == Projection::Perspective) {
            return glm::perspective(fovY, aspect, nearZ, farZ);
        }
        const float h = orthoHeight * 0.5f;
        const float w = h * aspect;
        return glm::ortho(-w, w, -h, h, nearZ, farZ);
    }
};

} // namespace engine
