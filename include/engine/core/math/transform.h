//
//  transform.h
//  engine::core / math
//
//  A TRS transform: position, rotation (quaternion), scale, and its 4x4 matrix. Pure math
//  data — usable as an ECS component, a physics body pose, a camera, etc. Trivially copyable
//  so it satisfies the ECS's trivially-relocatable component requirement.
//

#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace engine {

struct Transform {
    glm::vec3 position{0.0f};
    glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};   // (w, x, y, z) identity
    glm::vec3 scale{1.0f};

    glm::mat4 matrix() const {
        glm::mat4 m = glm::translate(glm::mat4(1.0f), position);
        m = m * glm::mat4_cast(rotation);
        m = glm::scale(m, scale);
        return m;
    }
};

} // namespace engine
