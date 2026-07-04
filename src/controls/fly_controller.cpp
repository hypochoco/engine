//
//  fly_controller.cpp
//  engine::controls
//

#include "engine/controls/fly_controller.h"

#include <cmath>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "engine/core/math/transform.h"
#include "engine/core/time.h"
#include "engine/ecs/ecs.h"
#include "engine/input/input.h"

namespace engine::controls {
namespace {

glm::vec3 forwardFromYawPitch(float yawDeg, float pitchDeg) {
    const float cy = std::cos(glm::radians(yawDeg)),   sy = std::sin(glm::radians(yawDeg));
    const float cp = std::cos(glm::radians(pitchDeg)), sp = std::sin(glm::radians(pitchDeg));
    return glm::normalize(glm::vec3(cy * cp, sp, sy * cp));
}

} // namespace

void flyControllerSystem(ecs::World& world) {
    const auto* in   = world.getResource<input::InputState>();
    const auto* time = world.getResource<engine::Time>();
    if (!in || !time) return;
    const float dt = time->dt;

    world.query<engine::Transform, FlyController>().each(
        [&](ecs::Entity, engine::Transform& t, FlyController& fc) {
            // Mouse-look while the right button is held.
            if (in->mouseDown(input::MouseButton::Right)) {
                const glm::vec2 d = in->mouseDelta();
                fc.yaw   += d.x * fc.lookSensitivity;
                fc.pitch -= d.y * fc.lookSensitivity;
                fc.pitch  = glm::clamp(fc.pitch, -89.0f, 89.0f);
            }

            const glm::vec3 up(0.0f, 1.0f, 0.0f);
            const glm::vec3 f = forwardFromYawPitch(fc.yaw, fc.pitch);
            const glm::vec3 r = glm::normalize(glm::cross(f, up));
            const glm::vec3 u = glm::cross(r, f);

            float speed = fc.moveSpeed * dt;
            if (in->keyDown(input::Key::LeftShift)) speed *= fc.sprintMultiplier;

            glm::vec3 p = t.position;
            if (in->keyDown(input::Key::W)) p += f * speed;
            if (in->keyDown(input::Key::S)) p -= f * speed;
            if (in->keyDown(input::Key::D)) p += r * speed;
            if (in->keyDown(input::Key::A)) p -= r * speed;
            if (in->keyDown(input::Key::E) || in->keyDown(input::Key::Space))       p += u * speed;
            if (in->keyDown(input::Key::Q) || in->keyDown(input::Key::LeftControl)) p -= u * speed;
            t.position = p;

            // Orientation whose local -Z looks along `f` (camera convention): the rotation
            // matrix columns are the camera axes (right, up, -forward) in world space, so
            // inverse(pose) == lookAt(pos, pos+f, up).
            t.rotation = glm::quat_cast(glm::mat3(r, u, -f));
        });
}

} // namespace engine::controls
