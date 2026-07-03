//
//  types.h
//  engine::physics
//
//  Shared scalar + math typedefs. `Real` is localized here (§14.3 of the physics plan) so a
//  future switch to double or a dual/adjoint scalar for differentiable physics is a contained
//  change rather than a codebase-wide rewrite.
//

#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace engine::physics {

using Real = float;

using Vec3 = glm::vec3;
using Mat3 = glm::mat3;
using Quat = glm::quat;

inline constexpr Real kEpsilon = Real(1e-6);

} // namespace engine::physics
