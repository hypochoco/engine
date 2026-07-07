//
//  sampling.h
//  engine::pathtracer
//
//  Pure, stateless path-tracing primitives: sampling routines (RNG-agnostic — they take uniforms
//  in [0,1) as arguments), Fresnel, an orthonormal basis, and Möller–Trumbore ray-triangle
//  intersection. Kept header-only + side-effect-free so they are (a) unit-testable in isolation and
//  (b) reusable by the coming GPU/BVH paths. The integrator composes these.
//

#pragma once

#include <cmath>

#include <glm/glm.hpp>

namespace engine::pt {

inline constexpr float PI_F = 3.14159265358979323846f;

// Orthonormal tangent/bitangent for a unit normal `n` (Duff-style branch, robust near the poles).
inline void orthonormalBasis(const glm::vec3& n, glm::vec3& t, glm::vec3& b) {
    t = (std::fabs(n.x) > std::fabs(n.z)) ? glm::normalize(glm::vec3(-n.y, n.x, 0.0f))
                                          : glm::normalize(glm::vec3(0.0f, -n.z, n.y));
    b = glm::cross(n, t);
}

// Cosine-weighted hemisphere direction around unit `n` from two uniforms. Sets pdf = cosθ/π.
inline glm::vec3 cosineSampleHemisphere(const glm::vec3& n, float u1, float u2, float& pdf) {
    const float phi  = 2.0f * PI_F * u1;
    const float cosT = std::sqrt(1.0f - u2);
    const float sinT = std::sqrt(u2);
    glm::vec3 t, b;
    orthonormalBasis(n, t, b);
    const glm::vec3 wi = std::cos(phi) * sinT * t + std::sin(phi) * sinT * b + cosT * n;
    pdf = cosT / PI_F;
    return glm::normalize(wi);
}

// Uniform point on triangle (v0,v1,v2) from two uniforms (√u1 barycentrics — unbiased).
inline glm::vec3 sampleTriangleUniform(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2,
                                       float u1, float u2) {
    const float su = std::sqrt(u1);
    return (1.0f - su) * v0 + (su * (1.0f - u2)) * v1 + (su * u2) * v2;
}

// Schlick approximation of the Fresnel reflectance (unpolarized). `cosI` = |cos(incidence)| ∈ [0,1].
inline float fresnelSchlick(float cosI, float etaI, float etaT) {
    float r0 = (etaI - etaT) / (etaI + etaT);
    r0 *= r0;
    const float c = 1.0f - glm::clamp(cosI, 0.0f, 1.0f);
    return r0 + (1.0f - r0) * (c * c * c * c * c);
}

// Möller–Trumbore ray/triangle test. On hit: `tOut` (> tMin) is the distance and (u,v) are the
// barycentrics of v1,v2 (so the point is (1-u-v)·v0 + u·v1 + v·v2). Culls no faces (two-sided).
inline bool intersectTriangle(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2,
                              const glm::vec3& o, const glm::vec3& d, float tMin,
                              float& tOut, float& u, float& v) {
    const glm::vec3 e1 = v1 - v0;
    const glm::vec3 e2 = v2 - v0;
    const glm::vec3 h  = glm::cross(d, e2);
    const float a = glm::dot(e1, h);
    if (std::fabs(a) < 1e-8f) return false;         // ray parallel to triangle plane
    const float f = 1.0f / a;
    const glm::vec3 s = o - v0;
    u = f * glm::dot(s, h);
    if (u < 0.0f || u > 1.0f) return false;
    const glm::vec3 q = glm::cross(s, e1);
    v = f * glm::dot(d, q);
    if (v < 0.0f || u + v > 1.0f) return false;
    tOut = f * glm::dot(e2, q);
    return tOut > tMin;
}

} // namespace engine::pt
