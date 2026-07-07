//
//  scene.h
//  engine::pathtracer
//
//  CPU scene model for the reference path tracer (step 1 of the path-tracer head; see
//  notes/investigations/path-tracing/2026-07-06-path-tracer-salvage-assessment.md). Plain glm data
//  — no Eigen, no RHI. World-space triangle soup + materials + an emissive index (for next-event
//  estimation) + a pinhole camera. Geometry is fed in from `engine::core` MeshData; a later adapter
//  will build a Scene from a render::RenderView. Acceleration structure (BVH) is deferred —
//  intersection is brute-force for now (fine for the small validation scenes).
//

#pragma once

#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

#include "engine/core/geometry/mesh.h"

namespace engine::pt {

enum class MaterialType : uint8_t {
    Diffuse,      // Lambertian
    Mirror,       // perfect specular reflection
    Dielectric,   // Fresnel reflect/refract (glass)
    Glossy,       // Phong specular lobe
};

// A path-tracer material. Distinct from core::Material (which is a raster PBR-ish stub): this adds
// emission + an explicit BSDF type, which offline light transport needs.
struct Material {
    glm::vec3    albedo   {0.8f};   // reflectance (diffuse/specular tint)
    glm::vec3    emission {0.0f};   // emitted radiance; > 0 ⇒ area light
    MaterialType type     = MaterialType::Diffuse;
    float        ior      = 1.5f;   // index of refraction (Dielectric)
    float        shininess = 32.0f; // Phong exponent (Glossy)

    bool emissive() const { return emission.x > 0.0f || emission.y > 0.0f || emission.z > 0.0f; }
};

// A world-space triangle with per-vertex normals and a material index.
struct Triangle {
    glm::vec3 v0{0.0f}, v1{0.0f}, v2{0.0f};
    glm::vec3 n0{0.0f}, n1{0.0f}, n2{0.0f};
    uint32_t  material = 0;

    glm::vec3 geoNormal() const { return glm::normalize(glm::cross(v1 - v0, v2 - v0)); }
    float     area()      const { return 0.5f * glm::length(glm::cross(v1 - v0, v2 - v0)); }
};

// A pinhole camera. `forward`/`up` need not be exactly orthonormal — the integrator orthonormalizes.
struct Camera {
    glm::vec3 eye     {0.0f, 0.0f, 0.0f};
    glm::vec3 forward {0.0f, 0.0f, -1.0f};
    glm::vec3 up      {0.0f, 1.0f, 0.0f};
    float     fovY    = glm::radians(45.0f);   // vertical field of view (aspect supplied at render time)
};

// Result of a scene ray query.
struct Hit {
    bool      valid = false;
    float     t     = 0.0f;         // distance along the ray
    uint32_t  tri   = 0;            // index into Scene::triangles
    glm::vec3 p{0.0f};              // world-space hit position
    glm::vec3 n{0.0f};              // interpolated (shading) normal, unit length
};

struct Scene {
    std::vector<Triangle> triangles;
    std::vector<Material> materials;
    std::vector<uint32_t> emissive;   // indices into `triangles` whose material emits (built by finalize())
    Camera                camera;

    uint32_t addMaterial(const Material& m) {
        materials.push_back(m);
        return static_cast<uint32_t>(materials.size() - 1);
    }

    // Append an indexed core MeshData, transformed by `model`, all faces sharing `material`.
    // Normals are transformed by the inverse-transpose (correct under non-uniform scale).
    void addMesh(const engine::MeshData& mesh, const glm::mat4& model, uint32_t material);

    // Rebuild the emissive-triangle index list. Call after all geometry is added.
    void finalize();

    // Nearest-hit ray query (brute force over all triangles; BVH deferred). Ignores hits ≤ tMin.
    Hit intersect(const glm::vec3& o, const glm::vec3& d, float tMin = 1e-4f) const;

    // Any occluder strictly between the origin and origin + maxDist·d? (shadow-ray test)
    bool occluded(const glm::vec3& o, const glm::vec3& d, float maxDist, float tMin = 1e-4f) const;
};

} // namespace engine::pt
