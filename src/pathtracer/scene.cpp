//
//  scene.cpp
//  engine::pathtracer
//

#include "engine/pathtracer/scene.h"

#include <limits>

#include <glm/gtc/matrix_inverse.hpp>

#include "engine/pathtracer/sampling.h"

namespace engine::pt {

void Scene::addMesh(const engine::MeshData& mesh, const glm::mat4& model, uint32_t material) {
    const glm::mat3 nmat = glm::inverseTranspose(glm::mat3(model));
    const auto& idx = mesh.indices;
    const auto& v   = mesh.vertices;
    triangles.reserve(triangles.size() + idx.size() / 3);

    auto xformN = [&](const glm::vec3& n) {
        const glm::vec3 t = nmat * n;
        const float len2 = glm::dot(t, t);
        return len2 > 1e-20f ? t * glm::inversesqrt(len2) : glm::vec3(0.0f);   // 0 ⇒ use geo normal at shade time
    };

    for (size_t i = 0; i + 2 < idx.size(); i += 3) {
        const Vertex& a = v[idx[i + 0]];
        const Vertex& b = v[idx[i + 1]];
        const Vertex& c = v[idx[i + 2]];
        Triangle t;
        t.v0 = glm::vec3(model * glm::vec4(a.position, 1.0f));
        t.v1 = glm::vec3(model * glm::vec4(b.position, 1.0f));
        t.v2 = glm::vec3(model * glm::vec4(c.position, 1.0f));
        t.n0 = xformN(a.normal);
        t.n1 = xformN(b.normal);
        t.n2 = xformN(c.normal);
        t.material = material;
        triangles.push_back(t);
    }
}

void Scene::finalize() {
    emissive.clear();
    for (uint32_t i = 0; i < triangles.size(); ++i) {
        if (materials[triangles[i].material].emissive()) emissive.push_back(i);
    }
}

Hit Scene::intersect(const glm::vec3& o, const glm::vec3& d, float tMin) const {
    Hit best;
    best.t = std::numeric_limits<float>::max();
    float t, u, v, bu = 0.0f, bv = 0.0f;
    for (uint32_t i = 0; i < triangles.size(); ++i) {
        const Triangle& tr = triangles[i];
        if (intersectTriangle(tr.v0, tr.v1, tr.v2, o, d, tMin, t, u, v) && t < best.t) {
            best.valid = true; best.t = t; best.tri = i; bu = u; bv = v;
        }
    }
    if (best.valid) {
        const Triangle& tr = triangles[best.tri];
        best.p = o + best.t * d;
        const glm::vec3 n = (1.0f - bu - bv) * tr.n0 + bu * tr.n1 + bv * tr.n2;
        best.n = (glm::dot(n, n) > 1e-12f) ? glm::normalize(n) : tr.geoNormal();
    }
    return best;
}

bool Scene::occluded(const glm::vec3& o, const glm::vec3& d, float maxDist, float tMin) const {
    float t, u, v;
    for (const Triangle& tr : triangles) {
        if (intersectTriangle(tr.v0, tr.v1, tr.v2, o, d, tMin, t, u, v) && t < maxDist - tMin) return true;
    }
    return false;
}

} // namespace engine::pt
