//
//  simplify.cpp
//  engine::core::geo — quadric-error (Garland–Heckbert) edge-collapse simplification
//

#include "engine/core/geometry/geo.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <queue>
#include <unordered_set>
#include <vector>

namespace engine::geo {

namespace {

// Quadric for a plane (n, d): K = [n; d] [n; d]^T (symmetric 4×4), accumulated per vertex.
glm::mat4 planeQuadric(const glm::vec3& n, float d) {
    const glm::vec4 v(n, d);
    glm::mat4 K(0.0f);
    for (int c = 0; c < 4; ++c)
        for (int r = 0; r < 4; ++r)
            K[c][r] = v[c] * v[r];
    return K;
}

float quadricError(const glm::mat4& Q, const glm::vec3& p) {
    const glm::vec4 x(p, 1.0f);
    return glm::dot(x, Q * x);
}

// Optimal collapsed position: minimize x^T Q x with x_w = 1. Fall back to the midpoint when the
// system is (near-)singular.
glm::vec3 optimalPos(const glm::mat4& Q, const glm::vec3& a, const glm::vec3& b) {
    glm::mat4 A = Q;
    A[0][3] = 0.0f; A[1][3] = 0.0f; A[2][3] = 0.0f; A[3][3] = 1.0f;   // last row → (0,0,0,1)
    if (std::fabs(glm::determinant(A)) > 1e-8f) {
        const glm::vec4 x = glm::inverse(A) * glm::vec4(0, 0, 0, 1);
        return glm::vec3(x);
    }
    const glm::vec3 m = 0.5f * (a + b);
    const float ea = quadricError(Q, a), eb = quadricError(Q, b), em = quadricError(Q, m);
    if (ea <= eb && ea <= em) return a;
    if (eb <= ea && eb <= em) return b;
    return m;
}

struct Cand {
    float cost;
    int   i, j;        // collapse j → i
    int   vi, vj;      // version stamps for lazy invalidation
    bool operator>(const Cand& o) const { return cost > o.cost; }
};

} // namespace

MeshData simplify(const MeshData& mesh, std::size_t targetTriangles) {
    const std::size_t triCount = mesh.indices.size() / 3;
    if (triCount <= targetTriangles || mesh.indices.size() < 3) return mesh;

    const std::size_t nv = mesh.vertices.size();
    std::vector<glm::vec3> pos(nv);
    for (std::size_t i = 0; i < nv; ++i) pos[i] = mesh.vertices[i].position;

    std::vector<std::array<int, 3>> tri(triCount);
    std::vector<uint8_t> tAlive(triCount, 1);
    std::vector<glm::mat4> Q(nv, glm::mat4(0.0f));
    std::vector<std::unordered_set<int>> vtris(nv);

    for (std::size_t t = 0; t < triCount; ++t) {
        const int a = int(mesh.indices[3 * t]), b = int(mesh.indices[3 * t + 1]), c = int(mesh.indices[3 * t + 2]);
        tri[t] = { a, b, c };
        vtris[a].insert(int(t)); vtris[b].insert(int(t)); vtris[c].insert(int(t));
        const glm::vec3 n = glm::cross(pos[b] - pos[a], pos[c] - pos[a]);
        const float len = glm::length(n);
        if (len < 1e-20f) continue;
        const glm::vec3 un = n / len;
        const glm::mat4 K = planeQuadric(un, -glm::dot(un, pos[a]));
        Q[a] += K; Q[b] += K; Q[c] += K;
    }

    std::vector<uint8_t> vAlive(nv, 1);
    std::vector<int> version(nv, 0);

    auto triNormal = [&](const std::array<int, 3>& f, int rep, int with, const glm::vec3& withPos) {
        glm::vec3 p[3];
        for (int e = 0; e < 3; ++e) p[e] = (f[e] == rep) ? withPos : (f[e] == with ? withPos : pos[f[e]]);
        return glm::cross(p[1] - p[0], p[2] - p[0]);
    };

    std::priority_queue<Cand, std::vector<Cand>, std::greater<Cand>> pq;
    auto pushEdge = [&](int i, int j) {
        if (i == j || !vAlive[i] || !vAlive[j]) return;
        const glm::mat4 Qs = Q[i] + Q[j];
        const glm::vec3 x = optimalPos(Qs, pos[i], pos[j]);
        pq.push({ quadricError(Qs, x), i, j, version[i], version[j] });
    };

    // Seed with every unique edge.
    {
        std::unordered_set<std::int64_t> seen;
        auto key = [](int a, int b) { if (a > b) std::swap(a, b); return (std::int64_t(a) << 32) | std::uint32_t(b); };
        for (std::size_t t = 0; t < triCount; ++t)
            for (int e = 0; e < 3; ++e) {
                const int a = tri[t][e], b = tri[t][(e + 1) % 3];
                if (seen.insert(key(a, b)).second) pushEdge(a, b);
            }
    }

    std::size_t aliveTris = triCount;

    while (aliveTris > targetTriangles && !pq.empty()) {
        const Cand cand = pq.top(); pq.pop();
        const int i = cand.i, j = cand.j;
        if (!vAlive[i] || !vAlive[j]) continue;
        if (version[i] != cand.vi || version[j] != cand.vj) continue;   // stale

        // Faces shared by (i,j) are removed; the rest must not flip. Manifold guard: exactly the
        // shared faces collapse.
        const glm::vec3 newPos = optimalPos(Q[i] + Q[j], pos[i], pos[j]);
        bool flip = false;
        int shared = 0;
        for (int t : vtris[i]) { if (!tAlive[t]) continue; const auto& f = tri[t];
            if (f[0] == j || f[1] == j || f[2] == j) { ++shared; continue; }
            if (glm::dot(triNormal(f, i, -1, pos[i]), triNormal(f, i, -1, newPos)) < 0.0f) { flip = true; break; } }
        if (!flip)
            for (int t : vtris[j]) { if (!tAlive[t]) continue; const auto& f = tri[t];
                if (f[0] == i || f[1] == i || f[2] == i) continue;   // shared, already counted
                if (glm::dot(triNormal(f, j, -1, pos[j]), triNormal(f, j, i, newPos)) < 0.0f) { flip = true; break; } }
        if (flip || shared == 0) continue;   // reject foldover / non-edge pairs

        // Commit collapse j → i.
        pos[i] = newPos;
        Q[i] += Q[j];
        std::vector<int> touched;
        for (int t : vtris[j]) {
            auto& f = tri[t];
            if (!tAlive[t]) continue;
            if (f[0] == i || f[1] == i || f[2] == i) {           // shared face → dies
                tAlive[t] = 0; --aliveTris;
                for (int e = 0; e < 3; ++e) if (f[e] != j) vtris[f[e]].erase(t);
            } else {
                for (int e = 0; e < 3; ++e) if (f[e] == j) f[e] = i;
                vtris[i].insert(t);
                touched.push_back(t);
            }
        }
        vtris[j].clear();
        vAlive[j] = 0;
        ++version[i]; ++version[j];

        // Bump neighbors' versions and re-seed edges incident to i.
        std::unordered_set<int> nbrs;
        for (int t : vtris[i]) { if (!tAlive[t]) continue; for (int e = 0; e < 3; ++e) if (tri[t][e] != i) nbrs.insert(tri[t][e]); }
        for (int nb : nbrs) ++version[nb];
        for (int nb : nbrs) pushEdge(i, nb);
    }

    // Compact surviving vertices/faces and recompute normals.
    MeshData out;
    std::vector<int> remap(nv, -1);
    for (std::size_t v = 0; v < nv; ++v) if (vAlive[v]) {
        remap[v] = int(out.vertices.size());
        Vertex nvx; nvx.position = pos[v]; nvx.color = glm::vec3(1.0f); out.vertices.push_back(nvx);
    }
    std::vector<glm::vec3> nrm(out.vertices.size(), glm::vec3(0.0f));
    for (std::size_t t = 0; t < triCount; ++t) {
        if (!tAlive[t]) continue;
        const int a = remap[tri[t][0]], b = remap[tri[t][1]], c = remap[tri[t][2]];
        if (a < 0 || b < 0 || c < 0 || a == b || b == c || a == c) continue;
        out.indices.insert(out.indices.end(), { uint32_t(a), uint32_t(b), uint32_t(c) });
        const glm::vec3 fn = glm::cross(out.vertices[b].position - out.vertices[a].position,
                                        out.vertices[c].position - out.vertices[a].position);
        nrm[a] += fn; nrm[b] += fn; nrm[c] += fn;
    }
    for (std::size_t v = 0; v < out.vertices.size(); ++v)
        out.vertices[v].normal = glm::length(nrm[v]) > 1e-20f ? glm::normalize(nrm[v]) : glm::vec3(0, 1, 0);

    return out;
}

} // namespace engine::geo
