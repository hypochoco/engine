//
//  mesher.cpp
//  engine::core::geo — capsule-SDF union → watertight mesh (Naive Surface Nets)
//

#include "engine/core/geometry/geo.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <thread>
#include <vector>

namespace engine::geo {

namespace {

inline float dot2(const glm::vec3& v) { return glm::dot(v, v); }

// Exact signed distance to a round cone (tapered capsule): radius r1 at a, r2 at b. (Inigo Quilez.)
float sdRoundCone(const glm::vec3& p, const glm::vec3& a, const glm::vec3& b, float r1, float r2) {
    const glm::vec3 ba = b - a;
    const float l2 = glm::dot(ba, ba);
    if (l2 < 1e-12f) return glm::length(p - a) - r1;     // degenerate → sphere
    const float rr = r1 - r2;
    const float a2 = l2 - rr * rr;
    const float il2 = 1.0f / l2;
    const glm::vec3 pa = p - a;
    const float y = glm::dot(pa, ba);
    const float z = y - l2;
    const float x2 = dot2(pa * l2 - ba * y);
    const float y2 = y * y * l2;
    const float z2 = z * z * l2;
    const float k = glm::sign(rr) * rr * rr * x2;
    if (glm::sign(z) * a2 * z2 > k) return std::sqrt(x2 + z2) * il2 - r2;
    if (glm::sign(y) * a2 * y2 < k) return std::sqrt(x2 + y2) * il2 - r1;
    return (std::sqrt(x2 * a2 * il2) + y * rr) * il2 - r1;
}

// Smooth minimum (polynomial). k<=0 ⇒ hard min.
inline float smin(float a, float b, float k) {
    if (k <= 0.0f) return std::min(a, b);
    const float h = glm::clamp(0.5f + 0.5f * (b - a) / k, 0.0f, 1.0f);
    return glm::mix(b, a, h) - k * h * (1.0f - h);
}

struct Field {
    std::span<const Capsule> caps;
    float smooth;
    float operator()(const glm::vec3& p) const {
        float d = 1e30f;
        for (const Capsule& c : caps) d = smin(d, sdRoundCone(p, c.a, c.b, c.ra, c.rb), smooth);
        return d;
    }
};

} // namespace

MeshData meshCapsules(std::span<const Capsule> capsules, const MesherParams& params) {
    MeshData out;
    if (capsules.empty()) return out;

    const float voxel = std::max(params.voxel, 1e-4f);
    const Field field{ capsules, params.smooth };

    // AABB of the capsule union (segment endpoints ± radius), then margin so grid corners are outside.
    glm::vec3 lo(1e30f), hi(-1e30f);
    float maxR = 0.0f;
    for (const Capsule& c : capsules) {
        const float r = std::max(c.ra, c.rb);
        lo = glm::min(lo, glm::min(c.a, c.b) - r);
        hi = glm::max(hi, glm::max(c.a, c.b) + r);
        maxR = std::max(maxR, r);
    }
    const float pad = (params.padding < 0.0f ? 2.0f * voxel : params.padding) + params.smooth;
    lo -= pad; hi += pad;

    const glm::vec3 origin = lo;
    const glm::ivec3 dim = glm::ivec3(glm::ceil((hi - lo) / voxel)) + 2;   // corner grid dims
    if (dim.x < 2 || dim.y < 2 || dim.z < 2) return out;
    // Guard against pathological memory use.
    const std::int64_t corners = std::int64_t(dim.x) * dim.y * dim.z;
    if (corners > 64ll * 1024 * 1024) return out;

    const int nx = dim.x, ny = dim.y, nz = dim.z;
    auto cornerPos = [&](int i, int j, int k) { return origin + voxel * glm::vec3(i, j, k); };
    auto fIdx = [&](int i, int j, int k) { return i + nx * (j + ny * k); };

    // Sample the field at every corner (parallel over z-slabs — the dominant bake cost).
    std::vector<float> f(static_cast<std::size_t>(corners));
    {
        const unsigned hw = std::max(1u, std::thread::hardware_concurrency());
        const int nthreads = std::min<int>(static_cast<int>(hw), std::max(1, nz));
        auto sampleSlab = [&](int kBegin, int kEnd) {
            for (int k = kBegin; k < kEnd; ++k)
                for (int j = 0; j < ny; ++j)
                    for (int i = 0; i < nx; ++i)
                        f[fIdx(i, j, k)] = field(cornerPos(i, j, k));
        };
        if (nthreads <= 1) {
            sampleSlab(0, nz);
        } else {
            std::vector<std::thread> pool;
            const int per = (nz + nthreads - 1) / nthreads;
            for (int t = 0; t < nthreads; ++t) {
                const int kb = t * per, ke = std::min(nz, kb + per);
                if (kb < ke) pool.emplace_back(sampleSlab, kb, ke);
            }
            for (auto& th : pool) th.join();
        }
    }

    // Gradient (central differences) → outward normal.
    const float ge = 0.5f * voxel;
    auto gradient = [&](const glm::vec3& p) {
        return glm::normalize(glm::vec3(
            field(p + glm::vec3(ge, 0, 0)) - field(p - glm::vec3(ge, 0, 0)),
            field(p + glm::vec3(0, ge, 0)) - field(p - glm::vec3(0, ge, 0)),
            field(p + glm::vec3(0, 0, ge)) - field(p - glm::vec3(0, 0, ge))));
    };

    const int cx = nx - 1, cy = ny - 1, cz = nz - 1;
    std::vector<uint32_t> cellVert(static_cast<std::size_t>(cx) * cy * cz, UINT32_MAX);
    auto cIdx = [&](int i, int j, int k) { return i + cx * (j + cy * k); };

    // Cube corner offsets (bit0=x, bit1=y, bit2=z) and the 12 edges as corner-index pairs.
    static const int co[8][3] = { {0,0,0},{1,0,0},{0,1,0},{1,1,0},{0,0,1},{1,0,1},{0,1,1},{1,1,1} };
    static const int ce[12][2] = { {0,1},{2,3},{4,5},{6,7}, {0,2},{1,3},{4,6},{5,7}, {0,4},{1,5},{2,6},{3,7} };

    // Pass 1: one dual vertex per active cell (average of its edge zero-crossings).
    for (int k = 0; k < cz; ++k)
        for (int j = 0; j < cy; ++j)
            for (int i = 0; i < cx; ++i) {
                float cf[8];
                bool neg = false, pos = false;
                for (int c = 0; c < 8; ++c) {
                    cf[c] = f[fIdx(i + co[c][0], j + co[c][1], k + co[c][2])];
                    neg |= (cf[c] < 0.0f); pos |= (cf[c] >= 0.0f);
                }
                if (!(neg && pos)) continue;                       // no sign change → inactive cell

                glm::vec3 sum(0.0f); int n = 0;
                for (const auto& e : ce) {
                    const float a = cf[e[0]], b = cf[e[1]];
                    if ((a < 0.0f) == (b < 0.0f)) continue;
                    const float t = a / (a - b);                   // zero crossing along the edge
                    const glm::vec3 pa = cornerPos(i + co[e[0]][0], j + co[e[0]][1], k + co[e[0]][2]);
                    const glm::vec3 pb = cornerPos(i + co[e[1]][0], j + co[e[1]][1], k + co[e[1]][2]);
                    sum += glm::mix(pa, pb, t); ++n;
                }
                const glm::vec3 vp = sum / static_cast<float>(n);
                Vertex v; v.position = vp; v.normal = gradient(vp); v.color = glm::vec3(1.0f);
                cellVert[cIdx(i, j, k)] = static_cast<uint32_t>(out.vertices.size());
                out.vertices.push_back(v);
            }

    // Pass 2: for each interior grid edge with a sign change, connect the 4 surrounding cells' dual
    // vertices into a quad (two triangles). Winding is fixed afterward from the SDF gradient.
    auto quad = [&](uint32_t a, uint32_t b, uint32_t c, uint32_t d) {
        if (a == UINT32_MAX || b == UINT32_MAX || c == UINT32_MAX || d == UINT32_MAX) return;
        out.indices.insert(out.indices.end(), { a, b, c, a, c, d });
    };
    for (int k = 0; k < nz; ++k)
        for (int j = 0; j < ny; ++j)
            for (int i = 0; i < nx; ++i) {
                const float f0 = f[fIdx(i, j, k)];
                if (i + 1 < nx && j >= 1 && k >= 1 && j < cy && k < cz) {          // +x edge
                    const float f1 = f[fIdx(i + 1, j, k)];
                    if ((f0 < 0.0f) != (f1 < 0.0f))
                        quad(cellVert[cIdx(i, j - 1, k - 1)], cellVert[cIdx(i, j, k - 1)],
                             cellVert[cIdx(i, j, k)],         cellVert[cIdx(i, j - 1, k)]);
                }
                if (j + 1 < ny && i >= 1 && k >= 1 && i < cx && k < cz) {          // +y edge
                    const float f1 = f[fIdx(i, j + 1, k)];
                    if ((f0 < 0.0f) != (f1 < 0.0f))
                        quad(cellVert[cIdx(i - 1, j, k - 1)], cellVert[cIdx(i, j, k - 1)],
                             cellVert[cIdx(i, j, k)],         cellVert[cIdx(i - 1, j, k)]);
                }
                if (k + 1 < nz && i >= 1 && j >= 1 && i < cx && j < cy) {          // +z edge
                    const float f1 = f[fIdx(i, j, k + 1)];
                    if ((f0 < 0.0f) != (f1 < 0.0f))
                        quad(cellVert[cIdx(i - 1, j - 1, k)], cellVert[cIdx(i, j - 1, k)],
                             cellVert[cIdx(i, j, k)],         cellVert[cIdx(i - 1, j, k)]);
                }
            }

    // Fix winding: make each triangle's geometric normal agree with the (outward) SDF gradient.
    for (std::size_t t = 0; t + 2 < out.indices.size(); t += 3) {
        const glm::vec3& p0 = out.vertices[out.indices[t]].position;
        const glm::vec3& p1 = out.vertices[out.indices[t + 1]].position;
        const glm::vec3& p2 = out.vertices[out.indices[t + 2]].position;
        const glm::vec3 gn = glm::cross(p1 - p0, p2 - p0);
        const glm::vec3 grad = out.vertices[out.indices[t]].normal
                             + out.vertices[out.indices[t + 1]].normal
                             + out.vertices[out.indices[t + 2]].normal;
        if (glm::dot(gn, grad) < 0.0f) std::swap(out.indices[t + 1], out.indices[t + 2]);
    }

    return out;
}

} // namespace engine::geo
