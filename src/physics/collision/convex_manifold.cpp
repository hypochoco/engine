//
//  convex_manifold.cpp
//  engine::physics / collision
//

#include "engine/physics/collision/convex_manifold.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace engine::physics::collide {
namespace {

// Vertices whose projection onto `dir` is within `tol` of the maximum — the supporting feature.
std::vector<Vec3> extractFace(std::span<const Vec3> verts, const Vec3& dir, Real tol) {
    Real mx = -1e30f;
    for (const Vec3& v : verts) mx = std::max(mx, glm::dot(v, dir));
    std::vector<Vec3> poly;
    for (const Vec3& v : verts)
        if (glm::dot(v, dir) >= mx - tol) poly.push_back(v);
    return poly;
}

Vec3 centroid(const std::vector<Vec3>& poly) {
    Vec3 c(0);
    for (const Vec3& p : poly) c += p;
    return poly.empty() ? c : c / static_cast<Real>(poly.size());
}

// Order polygon vertices CCW around `axis` (so consecutive vertices form edges).
void orderPolygon(std::vector<Vec3>& poly, const Vec3& axis) {
    if (poly.size() < 3) return;
    const Vec3 c = centroid(poly);
    Vec3 u = poly[0] - c;
    u -= axis * glm::dot(u, axis);
    if (glm::dot(u, u) < kEpsilon) return;
    u = glm::normalize(u);
    const Vec3 w = glm::cross(axis, u);
    std::sort(poly.begin(), poly.end(), [&](const Vec3& p, const Vec3& q) {
        const Vec3 pp = p - c, qq = q - c;
        return std::atan2(glm::dot(pp, w), glm::dot(pp, u)) <
               std::atan2(glm::dot(qq, w), glm::dot(qq, u));
    });
}

// Sutherland-Hodgman: keep the part of `poly` with dot(n,p) <= offset.
void clip(std::vector<Vec3>& poly, const Vec3& n, Real offset) {
    std::vector<Vec3> out;
    const size_t m = poly.size();
    for (size_t k = 0; k < m; ++k) {
        const Vec3& cur = poly[k];
        const Vec3& prev = poly[(k + m - 1) % m];
        const Real dc = glm::dot(n, cur) - offset;
        const Real dp = glm::dot(n, prev) - offset;
        if (dc <= 0) {
            if (dp > 0) out.push_back(prev + (cur - prev) * (dp / (dp - dc)));
            out.push_back(cur);
        } else if (dp <= 0) {
            out.push_back(prev + (cur - prev) * (dp / (dp - dc)));
        }
    }
    poly.swap(out);
}

} // namespace

int polytopeManifold(std::span<const Vec3> vertsA, std::span<const Vec3> vertsB,
                     const Contact& epa, Contact out[4]) {
    const Vec3 n = epa.normal;         // A -> B
    const Real tol = Real(0.02);

    std::vector<Vec3> Pa = extractFace(vertsA, n, tol);    // A's face toward B
    std::vector<Vec3> Pb = extractFace(vertsB, -n, tol);   // B's face toward A

    bool refA;
    if (Pa.size() >= 3 && Pa.size() >= Pb.size()) refA = true;
    else if (Pb.size() >= 3)                      refA = false;
    else { out[0] = epa; return 1; }               // edge/vertex contact → single point

    const Vec3 refN = refA ? n : -n;
    std::vector<Vec3>& refPoly = refA ? Pa : Pb;
    std::vector<Vec3>& incPoly = refA ? Pb : Pa;
    if (incPoly.empty()) { out[0] = epa; return 1; }

    orderPolygon(refPoly, refN);
    orderPolygon(incPoly, refN);

    // Clip the incident polygon against the reference face's side planes.
    const Vec3 rc = centroid(refPoly);
    const size_t m = refPoly.size();
    for (size_t k = 0; k < m && !incPoly.empty(); ++k) {
        const Vec3 e0 = refPoly[k], e1 = refPoly[(k + 1) % m];
        Vec3 sideN = glm::cross(e1 - e0, refN);
        if (glm::dot(sideN, sideN) < kEpsilon) continue;
        sideN = glm::normalize(sideN);
        Real off = glm::dot(sideN, e0);
        if (glm::dot(sideN, rc) > off) { sideN = -sideN; off = glm::dot(sideN, e0); }   // inside = centroid side
        clip(incPoly, sideN, off);
    }
    if (incPoly.empty()) { out[0] = epa; return 1; }

    // Keep points penetrating the reference face plane; separation = signed distance below it.
    const Real refOff = glm::dot(refN, refPoly.empty() ? epa.point : refPoly[0]);
    Contact cand[32];
    int cnt = 0;
    for (const Vec3& p : incPoly) {
        const Real sep = glm::dot(refN, p) - refOff;
        if (sep <= 0 && cnt < 32) {
            cand[cnt].normal = n;                  // always A -> B
            cand[cnt].point = p;
            cand[cnt].separation = sep;
            cand[cnt].touching = true;
            ++cnt;
        }
    }
    if (cnt == 0) { out[0] = epa; return 1; }

    const int keep = std::min(cnt, 4);
    for (int i = 0; i < keep; ++i) {
        int bi = i;
        for (int j = i + 1; j < cnt; ++j)
            if (cand[j].separation < cand[bi].separation) bi = j;
        std::swap(cand[i], cand[bi]);
        out[i] = cand[i];
    }
    return keep;
}

} // namespace engine::physics::collide
