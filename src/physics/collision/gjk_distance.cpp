//
//  gjk_distance.cpp
//  engine::physics / collision
//

#include "engine/physics/collision/gjk_distance.h"

#include <cmath>

namespace engine::physics {
namespace {

struct SV { Vec3 p, a, b; };   // Minkowski point + witness on A / B

// Barycentric coords (u,v,w) of the origin's closest point on triangle (pa,pb,pc). (Ericson)
void closestTriBary(const Vec3& pa, const Vec3& pb, const Vec3& pc, Real& u, Real& v, Real& w) {
    const Vec3 ab = pb - pa, ac = pc - pa;
    const Vec3 ap = -pa;
    const Real d1 = glm::dot(ab, ap), d2 = glm::dot(ac, ap);
    if (d1 <= 0 && d2 <= 0) { u = 1; v = 0; w = 0; return; }
    const Vec3 bp = -pb;
    const Real d3 = glm::dot(ab, bp), d4 = glm::dot(ac, bp);
    if (d3 >= 0 && d4 <= d3) { u = 0; v = 1; w = 0; return; }
    const Real vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) { const Real t = d1 / (d1 - d3); u = 1 - t; v = t; w = 0; return; }
    const Vec3 cp = -pc;
    const Real d5 = glm::dot(ab, cp), d6 = glm::dot(ac, cp);
    if (d6 >= 0 && d5 <= d6) { u = 0; v = 0; w = 1; return; }
    const Real vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) { const Real t = d2 / (d2 - d6); u = 1 - t; v = 0; w = t; return; }
    const Real va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
        const Real t = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        u = 0; v = 1 - t; w = t; return;
    }
    const Real denom = Real(1) / (va + vb + vc);
    v = vb * denom; w = vc * denom; u = 1 - v - w;
}

Vec3 reduceTriangle(SV s[], int& n, Vec3& wA, Vec3& wB) {
    Real u, v, w;
    closestTriBary(s[0].p, s[1].p, s[2].p, u, v, w);
    SV keep[3]; Real wt[3]; int m = 0;
    if (u > 1e-6f) { keep[m] = s[0]; wt[m] = u; ++m; }
    if (v > 1e-6f) { keep[m] = s[1]; wt[m] = v; ++m; }
    if (w > 1e-6f) { keep[m] = s[2]; wt[m] = w; ++m; }
    Real tot = 0; for (int i = 0; i < m; ++i) tot += wt[i];
    Vec3 c(0); wA = Vec3(0); wB = Vec3(0);
    for (int i = 0; i < m; ++i) { const Real bw = wt[i] / tot; s[i] = keep[i]; c += keep[i].p * bw; wA += keep[i].a * bw; wB += keep[i].b * bw; }
    n = m;
    return c;
}

Vec3 reduce(SV s[], int& n, Vec3& wA, Vec3& wB) {
    if (n == 1) { wA = s[0].a; wB = s[0].b; return s[0].p; }
    if (n == 2) {
        const Vec3 A = s[0].p, B = s[1].p, AB = B - A;
        Real t = glm::dot(-A, AB);
        if (t <= 0) { n = 1; wA = s[0].a; wB = s[0].b; return A; }
        const Real den = glm::dot(AB, AB);
        if (t >= den) { s[0] = s[1]; n = 1; wA = s[0].a; wB = s[0].b; return s[0].p; }
        t /= den;
        wA = s[0].a * (1 - t) + s[1].a * t;
        wB = s[0].b * (1 - t) + s[1].b * t;
        return A + AB * t;
    }
    if (n == 3) return reduceTriangle(s, n, wA, wB);

    // n == 4 (tetra): if the origin is enclosed → overlap; else reduce to the closest face.
    const int faces[4][3] = { {0,1,2}, {0,1,3}, {0,2,3}, {1,2,3} };
    const int opp[4] = { 3, 2, 1, 0 };
    bool found = false;
    Real bestD2 = 1e30f;
    SV bestS[3]; int bestN = 0;
    Vec3 bestWA(0), bestWB(0), bestC(0);
    for (int f = 0; f < 4; ++f) {
        const Vec3& pi = s[faces[f][0]].p;
        const Vec3& pj = s[faces[f][1]].p;
        const Vec3& pk = s[faces[f][2]].p;
        Vec3 nrm = glm::cross(pj - pi, pk - pi);
        if (glm::dot(nrm, s[opp[f]].p - pi) > 0) nrm = -nrm;   // outward (away from the 4th vertex)
        if (glm::dot(nrm, -pi) <= kEpsilon) continue;          // origin not outside this face
        SV tri[3] = { s[faces[f][0]], s[faces[f][1]], s[faces[f][2]] };
        int tn = 3; Vec3 ta, tb;
        const Vec3 c = reduceTriangle(tri, tn, ta, tb);
        const Real d2 = glm::dot(c, c);
        if (d2 < bestD2) {
            bestD2 = d2; found = true; bestN = tn; bestWA = ta; bestWB = tb; bestC = c;
            for (int i = 0; i < tn; ++i) bestS[i] = tri[i];
        }
    }
    if (!found) { wA = wB = Vec3(0); return Vec3(0); }          // origin enclosed → overlap
    for (int i = 0; i < bestN; ++i) s[i] = bestS[i];
    n = bestN; wA = bestWA; wB = bestWB;
    return bestC;
}

} // namespace

Real gjkClosest(const SupportShape& a, const SupportShape& b, Vec3& witnessA, Vec3& witnessB) {
    auto sup = [&](const Vec3& d) { SV v; v.a = a.support(d); v.b = b.support(-d); v.p = v.a - v.b; return v; };

    SV s[4];
    Vec3 dir = a.center - b.center;
    if (glm::dot(dir, dir) < kEpsilon) dir = Vec3(1, 0, 0);
    s[0] = sup(dir);
    int n = 1;
    Vec3 closest = reduce(s, n, witnessA, witnessB);

    for (int iter = 0; iter < 64; ++iter) {
        if (glm::dot(closest, closest) < 1e-10f) return 0;   // overlap
        dir = -closest;
        const SV w = sup(dir);
        // Converged if the new support is no farther toward the origin than `closest`.
        if (glm::dot(w.p, closest) >= glm::dot(closest, closest) - 1e-7f * (glm::dot(closest, closest) + 1e-6f))
            break;
        s[n++] = w;
        closest = reduce(s, n, witnessA, witnessB);
    }
    return glm::length(closest);
}

} // namespace engine::physics
