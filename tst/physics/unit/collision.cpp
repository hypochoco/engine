//
//  gjk_epa_test.cpp
//  engine::tst
//
//  Analytic checks for GJK boolean intersection + EPA penetration on convex shapes (box/box,
//  sphere/box) with known overlap depths and normals.
//

#include "harness/harness.h"
#include <cmath>
#include <cstdio>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "engine/physics/collision/capsule.h"
#include "engine/physics/collision/convex.h"
#include "engine/physics/collision/gjk.h"
#include "engine/physics/collision/gjk_distance.h"

using namespace engine::physics;

static bool approx(Real a, Real b, Real e = Real(1e-3)) { return std::fabs(a - b) <= e; }

TST_CASE(physics, unit, collision) {
    const Quat I(1, 0, 0, 0);
    const Vec3 half(0.5f);

    // Two unit boxes overlapping by 0.2 along x.
    {
        auto A = SupportShape::box(Vec3(0, 0, 0), I, half);
        auto B = SupportShape::box(Vec3(0.8f, 0, 0), I, half);
        Simplex s;
        assert(gjkIntersect(A, B, s));
        Contact c;
        assert(collide::convexVsConvex(A, B, c));
        std::printf("box-box x: normal=(%.2f,%.2f,%.2f) sep=%.3f\n",
                    c.normal.x, c.normal.y, c.normal.z, c.separation);
        assert(approx(c.separation, -0.2f));
        assert(approx(std::fabs(c.normal.x), 1.0f) && c.normal.x > 0);   // A -> B is +x
    }

    // Separated boxes (gap 0.2) → no intersection.
    {
        auto A = SupportShape::box(Vec3(0, 0, 0), I, half);
        auto B = SupportShape::box(Vec3(1.2f, 0, 0), I, half);
        Simplex s;
        assert(!gjkIntersect(A, B, s));
        Contact c;
        assert(!collide::convexVsConvex(A, B, c));
    }

    // Overlap along y by 0.3.
    {
        auto A = SupportShape::box(Vec3(0, 0, 0), I, half);
        auto B = SupportShape::box(Vec3(0, 0.7f, 0), I, half);
        Contact c;
        assert(collide::convexVsConvex(A, B, c));
        std::printf("box-box y: normal=(%.2f,%.2f,%.2f) sep=%.3f\n",
                    c.normal.x, c.normal.y, c.normal.z, c.separation);
        assert(approx(c.separation, -0.3f));
        assert(approx(std::fabs(c.normal.y), 1.0f) && c.normal.y > 0);
    }

    // Overlap along z by 0.1.
    {
        auto A = SupportShape::box(Vec3(0, 0, 0), I, half);
        auto B = SupportShape::box(Vec3(0, 0, 0.9f), I, half);
        Contact c;
        assert(collide::convexVsConvex(A, B, c));
        std::printf("box-box z: normal=(%.2f,%.2f,%.2f) sep=%.3f\n",
                    c.normal.x, c.normal.y, c.normal.z, c.separation);
        assert(approx(c.separation, -0.1f));
        assert(approx(std::fabs(c.normal.z), 1.0f) && c.normal.z > 0);
    }

    // Rotated box vs box still intersects (GJK boolean).
    {
        const Quat rot = glm::angleAxis(glm::radians(35.0f), glm::normalize(Vec3(1, 1, 0)));
        auto A = SupportShape::box(Vec3(0, 0, 0), rot, half);
        auto B = SupportShape::box(Vec3(0.6f, 0.2f, 0), I, half);
        Simplex s;
        assert(gjkIntersect(A, B, s));
        Contact c;
        assert(collide::convexVsConvex(A, B, c));
        std::printf("rotated box-box: normal=(%.2f,%.2f,%.2f) sep=%.3f\n",
                    c.normal.x, c.normal.y, c.normal.z, c.separation);
        assert(c.separation < 0);
    }

    // Convex hull (box-shaped, 8 verts) overlap via the hull support path.
    {
        Vec3 hv[8];
        int k = 0;
        for (int sx = -1; sx <= 1; sx += 2)
            for (int sy = -1; sy <= 1; sy += 2)
                for (int sz = -1; sz <= 1; sz += 2)
                    hv[k++] = Vec3(sx * 0.5f, sy * 0.5f, sz * 0.5f);
        auto A = SupportShape::hull(Vec3(0, 0, 0), I, hv, 8);
        auto B = SupportShape::hull(Vec3(0.8f, 0, 0), I, hv, 8);
        Contact c;
        assert(collide::convexVsConvex(A, B, c));
        std::printf("hull-hull x: normal=(%.2f,%.2f,%.2f) sep=%.3f\n",
                    c.normal.x, c.normal.y, c.normal.z, c.separation);
        assert(approx(c.separation, -0.2f));
        assert(c.normal.x > 0.9f);
    }

    // GJK distance: a point above a box, and a lying segment above a box.
    {
        auto box = SupportShape::box(Vec3(0, 0.5f, 0), I, Vec3(2, 0.5f, 2));   // top at y=1
        auto pt  = SupportShape::sphere(Vec3(0, 1.6f, 0), 0);
        Vec3 wa, wb;
        Real d = gjkClosest(pt, box, wa, wb);
        std::printf("dist pt-box=%.3f witnessBox=(%.2f,%.2f,%.2f)\n", d, wb.x, wb.y, wb.z);
        assert(approx(d, 0.6f, 1e-2f) && approx(wb.y, 1.0f, 1e-2f));

        auto seg = SupportShape::capsule(Vec3(0, 1.6f, 0),
                                         glm::angleAxis(glm::radians(90.0f), Vec3(0, 0, 1)), 0, 0.8f);
        d = gjkClosest(seg, box, wa, wb);
        std::printf("dist seg-box=%.3f witnessBox.y=%.3f\n", d, wb.y);
        assert(approx(d, 0.6f, 1e-2f));
    }

    // capsuleVsConvex at a resting configuration (lying capsule just above a box top).
    {
        auto box = SupportShape::box(Vec3(0, 0.5f, 0), I, Vec3(2, 0.5f, 2));   // top y=1
        Capsule cap{ 0.4f, 0.8f };
        const Quat rot = glm::angleAxis(glm::radians(90.0f), Vec3(0, 0, 1));   // lying along X
        Contact cs[2];
        int nc = collide::capsuleVsConvex(Vec3(0, 1.35f, 0), rot, cap, box, Real(0), cs);
        assert(nc >= 1);
        assert(cs[0].normal.y > 0.9f);
        assert(cs[0].separation < 0);
    }

    std::printf("gjk/epa ok\n");
}
