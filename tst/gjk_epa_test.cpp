//
//  gjk_epa_test.cpp
//  engine::tst
//
//  Analytic checks for GJK boolean intersection + EPA penetration on convex shapes (box/box,
//  sphere/box) with known overlap depths and normals.
//

#include <cassert>
#include <cmath>
#include <cstdio>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "engine/physics/collision/convex.h"
#include "engine/physics/collision/gjk.h"

using namespace engine::physics;

static bool approx(Real a, Real b, Real e = Real(1e-3)) { return std::fabs(a - b) <= e; }

int main() {
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

    std::printf("gjk/epa ok\n");
    return 0;
}
