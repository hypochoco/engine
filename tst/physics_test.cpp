//
//  physics_test.cpp
//  engine::tst
//
//  Phase-0 physics kernels, verified against closed-form / analytic results (headless, no ECS,
//  no graphics): semi-implicit free-fall, sphere-plane and sphere-sphere contacts, SO(3)
//  exp/log orientation integration, and sphere inertia.
//

#include <cassert>
#include <cmath>
#include <cstdio>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "engine/physics/physics.h"

using namespace engine::physics;

static bool approx(Real a, Real b, Real eps = Real(1e-4)) { return std::fabs(a - b) <= eps; }
static bool approxVec(const Vec3& a, const Vec3& b, Real eps = Real(1e-4)) {
    return approx(a.x, b.x, eps) && approx(a.y, b.y, eps) && approx(a.z, b.z, eps);
}

int main() {
    // --- 1. Semi-implicit free fall matches the closed form ---
    // After n steps at dt: v = -g*dt*n ; y = -g*dt²*n(n+1)/2.
    {
        const Real g = Real(9.81), dt = Real(0.01);
        const int n = 100;
        Vec3 pos(0), vel(0);
        const Vec3 accel(0, -g, 0);
        for (int i = 0; i < n; ++i) integrateLinear(pos, vel, accel, dt);

        const Real expectedV = -g * dt * n;
        const Real expectedY = -g * dt * dt * (Real(n) * (n + 1) / Real(2));
        std::printf("free-fall: y=%.5f (exp %.5f)  v=%.5f (exp %.5f)\n",
                    pos.y, expectedY, vel.y, expectedV);
        assert(approx(vel.y, expectedV));
        assert(approx(pos.y, expectedY));
    }

    // --- 2. Sphere vs plane ---
    {
        Plane ground{ Vec3(0, 1, 0), Real(0) };
        Sphere s{ Real(0.5) };
        Contact c;

        // Penetrating: center 0.3 above ground, radius 0.5 -> separation -0.2.
        bool hit = collide::sphereVsPlane(Vec3(0, 0.3f, 0), s, ground, Real(0), c);
        assert(hit);
        assert(approx(c.separation, Real(-0.2)));
        assert(approxVec(c.normal, Vec3(0, 1, 0)));
        assert(approxVec(c.point, Vec3(0, -0.2f, 0)));   // lowest point of the sphere

        // Clear: center 1.0 above -> separation +0.5, not touching.
        Contact c2;
        bool hit2 = collide::sphereVsPlane(Vec3(0, 1.0f, 0), s, ground, Real(0), c2);
        assert(!hit2);
        assert(approx(c2.separation, Real(0.5)));
        std::printf("sphere-plane: pen sep=%.3f, clear sep=%.3f\n", c.separation, c2.separation);
    }

    // --- 3. Sphere vs sphere ---
    {
        Sphere a{ Real(1) }, b{ Real(1) };
        Contact c;
        bool hit = collide::sphereVsSphere(Vec3(0, 0, 0), a, Vec3(1.5f, 0, 0), b, Real(0), c);
        assert(hit);
        assert(approx(c.separation, Real(-0.5)));         // 1.5 - (1+1)
        assert(approxVec(c.normal, Vec3(1, 0, 0)));
        assert(approxVec(c.point, Vec3(1, 0, 0)));         // on A's surface
        std::printf("sphere-sphere: sep=%.3f, normal=(%.1f,%.1f,%.1f)\n",
                    c.separation, c.normal.x, c.normal.y, c.normal.z);
    }

    // --- 4. Orientation integration via exp map (rotate about +y by 90deg) ---
    {
        const Real quarter = Real(M_PI) * Real(0.5);
        const Vec3 v(0, 0, 1);

        // Single big step.
        Quat q1 = integrateOrientation(Quat(1, 0, 0, 0), Vec3(0, quarter, 0), Real(1));
        assert(approx(glm::length(q1), Real(1)));          // stays unit
        assert(approxVec(q1 * v, Vec3(1, 0, 0), Real(1e-3)));   // +y 90deg: z -> x

        // Many small steps accumulate to the same rotation.
        Quat q = Quat(1, 0, 0, 0);
        const int n = 100;
        for (int i = 0; i < n; ++i)
            q = integrateOrientation(q, Vec3(0, quarter, 0), Real(1) / n);
        assert(approx(glm::length(q), Real(1)));
        assert(approxVec(q * v, Vec3(1, 0, 0), Real(1e-3)));

        // exp/log round-trip.
        Vec3 phi(0.2f, -0.5f, 0.3f);
        Vec3 phi2 = so3LogMap(so3ExpMap(phi));
        assert(approxVec(phi, phi2, Real(1e-4)));
        std::printf("orientation: |q|=%.6f, z->%.3f,%.3f,%.3f ; log(exp) roundtrip ok\n",
                    glm::length(q), (q * v).x, (q * v).y, (q * v).z);
    }

    // --- 5. Solid sphere inertia (2/5 m r²) and its inverse ---
    {
        const Real m = Real(2), r = Real(0.5);
        Mat3 I = solidSphereInertia(m, r);
        Mat3 Iinv = solidSphereInvInertia(m, r);
        const Real expected = Real(2) / Real(5) * m * r * r;   // 0.2
        assert(approx(I[0][0], expected));
        assert(approx(Iinv[0][0], Real(1) / expected));
        assert(approx((I * Iinv)[0][0], Real(1)));
        std::printf("inertia: I=%.4f, Iinv=%.4f\n", I[0][0], Iinv[0][0]);
    }

    std::printf("physics phase-0 ok\n");
    return 0;
}
