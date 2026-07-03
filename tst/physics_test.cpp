//
//  physics_test.cpp
//  engine::tst
//
//  Phase-0 physics kernels, verified against closed-form / analytic results (headless, no ECS,
//  no graphics): semi-implicit free-fall, sphere-plane and sphere-sphere contacts, SO(3)
//  exp/log orientation integration, and sphere inertia.
//

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "engine/core/threading/thread_pool.h"
#include "engine/physics/broadphase/sweep_and_prune.h"
#include "engine/physics/broadphase/uniform_grid.h"
#include "engine/physics/physics.h"
#include "engine/physics/world.h"

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

    // --- 6. Broadphase (sweep-and-prune) matches brute-force overlap set ---
    {
        std::vector<Aabb> boxes;
        // A jittered 6x6 grid of unit boxes so some neighbors overlap and most don't.
        for (int gx = 0; gx < 6; ++gx)
            for (int gy = 0; gy < 6; ++gy) {
                const Vec3 c(gx * 0.9f, gy * 0.9f, 0.0f);   // spacing 0.9 < size 1 -> neighbors touch
                boxes.push_back(Aabb{ c - Vec3(0.5f), c + Vec3(0.5f) });
            }

        std::vector<broadphase::Pair> sap;
        broadphase::sweepAndPrune(boxes, sap);

        std::vector<broadphase::Pair> grid;
        broadphase::uniformGrid(boxes, grid);

        std::vector<broadphase::Pair> brute;
        for (uint32_t i = 0; i < boxes.size(); ++i)
            for (uint32_t j = i + 1; j < boxes.size(); ++j)
                if (overlaps(boxes[i], boxes[j])) brute.emplace_back(i, j);
        std::sort(brute.begin(), brute.end());

        assert(sap == brute);
        assert(grid == brute);
        std::printf("broadphase: SAP + grid both found %zu pairs, match brute force (%zu)\n",
                    grid.size(), brute.size());
    }

    // --- 7. Parallel step produces the same result as serial (determinism) ---
    {
        engine::core::ThreadPool pool;
        auto buildPile = [](engine::core::ThreadPool* p) {
            WorldDef wd;
            wd.gravity = Vec3(0, -9.81f, 0);
            wd.velocityIterations = 8;
            wd.substeps = 1;
            wd.threadPool = p;
            wd.parallelThreshold = p ? 1 : 1000000;   // force the parallel path when pooled
            auto w = createPhysicsWorld(Backend::Realtime, wd);
            BodyDef plane;
            plane.type = BodyType::Static;
            plane.collider.type = ColliderDesc::Type::Plane;
            plane.collider.plane = Plane{ Vec3(0, 1, 0), 0.0f };
            plane.material.friction = 0.8f;
            w->createBody(plane);
            for (int x = 0; x < 5; ++x)
                for (int z = 0; z < 5; ++z)
                    for (int y = 0; y < 2; ++y) {
                        BodyDef s;
                        s.type = BodyType::Dynamic;
                        s.mass = 1.0f;
                        s.collider.type = ColliderDesc::Type::Sphere;
                        s.collider.sphere = Sphere{ 0.5f };
                        s.material.friction = 0.8f;
                        s.position = Vec3(x * 0.8f, 1.0f + y * 0.9f, z * 0.8f);   // overlapping cluster
                        w->createBody(s);
                    }
            return w;
        };

        auto serial = buildPile(nullptr);
        auto parallel = buildPile(&pool);
        for (int i = 0; i < 120; ++i) { serial->step(1.0f / 120.0f); parallel->step(1.0f / 120.0f); }

        const auto ps = serial->poses();
        const auto pp = parallel->poses();
        assert(ps.size() == pp.size());
        Real maxErr = 0;
        for (size_t k = 0; k < ps.size(); ++k)
            maxErr = std::max(maxErr, glm::length(ps[k].position - pp[k].position));
        std::printf("determinism: parallel vs serial max pos err = %.3e\n", maxErr);
        assert(maxErr < 1e-4f);
    }

    // --- 8. Box rests stably on a plane; sphere rests on top of a static box ---
    {
        WorldDef wd;
        wd.gravity = Vec3(0, -9.81f, 0);
        wd.velocityIterations = 12;
        wd.substeps = 2;
        auto w = createPhysicsWorld(Backend::Realtime, wd);

        BodyDef plane;
        plane.type = BodyType::Static;
        plane.collider.type = ColliderDesc::Type::Plane;
        plane.collider.plane = Plane{ Vec3(0, 1, 0), 0.0f };
        plane.material.friction = 0.8f;
        w->createBody(plane);

        BodyDef box;
        box.type = BodyType::Dynamic;
        box.mass = 1.0f;
        box.collider.type = ColliderDesc::Type::Box;
        box.collider.box = Box{ Vec3(0.5f) };
        box.material.friction = 0.8f;
        box.material.restitution = 0.0f;
        box.position = Vec3(0, 1.0f, 0);
        const BodyHandle bh = w->createBody(box);

        for (int i = 0; i < 300; ++i) w->step(1.0f / 120.0f);
        const auto p = w->pose(bh);
        const Real angSpeed = glm::length(w->angularVelocities()[bh.index]);
        const Real linSpeed = glm::length(w->linearVelocities()[bh.index]);
        std::printf("box on plane: y=%.3f |w|=%.3f |v|=%.3f\n", p.position.y, angSpeed, linSpeed);
        assert(p.position.y > 0.45f && p.position.y < 0.56f);   // resting at half-height
        assert(angSpeed < 0.3f);                                // flat, not tipping
        assert(linSpeed < 0.3f);                                // settled

        BodyDef staticBox;
        staticBox.type = BodyType::Static;
        staticBox.collider.type = ColliderDesc::Type::Box;
        staticBox.collider.box = Box{ Vec3(1.0f, 0.5f, 1.0f) };
        staticBox.position = Vec3(5, 0.5f, 0);
        w->createBody(staticBox);

        BodyDef sph;
        sph.type = BodyType::Dynamic;
        sph.mass = 1.0f;
        sph.collider.type = ColliderDesc::Type::Sphere;
        sph.collider.sphere = Sphere{ 0.5f };
        sph.material.friction = 0.8f;
        sph.material.restitution = 0.0f;
        sph.position = Vec3(5, 2.0f, 0);
        const BodyHandle sh = w->createBody(sph);

        for (int i = 0; i < 300; ++i) w->step(1.0f / 120.0f);
        const auto sp = w->pose(sh);
        std::printf("sphere on box: y=%.3f (expect ~1.5)\n", sp.position.y);
        assert(sp.position.y > 1.4f && sp.position.y < 1.6f);   // box top 1.0 + radius 0.5
    }

    std::printf("physics phase-0 ok\n");
    return 0;
}
