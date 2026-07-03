//
//  sequential_impulse_world.cpp
//  engine::physics / backends / realtime
//
//  Realtime backend (physics plan §7, Q5): semi-implicit (symplectic) Euler + a sequential-
//  impulse (PGS) contact solver with restitution, Coulomb friction, and Baumgarte position
//  correction, run over configurable substeps × velocity iterations. Friction impulses apply
//  at the contact point, producing torque via r × P — this is what makes a sphere truly ROLL
//  rather than slide. Broadphase is brute-force O(n²) for Phase 1 (SAP/BVH is Phase 2).
//
//  The concrete type is private to this TU; only createPhysicsWorld() is exported, so the
//  hot loops stay non-virtual (plan §1).
//

#include <algorithm>
#include <cmath>
#include <vector>

#include "engine/physics/collision/primitives.h"
#include "engine/physics/dynamics/body.h"
#include "engine/physics/dynamics/integrate.h"
#include "engine/physics/world.h"

namespace engine::physics {
namespace {

constexpr Real kBaumgarte = Real(0.2);    // position-correction fraction
constexpr Real kSlop      = Real(0.005);  // allowed penetration before correction

struct BodyData {
    Vec3            position{0};
    Quat            orientation{1, 0, 0, 0};
    Vec3            linVel{0};
    Vec3            angVel{0};
    Real            invMass = Real(0);
    Mat3            invInertiaLocal{Real(0)};
    PhysicsMaterial material{};
    BodyType        type = BodyType::Static;
    ColliderDesc    collider{};
    uint32_t        generation = 0;
    bool            alive = false;
};

struct Constraint {
    uint32_t a = 0, b = 0;       // body indices; normal points from a -> b
    Vec3     normal{0, 1, 0};
    Vec3     point{0};
    Real     penetration = 0;    // > 0 when overlapping
    Real     restitutionBias = 0;
    Real     normalImpulse = 0;  // accumulated (within step)
    Real     tangentImpulse = 0;
};

class SequentialImpulseWorld final : public PhysicsWorld {
public:
    explicit SequentialImpulseWorld(const WorldDef& def) : def_(def) {}

    BodyHandle createBody(const BodyDef& d) override {
        uint32_t index;
        if (!freeList_.empty()) { index = freeList_.back(); freeList_.pop_back(); }
        else {
            index = static_cast<uint32_t>(bodies_.size());
            bodies_.emplace_back();
            poses_.emplace_back();
            linVelOut_.emplace_back(0);
            angVelOut_.emplace_back(0);
        }
        BodyData& b = bodies_[index];
        b = BodyData{};
        b.position = d.position;
        b.orientation = d.orientation;
        b.linVel = d.linearVelocity;
        b.angVel = d.angularVelocity;
        b.material = d.material;
        b.type = d.type;
        b.collider = d.collider;
        b.alive = true;

        const bool dynamic = (d.type == BodyType::Dynamic) && d.mass > kEpsilon;
        b.invMass = dynamic ? Real(1) / d.mass : Real(0);
        if (dynamic && d.collider.type == ColliderDesc::Type::Sphere)
            b.invInertiaLocal = solidSphereInvInertia(d.mass, d.collider.sphere.radius);
        else
            b.invInertiaLocal = Mat3(Real(0));

        writeOutputs(index);
        return BodyHandle{ index, b.generation };
    }

    void destroyBody(BodyHandle h) override {
        if (!valid(h)) return;
        bodies_[h.index].alive = false;
        ++bodies_[h.index].generation;
        freeList_.push_back(h.index);
    }

    void setGravity(Vec3 g) override { def_.gravity = g; }

    void step(Real dt) override {
        const int substeps = def_.substeps > 0 ? def_.substeps : 1;
        const Real h = dt / static_cast<Real>(substeps);
        kSubDt_ = h;
        events_.clear();

        for (int s = 0; s < substeps; ++s) {
            // 1. Integrate velocities (gravity).
            for (BodyData& b : bodies_) {
                if (!b.alive || b.invMass == Real(0)) continue;
                b.linVel += def_.gravity * h;
            }

            // 2. Broadphase + narrowphase (brute force) -> constraints.
            buildConstraints(h);

            // 3. Sequential-impulse velocity solve.
            for (int it = 0; it < def_.velocityIterations; ++it)
                for (Constraint& c : constraints_) solveConstraint(c);

            // 4. Integrate positions + orientations.
            for (BodyData& b : bodies_) {
                if (!b.alive || b.invMass == Real(0)) continue;
                b.position += b.linVel * h;
                b.orientation = integrateOrientation(b.orientation, b.angVel, h);
            }
        }

        for (uint32_t i = 0; i < bodies_.size(); ++i) writeOutputs(i);
    }

    std::span<const engine::Transform> poses() const override { return poses_; }
    std::span<const Vec3> linearVelocities() const override { return linVelOut_; }
    std::span<const Vec3> angularVelocities() const override { return angVelOut_; }

    engine::Transform pose(BodyHandle h) const override {
        if (!valid(h)) return {};
        return poses_[h.index];
    }

    std::span<const ContactEvent> contacts() const override { return events_; }

private:
    bool valid(BodyHandle h) const {
        return h.valid() && h.index < bodies_.size() && bodies_[h.index].alive
            && bodies_[h.index].generation == h.generation;
    }

    void writeOutputs(uint32_t i) {
        const BodyData& b = bodies_[i];
        poses_[i].position = b.position;
        poses_[i].rotation = b.orientation;
        poses_[i].scale = Vec3(1);
        linVelOut_[i] = b.linVel;
        angVelOut_[i] = b.angVel;
    }

    // Fills constraints_ for the current configuration. normal always points a -> b.
    void buildConstraints(Real h) {
        constraints_.clear();
        const uint32_t n = static_cast<uint32_t>(bodies_.size());
        for (uint32_t i = 0; i < n; ++i) {
            if (!bodies_[i].alive) continue;
            for (uint32_t j = i + 1; j < n; ++j) {
                if (!bodies_[j].alive) continue;
                if (bodies_[i].invMass == Real(0) && bodies_[j].invMass == Real(0)) continue;
                narrowphase(i, j, h);
            }
        }
    }

    void narrowphase(uint32_t i, uint32_t j, Real h) {
        const BodyData& A = bodies_[i];
        const BodyData& B = bodies_[j];
        using T = ColliderDesc::Type;

        Contact c;
        uint32_t a = i, b = j;
        bool hit = false;

        if (A.collider.type == T::Sphere && B.collider.type == T::Sphere) {
            hit = collide::sphereVsSphere(A.position, A.collider.sphere,
                                          B.position, B.collider.sphere, Real(0), c);
            a = i; b = j;   // normal already A -> B
        } else if (A.collider.type == T::Sphere && B.collider.type == T::Plane) {
            hit = collide::sphereVsPlane(A.position, A.collider.sphere, B.collider.plane, Real(0), c);
            a = j; b = i;   // collide normal points plane -> sphere, i.e. b(plane)->a(sphere)
        } else if (A.collider.type == T::Plane && B.collider.type == T::Sphere) {
            hit = collide::sphereVsPlane(B.position, B.collider.sphere, A.collider.plane, Real(0), c);
            a = i; b = j;   // normal plane(A) -> sphere(B)
        }
        if (!hit) return;

        Constraint con;
        con.a = a; con.b = b;
        con.normal = c.normal;
        con.point = c.point;
        con.penetration = -c.separation;   // separation<0 => penetrating

        // Restitution from the pre-solve approach velocity.
        const Vec3 vrel = relativeVelocity(con);
        const Real vn = glm::dot(vrel, con.normal);
        const Real e = std::min(bodies_[a].material.restitution, bodies_[b].material.restitution);
        con.restitutionBias = (vn < Real(-1)) ? e * (-vn) : Real(0);  // ignore tiny approach
        (void)h;
        constraints_.push_back(con);

        events_.push_back(ContactEvent{
            BodyHandle{ a, bodies_[a].generation }, BodyHandle{ b, bodies_[b].generation },
            con.point, con.normal, c.separation });
    }

    Vec3 relativeVelocity(const Constraint& c) const {
        const BodyData& A = bodies_[c.a];
        const BodyData& B = bodies_[c.b];
        const Vec3 rA = c.point - A.position;
        const Vec3 rB = c.point - B.position;
        return (B.linVel + glm::cross(B.angVel, rB)) - (A.linVel + glm::cross(A.angVel, rA));
    }

    void solveConstraint(Constraint& c) {
        BodyData& A = bodies_[c.a];
        BodyData& B = bodies_[c.b];
        const Vec3 rA = c.point - A.position;
        const Vec3 rB = c.point - B.position;
        const Mat3 IinvA = worldInvInertia(A.orientation, A.invInertiaLocal);
        const Mat3 IinvB = worldInvInertia(B.orientation, B.invInertiaLocal);
        const Vec3& n = c.normal;

        // --- normal impulse ---
        {
            const Vec3 vrel = (B.linVel + glm::cross(B.angVel, rB))
                            - (A.linVel + glm::cross(A.angVel, rA));
            const Real vn = glm::dot(vrel, n);
            const Real kn = effectiveMass(A, B, IinvA, IinvB, rA, rB, n);
            const Real bias = (kBaumgarte / kSubDt_) * std::max(c.penetration - kSlop, Real(0));
            Real lambda = (kn > kEpsilon) ? (bias + c.restitutionBias - vn) / kn : Real(0);

            const Real oldImpulse = c.normalImpulse;
            c.normalImpulse = std::max(oldImpulse + lambda, Real(0));
            lambda = c.normalImpulse - oldImpulse;
            applyImpulse(A, B, IinvA, IinvB, rA, rB, lambda * n);
        }

        // --- friction impulse ---
        {
            const Vec3 vrel = (B.linVel + glm::cross(B.angVel, rB))
                            - (A.linVel + glm::cross(A.angVel, rA));
            Vec3 vt = vrel - glm::dot(vrel, n) * n;
            const Real vtLen = glm::length(vt);
            if (vtLen > kEpsilon) {
                const Vec3 t = vt / vtLen;
                const Real kt = effectiveMass(A, B, IinvA, IinvB, rA, rB, t);
                Real lambdaT = (kt > kEpsilon) ? -glm::dot(vrel, t) / kt : Real(0);

                const Real mu = std::sqrt(A.material.friction * B.material.friction);
                const Real maxF = mu * c.normalImpulse;
                const Real oldT = c.tangentImpulse;
                c.tangentImpulse = std::clamp(oldT + lambdaT, -maxF, maxF);
                lambdaT = c.tangentImpulse - oldT;
                applyImpulse(A, B, IinvA, IinvB, rA, rB, lambdaT * t);
            }
        }
    }

    static Real effectiveMass(const BodyData& A, const BodyData& B, const Mat3& IinvA,
                              const Mat3& IinvB, const Vec3& rA, const Vec3& rB, const Vec3& dir) {
        const Vec3 rnA = glm::cross(rA, dir);
        const Vec3 rnB = glm::cross(rB, dir);
        return A.invMass + B.invMass
             + glm::dot(dir, glm::cross(IinvA * rnA, rA))
             + glm::dot(dir, glm::cross(IinvB * rnB, rB));
    }

    static void applyImpulse(BodyData& A, BodyData& B, const Mat3& IinvA, const Mat3& IinvB,
                             const Vec3& rA, const Vec3& rB, const Vec3& P) {
        A.linVel -= A.invMass * P;
        A.angVel -= IinvA * glm::cross(rA, P);
        B.linVel += B.invMass * P;
        B.angVel += IinvB * glm::cross(rB, P);
    }

    WorldDef def_;
    Real     kSubDt_ = Real(1) / Real(60);   // set per step for the Baumgarte term
    std::vector<BodyData>         bodies_;
    std::vector<uint32_t>         freeList_;
    std::vector<Constraint>       constraints_;
    std::vector<engine::Transform> poses_;
    std::vector<Vec3>             linVelOut_;
    std::vector<Vec3>             angVelOut_;
    std::vector<ContactEvent>     events_;

public:
    void setSubDt(Real h) { kSubDt_ = h; }
};

} // namespace

std::unique_ptr<PhysicsWorld> createPhysicsWorld(Backend backend, const WorldDef& def) {
    (void)backend;   // only Realtime for now
    return std::make_unique<SequentialImpulseWorld>(def);
}

} // namespace engine::physics
