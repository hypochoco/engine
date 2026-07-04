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
#include <cstddef>
#include <utility>
#include <vector>

#include "engine/core/threading/thread_pool.h"
#include "engine/physics/broadphase/aabb.h"
#include "engine/physics/broadphase/sweep_and_prune.h"
#include "engine/physics/broadphase/uniform_grid.h"
#include "engine/physics/collision/box_box.h"
#include "engine/physics/collision/capsule.h"
#include "engine/physics/collision/convex.h"
#include "engine/physics/collision/convex_manifold.h"
#include "engine/physics/collision/primitives.h"
#include "engine/physics/collision/support.h"
#include "engine/physics/dynamics/body.h"
#include "engine/physics/dynamics/integrate.h"
#include "engine/physics/world.h"

namespace engine::physics {
namespace {

constexpr Real kBaumgarte = Real(0.2);    // position-correction fraction
constexpr Real kSlop      = Real(0.005);  // allowed penetration before correction
constexpr Real kMaxCorrection = Real(2);  // cap on Baumgarte correction velocity (m/s)
constexpr Real kAabbMargin = Real(0.01);  // broadphase AABB fattening
constexpr Real kJointBaumgarte = Real(0.2);  // joint position-drift correction fraction (no slop)

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
    uint32_t        collisionCategory = 0x0001;
    uint32_t        collisionMask     = 0xFFFFFFFFu;
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

// Per-candidate-pair narrowphase output (written to its own slot → lock-free parallel fill).
// Holds a small manifold (e.g. box-plane resting yields up to 4 contacts).
struct PairResult {
    int          count = 0;
    Constraint   c[4];
    ContactEvent e[4];
};

// Persistent joint (Phase B). Unlike contacts (rebuilt each step), joints live across steps and
// warm-start from their accumulated impulses. Solved serially in creation order (deterministic).
// Cached fields (rA/rB/K/bias) are recomputed each substep in prepareJoints(); the *Impulse
// accumulators carry across steps for warm-starting.
struct JointData {
    JointType type = JointType::Ball;
    uint32_t  a = 0, b = 0;          // body indices
    Vec3      localAnchorA{0}, localAnchorB{0};
    Vec3      localAxisA{0, 0, 1}, localAxisB{0, 0, 1};
    Quat      refRel{1, 0, 0, 0};    // reference relative orientation qA*·qB captured at creation (Fixed)

    // limits (B2) + actuator (B3)
    bool      enableLimit = false;
    Real      lowerLimit = 0, upperLimit = 0;
    Actuator  actuator{};

    // warm-start accumulators (persist across steps)
    Vec3      pointImpulse{0};        // accumulated point-to-point impulse
    Vec3      angImpulse{0};          // accumulated angular impulse (hinge: in axis-perp plane; fixed: 3-axis)
    Real      limitImpulse = 0;       // accumulated hinge-limit impulse about the axis (one-sided)

    // per-substep cached solve data (recomputed in prepareJoints)
    Vec3      rA{0}, rB{0};           // world anchor offsets from body centers
    Mat3      pointK{Real(0)};        // inverse of the point-to-point effective-mass matrix
    Vec3      pointBias{0};           // Baumgarte velocity bias for the anchor error
    Vec3      axis{0, 0, 1};          // world hinge axis (from body A), for Revolute
    Vec3      t1{1, 0, 0}, t2{0, 1, 0}; // world axes spanning the plane perpendicular to `axis`
    Mat3      angK{Real(0)};          // inverse angular effective-mass matrix (Fixed 3-axis)
    Vec3      angBias{0};             // angular Baumgarte bias
    Real      kt1 = 0, kt2 = 0;       // scalar angular effective masses (hinge 2-axis)
    Real      kAxis = 0;              // scalar effective mass about the hinge axis (limit)
    int       limitState = 0;         // 0 none, +1 at lower (push +), -1 at upper (push -)
    Real      limitBias = 0;          // Baumgarte bias for an active limit

    uint32_t  generation = 0;
    bool      alive = false;
};

class SequentialImpulseWorld final : public PhysicsWorld {
public:
    explicit SequentialImpulseWorld(const WorldDef& def)
        : def_(def), pool_(def.threadPool),
          threshold_(def.parallelThreshold > 0 ? static_cast<size_t>(def.parallelThreshold) : 1) {}

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
        b.collisionCategory = d.collisionCategory;
        b.collisionMask = d.collisionMask;
        b.alive = true;

        const bool dynamic = (d.type == BodyType::Dynamic) && d.mass > kEpsilon;
        b.invMass = dynamic ? Real(1) / d.mass : Real(0);
        if (dynamic && d.collider.type == ColliderDesc::Type::Sphere)
            b.invInertiaLocal = solidSphereInvInertia(d.mass, d.collider.sphere.radius);
        else if (dynamic && d.collider.type == ColliderDesc::Type::Box)
            b.invInertiaLocal = solidBoxInvInertia(d.mass, d.collider.box.halfExtents);
        else if (dynamic && d.collider.type == ColliderDesc::Type::ConvexHull) {
            Vec3 lo(1e30f), hi(-1e30f);
            for (const Vec3& v : d.collider.convexHull.vertices) { lo = glm::min(lo, v); hi = glm::max(hi, v); }
            b.invInertiaLocal = solidBoxInvInertia(d.mass, (hi - lo) * Real(0.5));   // AABB approx
        } else if (dynamic && d.collider.type == ColliderDesc::Type::Capsule)
            b.invInertiaLocal = solidCapsuleInvInertia(d.mass, d.collider.capsule.radius, d.collider.capsule.halfHeight);
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

    JointHandle createJoint(const JointDef& d) override {
        if (!valid(d.a) || !valid(d.b)) return JointHandle{};
        uint32_t index;
        if (!jointFreeList_.empty()) { index = jointFreeList_.back(); jointFreeList_.pop_back(); }
        else { index = static_cast<uint32_t>(joints_.size()); joints_.emplace_back(); }

        JointData& j = joints_[index];
        const uint32_t gen = j.generation;   // preserve generation across slot reuse
        j = JointData{};
        j.generation = gen;
        j.type = d.type;
        j.a = d.a.index;
        j.b = d.b.index;
        j.localAnchorA = d.localAnchorA;
        j.localAnchorB = d.localAnchorB;
        j.localAxisA = glm::normalize(d.localAxisA);
        j.localAxisB = glm::normalize(d.localAxisB);
        j.enableLimit = d.enableLimit;
        j.lowerLimit = d.lowerLimit;
        j.upperLimit = d.upperLimit;
        j.actuator = d.actuator;
        // Reference relative orientation qA*·qB, captured at creation (Fixed keeps this).
        j.refRel = glm::normalize(glm::conjugate(bodies_[j.a].orientation) * bodies_[j.b].orientation);
        j.alive = true;
        return JointHandle{ index, j.generation };
    }

    void destroyJoint(JointHandle h) override {
        if (!jointValid(h)) return;
        joints_[h.index].alive = false;
        ++joints_[h.index].generation;
        jointFreeList_.push_back(h.index);
    }

    void setJointActuator(JointHandle h, const Actuator& a) override {
        if (jointValid(h)) joints_[h.index].actuator = a;
    }
    void setJointTarget(JointHandle h, Real target) override {
        if (jointValid(h)) joints_[h.index].actuator.target = target;
    }
    void setJointTorque(JointHandle h, Real torque) override {
        if (jointValid(h)) joints_[h.index].actuator.torque = torque;
    }
    void setJointTargets(std::span<const Real> targets) override {
        const size_t n = std::min(targets.size(), joints_.size());
        for (size_t i = 0; i < n; ++i) if (joints_[i].alive) joints_[i].actuator.target = targets[i];
    }
    void setJointTorques(std::span<const Real> torques) override {
        const size_t n = std::min(torques.size(), joints_.size());
        for (size_t i = 0; i < n; ++i) if (joints_[i].alive) joints_[i].actuator.torque = torques[i];
    }
    JointState jointState(JointHandle h) const override {
        if (!jointValid(h) || joints_[h.index].type != JointType::Revolute) return {};
        const HingeState hs = hingeState(joints_[h.index]);
        return JointState{ hs.q, hs.qd };
    }
    std::span<const JointState> jointStates() const override { return jointStates_; }

    void setGravity(Vec3 g) override { def_.gravity = g; }

    void step(Real dt) override {
        const int substeps = def_.substeps > 0 ? def_.substeps : 1;
        const Real h = dt / static_cast<Real>(substeps);
        kSubDt_ = h;
        events_.clear();

        for (int s = 0; s < substeps; ++s) {
            // 1. Integrate velocities (gravity) + apply joint actuator torques (B3).
            forEachDynamic([&](BodyData& b) { b.linVel += def_.gravity * h; });
            applyActuators(h);

            // 2. Broadphase + narrowphase -> constraints.
            buildConstraints(h);

            // 2b. Joints: cache per-substep solve data + warm-start from accumulated impulses.
            prepareJoints(h);

            // 3. Sequential-impulse velocity solve. Joints solved serially (creation order,
            //    deterministic) before the graph-colored contacts each iteration.
            for (int it = 0; it < def_.velocityIterations; ++it) {
                solveJoints();
                solveColored();
            }

            // 3b. Optional velocity damping (drag) on dynamic bodies, so undamped DOFs settle.
            if (def_.linearDamping > Real(0) || def_.angularDamping > Real(0)) {
                const Real ld = std::max(Real(0), Real(1) - def_.linearDamping * h);
                const Real ad = std::max(Real(0), Real(1) - def_.angularDamping * h);
                forEachDynamic([&](BodyData& b) { b.linVel *= ld; b.angVel *= ad; });
            }

            // 4. Integrate positions + orientations (Dynamic + Kinematic; Kinematic advances by
            //    its scripted velocity, unaffected by gravity/impulses).
            forEachMoving([&](BodyData& b) {
                b.position += b.linVel * h;
                b.orientation = integrateOrientation(b.orientation, b.angVel, h);
            });
        }

        for (uint32_t i = 0; i < bodies_.size(); ++i) writeOutputs(i);
        writeJointStates();
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

    bool jointValid(JointHandle h) const {
        return h.valid() && h.index < joints_.size() && joints_[h.index].alive
            && joints_[h.index].generation == h.generation;
    }

    // Category/mask collision filter (B4): both directions must pass. Lets an articulation's
    // jointed limbs (same category, masking that category out) skip colliding with each other.
    static bool collisionFilter(const BodyData& A, const BodyData& B) {
        return (A.collisionCategory & B.collisionMask) != 0u
            && (B.collisionCategory & A.collisionMask) != 0u;
    }
    void writeOutputs(uint32_t i) {
        const BodyData& b = bodies_[i];
        poses_[i].position = b.position;
        poses_[i].rotation = b.orientation;
        poses_[i].scale = Vec3(1);
        linVelOut_[i] = b.linVel;
        angVelOut_[i] = b.angVel;
    }

    // Applies `f(body)` to each alive dynamic body, in parallel when the pool is set and the
    // body count is large (writes touch disjoint bodies → deterministic).
    template <class F>
    void forEachDynamic(F&& f) {
        const size_t n = bodies_.size();
        auto one = [&](size_t i) {
            BodyData& b = bodies_[i];
            if (b.alive && b.invMass != Real(0)) f(b);
        };
        if (pool_ && n >= threshold_) pool_->parallelFor(n, one, 1024);
        else for (size_t i = 0; i < n; ++i) one(i);
    }

    // Applies `f(body)` to each alive body that is integrated (Dynamic + Kinematic). Kinematic
    // bodies have invMass==0 (never pushed by the solver) but DO advance by their scripted
    // velocity, so they must be integrated even though forEachDynamic skips them.
    template <class F>
    void forEachMoving(F&& f) {
        const size_t n = bodies_.size();
        auto one = [&](size_t i) {
            BodyData& b = bodies_[i];
            if (b.alive && b.type != BodyType::Static) f(b);
        };
        if (pool_ && n >= threshold_) pool_->parallelFor(n, one, 1024);
        else for (size_t i = 0; i < n; ++i) one(i);
    }

    // Fills constraints_ for the current configuration. normal always points a -> b. Finite
    // colliders (spheres) go through the broadphase; half-space planes are infinite so they're
    // tested against every finite body directly. Narrowphase is parallelized over candidate
    // pairs (each writes its own slot); compaction is serial in candidate order → deterministic.
    void buildConstraints(Real h) {
        constraints_.clear();
        finiteIdx_.clear();
        finiteAabb_.clear();
        planeIdx_.clear();

        for (uint32_t i = 0; i < bodies_.size(); ++i) {
            const BodyData& b = bodies_[i];
            if (!b.alive) continue;
            Aabb box;
            bool finite = true;
            if (b.collider.type == ColliderDesc::Type::Sphere) {
                box = Aabb::fromSphere(b.position, b.collider.sphere.radius);
            } else if (b.collider.type == ColliderDesc::Type::Box) {
                const Mat3 R = glm::mat3_cast(b.orientation);
                Mat3 absR;
                for (int cc = 0; cc < 3; ++cc)
                    for (int rr = 0; rr < 3; ++rr) absR[cc][rr] = std::fabs(R[cc][rr]);
                const Vec3 ext = absR * b.collider.box.halfExtents;
                box = Aabb{ b.position - ext, b.position + ext };
            } else if (b.collider.type == ColliderDesc::Type::ConvexHull) {
                const Mat3 R = glm::mat3_cast(b.orientation);
                Vec3 lo(1e30f), hi(-1e30f);
                for (const Vec3& v : b.collider.convexHull.vertices) {
                    const Vec3 w = b.position + R * v;
                    lo = glm::min(lo, w); hi = glm::max(hi, w);
                }
                box = Aabb{ lo, hi };
            } else if (b.collider.type == ColliderDesc::Type::Capsule) {
                const Vec3 axis = b.orientation * Vec3(0, b.collider.capsule.halfHeight, 0);
                const Vec3 r(b.collider.capsule.radius);
                box = Aabb{ glm::min(b.position - axis, b.position + axis) - r,
                            glm::max(b.position - axis, b.position + axis) + r };
            } else {   // Plane (infinite half-space) — tested against every finite body directly
                planeIdx_.push_back(i);
                finite = false;
            }
            if (!finite) continue;

            if (def_.continuousDetection && b.type != BodyType::Static) {   // swept AABB (CCD)
                const Vec3 d = b.linVel * h;
                box.min += glm::min(d, Vec3(0));
                box.max += glm::max(d, Vec3(0));
            }
            box.expand(kAabbMargin);
            finiteIdx_.push_back(i);
            finiteAabb_.push_back(box);
        }

        if (def_.broadphase == BroadphaseKind::UniformGrid)
            broadphase::uniformGrid(finiteAabb_, pairs_, pool_);
        else
            broadphase::sweepAndPrune(finiteAabb_, pairs_);

        // Candidate body-index pairs: finite-finite (from broadphase) + plane-finite.
        candidatePairs_.clear();
        for (const auto& [pa, pb] : pairs_) {
            const uint32_t i = finiteIdx_[pa];
            const uint32_t j = finiteIdx_[pb];
            if (bodies_[i].invMass == Real(0) && bodies_[j].invMass == Real(0)) continue;
            if (!collisionFilter(bodies_[i], bodies_[j])) continue;
            candidatePairs_.emplace_back(i, j);
        }
        for (uint32_t p : planeIdx_)
            for (uint32_t f : finiteIdx_) {
                if (bodies_[p].invMass == Real(0) && bodies_[f].invMass == Real(0)) continue;
                if (!collisionFilter(bodies_[p], bodies_[f])) continue;
                candidatePairs_.emplace_back(p, f);
            }

        const size_t m = candidatePairs_.size();
        perPair_.assign(m, PairResult{});
        auto doPair = [&](size_t k) {
            narrowphase(candidatePairs_[k].first, candidatePairs_[k].second, h, perPair_[k]);
        };
        if (pool_ && m >= threshold_) pool_->parallelFor(m, doPair, 256);
        else for (size_t k = 0; k < m; ++k) doPair(k);

        for (size_t k = 0; k < m; ++k)
            for (int t = 0; t < perPair_[k].count; ++t) {
                constraints_.push_back(perPair_[k].c[t]);
                events_.push_back(perPair_[k].e[t]);
            }

        colorConstraints();
    }

    // Greedy graph-coloring of the contact graph: two constraints get the same color only if
    // they share no *dynamic* body (static bodies are never written, so they don't conflict).
    // Constraints of one color touch disjoint dynamic bodies → they can be solved in parallel.
    // Coloring + the resulting solve order are deterministic (fixed constraint order), so the
    // serial and pooled paths produce identical results.
    void colorConstraints() {
        const size_t k = constraints_.size();
        constraintColor_.assign(k, 0);
        bodyColorMask_.assign(bodies_.size(), 0);
        numColors_ = 0;

        for (size_t i = 0; i < k; ++i) {
            const Constraint& c = constraints_[i];
            uint64_t forbidden = 0;
            if (bodies_[c.a].invMass != Real(0)) forbidden |= bodyColorMask_[c.a];
            if (bodies_[c.b].invMass != Real(0)) forbidden |= bodyColorMask_[c.b];
            uint32_t color = 0;
            while (color < 63 && (forbidden & (1ull << color))) ++color;   // lowest free color
            constraintColor_[i] = color;
            const uint64_t bit = 1ull << color;
            if (bodies_[c.a].invMass != Real(0)) bodyColorMask_[c.a] |= bit;
            if (bodies_[c.b].invMass != Real(0)) bodyColorMask_[c.b] |= bit;
            numColors_ = std::max(numColors_, color + 1);
        }

        // Counting-sort constraint indices into contiguous per-color runs.
        colorStart_.assign(numColors_ + 1, 0);
        for (uint32_t col : constraintColor_) ++colorStart_[col + 1];
        for (uint32_t c = 0; c < numColors_; ++c) colorStart_[c + 1] += colorStart_[c];
        ordered_.resize(k);
        colorCursor_.assign(colorStart_.begin(), colorStart_.end());
        for (size_t i = 0; i < k; ++i) ordered_[colorCursor_[constraintColor_[i]]++] = static_cast<uint32_t>(i);
    }

    // One Gauss-Seidel sweep: colors sequentially, constraints within a color in parallel
    // (disjoint dynamic bodies → no write conflicts).
    void solveColored() {
        for (uint32_t col = 0; col < numColors_; ++col) {
            const uint32_t begin = colorStart_[col];
            const uint32_t end = colorStart_[col + 1];
            const uint32_t count = end - begin;
            auto solveOne = [&](size_t t) { solveConstraint(constraints_[ordered_[begin + t]]); };
            if (pool_ && count >= threshold_) pool_->parallelFor(count, solveOne, 64);
            else for (uint32_t t = 0; t < count; ++t) solveOne(t);
        }
    }

    SupportShape supportOf(const BodyData& b) const {
        using T = ColliderDesc::Type;
        switch (b.collider.type) {
            case T::Box:  return SupportShape::box(b.position, b.orientation, b.collider.box.halfExtents);
            case T::ConvexHull:
                return SupportShape::hull(b.position, b.orientation,
                                          b.collider.convexHull.vertices.data(),
                                          static_cast<int>(b.collider.convexHull.vertices.size()));
            case T::Capsule:
                return SupportShape::capsule(b.position, b.orientation,
                                             b.collider.capsule.radius, b.collider.capsule.halfHeight);
            default:      return SupportShape::sphere(b.position, b.collider.sphere.radius);
        }
    }

    // World-space vertices of a polytope collider (box: 8 corners; hull: transformed verts).
    void worldVerts(const BodyData& b, std::vector<Vec3>& out) const {
        out.clear();
        if (b.collider.type == ColliderDesc::Type::Box) {
            const Vec3 he = b.collider.box.halfExtents;
            for (int sx = -1; sx <= 1; sx += 2)
                for (int sy = -1; sy <= 1; sy += 2)
                    for (int sz = -1; sz <= 1; sz += 2)
                        out.push_back(b.position + b.orientation * Vec3(sx * he.x, sy * he.y, sz * he.z));
        } else if (b.collider.type == ColliderDesc::Type::ConvexHull) {
            for (const Vec3& v : b.collider.convexHull.vertices)
                out.push_back(b.position + b.orientation * v);
        }
    }

    // Pure narrowphase: computes a small contact manifold for a body pair into `out`. Plane
    // (half-space) pairs use exact/clip tests; sphere-sphere and sphere-box are exact; all other
    // convex pairs (box-box, hull-hull, box-hull, sphere-hull) go through GJK + EPA.
    void narrowphase(uint32_t i, uint32_t j, Real h, PairResult& out) const {
        using T = ColliderDesc::Type;
        out.count = 0;
        const BodyData& A = bodies_[i];
        const BodyData& B = bodies_[j];

        // Speculative margin (CCD): generate contacts up to the distance the pair could close
        // this substep, so a fast body is stopped at the surface instead of tunnelling.
        const Real specMargin = def_.continuousDetection
            ? (glm::length(A.linVel) + glm::length(B.linVel)) * h : Real(0);

        auto add = [&](uint32_t a, uint32_t b, const Contact& c) {
            if (out.count >= 4) return;
            Constraint con;
            con.a = a; con.b = b;
            con.normal = c.normal;
            con.point = c.point;
            con.penetration = -c.separation;
            const Vec3 vrel = relativeVelocity(con);
            const Real vn = glm::dot(vrel, con.normal);
            const Real e = std::min(bodies_[a].material.restitution, bodies_[b].material.restitution);
            con.restitutionBias = (vn < Real(-1)) ? e * (-vn) : Real(0);
            out.c[out.count] = con;
            out.e[out.count] = ContactEvent{
                BodyHandle{ a, bodies_[a].generation }, BodyHandle{ b, bodies_[b].generation },
                con.point, con.normal, c.separation };
            ++out.count;
        };

        // --- half-space plane vs a finite shape (multi-contact for stable resting) ---
        if (A.collider.type == T::Plane || B.collider.type == T::Plane) {
            const uint32_t pi = (A.collider.type == T::Plane) ? i : j;   // plane
            const uint32_t oi = (A.collider.type == T::Plane) ? j : i;   // other
            const BodyData& P = bodies_[pi];
            const BodyData& O = bodies_[oi];
            Contact cs[4];
            int n = 0;
            if (O.collider.type == T::Sphere) {
                if (collide::sphereVsPlane(O.position, O.collider.sphere, P.collider.plane, specMargin, cs[0])) n = 1;
            } else if (O.collider.type == T::Box) {
                n = collide::boxVsPlane(O.position, O.orientation, O.collider.box, P.collider.plane, specMargin, cs);
            } else if (O.collider.type == T::ConvexHull) {
                thread_local std::vector<Vec3> wv;
                wv.clear();
                for (const Vec3& v : O.collider.convexHull.vertices) wv.push_back(O.position + O.orientation * v);
                n = collide::pointsVsPlane(wv.data(), static_cast<int>(wv.size()), P.collider.plane, specMargin, cs);
            } else if (O.collider.type == T::Capsule) {
                n = collide::capsuleVsPlane(O.position, O.orientation, O.collider.capsule, P.collider.plane, specMargin, cs);
            }
            for (int k = 0; k < n; ++k) add(pi, oi, cs[k]);   // normal plane -> other
            return;
        }

        // --- finite vs finite ---
        Contact c;
        if (A.collider.type == T::Sphere && B.collider.type == T::Sphere) {
            if (collide::sphereVsSphere(A.position, A.collider.sphere, B.position, B.collider.sphere, specMargin, c))
                add(i, j, c);
        } else if (A.collider.type == T::Sphere && B.collider.type == T::Box) {
            if (collide::sphereVsBox(B.position, B.orientation, B.collider.box, A.position, A.collider.sphere, specMargin, c))
                add(j, i, c);   // normal box(j) -> sphere(i)
        } else if (A.collider.type == T::Box && B.collider.type == T::Sphere) {
            if (collide::sphereVsBox(A.position, A.orientation, A.collider.box, B.position, B.collider.sphere, specMargin, c))
                add(i, j, c);   // normal box(i) -> sphere(j)
        } else if (A.collider.type == T::Box && B.collider.type == T::Box) {
            Contact cs[4];
            const int n = collide::boxVsBox(A.position, A.orientation, A.collider.box,
                                            B.position, B.orientation, B.collider.box, cs);
            for (int k = 0; k < n; ++k) add(i, j, cs[k]);   // normal A -> B
        } else if (A.collider.type == T::Capsule && B.collider.type == T::Sphere) {
            if (collide::capsuleVsSphere(A.position, A.orientation, A.collider.capsule, B.position, B.collider.sphere, specMargin, c))
                add(i, j, c);   // capsule(i) -> sphere(j)
        } else if (A.collider.type == T::Sphere && B.collider.type == T::Capsule) {
            if (collide::capsuleVsSphere(B.position, B.orientation, B.collider.capsule, A.position, A.collider.sphere, specMargin, c))
                add(j, i, c);   // capsule(j) -> sphere(i)
        } else if (A.collider.type == T::Capsule && B.collider.type == T::Capsule) {
            if (collide::capsuleVsCapsule(A.position, A.orientation, A.collider.capsule,
                                          B.position, B.orientation, B.collider.capsule, specMargin, c))
                add(i, j, c);   // A -> B
        } else if (A.collider.type == T::Capsule && (B.collider.type == T::Box || B.collider.type == T::ConvexHull)) {
            Contact cs[2];
            const int nc = collide::capsuleVsConvex(A.position, A.orientation, A.collider.capsule, supportOf(B), specMargin, cs);
            if (nc > 0) for (int k = 0; k < nc; ++k) add(j, i, cs[k]);       // convex(j) -> capsule(i)
            else if (collide::convexVsConvex(supportOf(A), supportOf(B), c)) add(i, j, c);   // deep overlap fallback
        } else if ((A.collider.type == T::Box || A.collider.type == T::ConvexHull) && B.collider.type == T::Capsule) {
            Contact cs[2];
            const int nc = collide::capsuleVsConvex(B.position, B.orientation, B.collider.capsule, supportOf(A), specMargin, cs);
            if (nc > 0) for (int k = 0; k < nc; ++k) add(i, j, cs[k]);       // convex(i) -> capsule(j)
            else if (collide::convexVsConvex(supportOf(A), supportOf(B), c)) add(i, j, c);
        } else {
            const bool aPoly = (A.collider.type == T::Box || A.collider.type == T::ConvexHull);
            const bool bPoly = (B.collider.type == T::Box || B.collider.type == T::ConvexHull);
            if (aPoly && bPoly) {
                // box-hull / hull-box / hull-hull → EPA normal + polytope face-clip manifold
                thread_local std::vector<Vec3> va, vb;
                worldVerts(A, va);
                worldVerts(B, vb);
                Contact epa;
                if (collide::convexVsConvex(supportOf(A), supportOf(B), epa)) {
                    Contact cs[4];
                    const int nc = collide::polytopeManifold(va, vb, epa, cs);
                    for (int k = 0; k < nc; ++k) add(i, j, cs[k]);   // normal A -> B
                }
            } else {
                // a curved shape is involved (sphere/capsule vs box/hull) → single EPA point
                if (collide::convexVsConvex(supportOf(A), supportOf(B), c)) add(i, j, c);
            }
        }
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
            // Target normal velocity the solver drives toward:
            //  * Overlapping → softened Baumgarte push-out plus any restitution rebound.
            //  * Separated (speculative) → if the pair is bouncing (restitution active), aim for
            //    the rebound velocity directly; adding the negative gap-closing bias here would
            //    brake a fast approach to a near-standstill one substep before impact and
            //    silently cancel restitution. Otherwise permit approach only up to the current
            //    gap this substep (prevents tunnelling without floating).
            Real target;
            if (c.penetration >= Real(0)) {
                const Real baumgarte = std::min((kBaumgarte / kSubDt_) * std::max(c.penetration - kSlop, Real(0)),
                                                kMaxCorrection);
                target = baumgarte + c.restitutionBias;
            } else {
                const Real speculative = c.penetration / kSubDt_;   // = -gap/h (negative)
                target = (c.restitutionBias > Real(0)) ? c.restitutionBias : speculative;
            }
            Real lambda = (kn > kEpsilon) ? (target - vn) / kn : Real(0);

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
        if (A.invMass != Real(0)) { A.linVel -= A.invMass * P; A.angVel -= IinvA * glm::cross(rA, P); }
        if (B.invMass != Real(0)) { B.linVel += B.invMass * P; B.angVel += IinvB * glm::cross(rB, P); }
    }

    // --- joint helpers (Phase B) ---

    // Cross-product matrix: skew(v) * x == cross(v, x). glm is column-major (m[col][row]).
    static Mat3 skew(const Vec3& v) {
        Mat3 s(Real(0));
        s[0][0] = 0;     s[1][0] = -v.z;  s[2][0] =  v.y;
        s[0][1] =  v.z;  s[1][1] = 0;     s[2][1] = -v.x;
        s[0][2] = -v.y;  s[1][2] =  v.x;  s[2][2] = 0;
        return s;
    }

    // Orthonormal basis (t1, t2) spanning the plane perpendicular to unit vector n.
    static void basisPerp(const Vec3& n, Vec3& t1, Vec3& t2) {
        const Vec3 ref = (std::fabs(n.x) <= std::fabs(n.y) && std::fabs(n.x) <= std::fabs(n.z))
            ? Vec3(1, 0, 0)
            : (std::fabs(n.y) <= std::fabs(n.z) ? Vec3(0, 1, 0) : Vec3(0, 0, 1));
        t1 = glm::normalize(glm::cross(n, ref));
        t2 = glm::cross(n, t1);
    }

    static void applyAngularImpulse(BodyData& A, BodyData& B, const Mat3& IinvA,
                                    const Mat3& IinvB, const Vec3& L) {
        if (A.invMass != Real(0)) A.angVel -= IinvA * L;
        if (B.invMass != Real(0)) B.angVel += IinvB * L;
    }

    // Hinge angle (about the axis, from the creation-time reference) + rate + world axis.
    struct HingeState { Vec3 axisW; Real q; Real qd; };
    HingeState hingeState(const JointData& j) const {
        const BodyData& A = bodies_[j.a];
        const BodyData& B = bodies_[j.b];
        HingeState hs;
        hs.axisW = glm::mat3_cast(A.orientation) * j.localAxisA;
        const Quat qRel = glm::normalize(glm::conjugate(A.orientation) * B.orientation);
        const Quat qErr = glm::normalize(qRel * glm::conjugate(j.refRel));   // drift, A-local
        const Vec3 v(qErr.x, qErr.y, qErr.z);
        hs.q  = Real(2) * std::atan2(glm::dot(v, j.localAxisA), qErr.w);       // twist about axis
        hs.qd = glm::dot(B.angVel - A.angVel, hs.axisW);
        return hs;
    }

    // Actuators (B3): apply each Revolute joint's actuator torque about its hinge axis as an
    // external angular impulse (RL torque semantics), equal-and-opposite on the two bodies. Runs
    // in the velocity-integration phase (alongside gravity), before the constraint solve.
    void applyActuators(Real h) {
        for (JointData& j : joints_) {
            if (!j.alive) continue;
            if (j.actuator.mode == ActuatorMode::None) continue;
            if (j.type == JointType::Fixed) continue;   // no free DOF to actuate
            BodyData& A = bodies_[j.a];
            BodyData& B = bodies_[j.b];
            if (A.invMass == Real(0) && B.invMass == Real(0)) continue;
            const Mat3 IinvA = worldInvInertia(A.orientation, A.invInertiaLocal);
            const Mat3 IinvB = worldInvInertia(B.orientation, B.invInertiaLocal);

            if (j.type == JointType::Revolute) {
                const HingeState hs = hingeState(j);
                Real tau = (j.actuator.mode == ActuatorMode::Torque)
                    ? j.actuator.torque
                    : j.actuator.kp * (j.actuator.target - hs.q)
                    + j.actuator.kd * (j.actuator.targetVel - hs.qd);
                if (j.actuator.maxTorque > Real(0))
                    tau = std::clamp(tau, -j.actuator.maxTorque, j.actuator.maxTorque);
                applyAngularImpulse(A, B, IinvA, IinvB, (tau * h) * hs.axisW);   // +τ→B, -τ→A
            } else {   // Ball: 3-DOF spherical actuation
                Vec3 tau3;
                if (j.actuator.mode == ActuatorMode::Torque) {
                    tau3 = j.actuator.ballTorque;
                } else {   // PDTarget toward a desired relative orientation
                    const Quat qRel = glm::normalize(glm::conjugate(A.orientation) * B.orientation);
                    const Quat qErr = glm::normalize(j.actuator.ballTarget * glm::conjugate(qRel));
                    const Vec3 eWorld = glm::mat3_cast(A.orientation) * so3LogMap(qErr);   // A-local → world
                    tau3 = j.actuator.kp * eWorld - j.actuator.kd * (B.angVel - A.angVel);
                }
                if (j.actuator.maxTorque > Real(0)) {
                    const Real len = glm::length(tau3);
                    if (len > j.actuator.maxTorque) tau3 *= j.actuator.maxTorque / len;
                }
                applyAngularImpulse(A, B, IinvA, IinvB, tau3 * h);
            }
        }
    }

    // Refresh the bulk joint-state readback (observation). Indexed by joint slot; called after
    // integration each step. Allocation-free once sized.
    void writeJointStates() {
        jointStates_.assign(joints_.size(), JointState{});
        for (size_t i = 0; i < joints_.size(); ++i) {
            if (joints_[i].alive && joints_[i].type == JointType::Revolute) {
                const HingeState hs = hingeState(joints_[i]);
                jointStates_[i] = JointState{ hs.q, hs.qd };
            }
        }
    }

    // Cache per-substep solve data (anchor offsets, effective-mass matrices, Baumgarte bias) and
    // warm-start from the accumulated impulses. Runs once per substep before the iteration loop.
    void prepareJoints(Real h) {
        const Real invH = (h > kEpsilon) ? Real(1) / h : Real(0);
        for (JointData& j : joints_) {
            if (!j.alive) continue;
            BodyData& A = bodies_[j.a];
            BodyData& B = bodies_[j.b];
            if (A.invMass == Real(0) && B.invMass == Real(0)) continue;

            const Mat3 RA = glm::mat3_cast(A.orientation);
            const Mat3 RB = glm::mat3_cast(B.orientation);
            const Mat3 IinvA = worldInvInertia(A.orientation, A.invInertiaLocal);
            const Mat3 IinvB = worldInvInertia(B.orientation, B.invInertiaLocal);

            // Point-to-point: K = (imA+imB)I - skew(rA)·IinvA·skew(rA) - skew(rB)·IinvB·skew(rB).
            j.rA = RA * j.localAnchorA;
            j.rB = RB * j.localAnchorB;
            const Mat3 sA = skew(j.rA);
            const Mat3 sB = skew(j.rB);
            const Mat3 K = Mat3(A.invMass + B.invMass) - sA * IinvA * sA - sB * IinvB * sB;
            j.pointK = glm::inverse(K);
            const Vec3 anchorA = A.position + j.rA;
            const Vec3 anchorB = B.position + j.rB;
            j.pointBias = (kJointBaumgarte * invH) * (anchorB - anchorA);   // drive anchors together

            applyImpulse(A, B, IinvA, IinvB, j.rA, j.rB, j.pointImpulse);   // warm start (point)

            if (j.type == JointType::Revolute) {
                const Vec3 aA = RA * j.localAxisA;
                const Vec3 aB = RB * j.localAxisB;
                j.axis = aA;
                basisPerp(aA, j.t1, j.t2);
                const Mat3 Isum = IinvA + IinvB;
                j.kt1 = glm::dot(j.t1, Isum * j.t1);
                j.kt2 = glm::dot(j.t2, Isum * j.t2);
                j.angBias = (kJointBaumgarte * invH) * glm::cross(aA, aB);   // axis misalignment
                applyAngularImpulse(A, B, IinvA, IinvB, j.angImpulse);       // warm start (angular)

                // hinge limit (B2): one-sided constraint about the axis when past a limit.
                j.limitState = 0;
                if (j.enableLimit && j.lowerLimit <= j.upperLimit) {
                    j.kAxis = glm::dot(aA, Isum * aA);
                    const Real q = hingeState(j).q;
                    if (q <= j.lowerLimit) {
                        j.limitState = +1;
                        j.limitBias = (kJointBaumgarte * invH) * (q - j.lowerLimit);   // < 0 → push up
                    } else if (q >= j.upperLimit) {
                        j.limitState = -1;
                        j.limitBias = (kJointBaumgarte * invH) * (q - j.upperLimit);   // > 0 → push down
                    }
                }
                if (j.limitState != 0) applyAngularImpulse(A, B, IinvA, IinvB, j.limitImpulse * j.axis);
                else                   j.limitImpulse = 0;
            } else if (j.type == JointType::Fixed) {
                j.angK = glm::inverse(IinvA + IinvB);
                const Quat qRel = glm::normalize(glm::conjugate(A.orientation) * B.orientation);
                const Quat qErr = glm::normalize(qRel * glm::conjugate(j.refRel));  // drift, A-local
                j.angBias = (kJointBaumgarte * invH) * (RA * so3LogMap(qErr));       // -> world
                applyAngularImpulse(A, B, IinvA, IinvB, j.angImpulse);
            } else {
                j.angImpulse = Vec3(0);
            }
        }
    }

    // One serial Gauss-Seidel sweep over joints (creation order → deterministic). Point-to-point
    // then the type-specific angular part; each accumulates into the warm-start impulse.
    void solveJoints() {
        for (JointData& j : joints_) {
            if (!j.alive) continue;
            BodyData& A = bodies_[j.a];
            BodyData& B = bodies_[j.b];
            if (A.invMass == Real(0) && B.invMass == Real(0)) continue;
            const Mat3 IinvA = worldInvInertia(A.orientation, A.invInertiaLocal);
            const Mat3 IinvB = worldInvInertia(B.orientation, B.invInertiaLocal);

            // Angular part first, then point-to-point last, so the anchor (linear) constraint is
            // satisfied at the end of each iteration (angular impulses perturb the anchor velocity
            // via ω, so solving them first and the point last converges far more stably).
            if (j.type == JointType::Revolute) {
                if (j.kt1 > kEpsilon) {
                    const Vec3 wrel = B.angVel - A.angVel;
                    const Real L = -(glm::dot(wrel, j.t1) + glm::dot(j.angBias, j.t1)) / j.kt1;
                    const Vec3 imp = L * j.t1;
                    j.angImpulse += imp;
                    applyAngularImpulse(A, B, IinvA, IinvB, imp);
                }
                if (j.kt2 > kEpsilon) {
                    const Vec3 wrel = B.angVel - A.angVel;
                    const Real L = -(glm::dot(wrel, j.t2) + glm::dot(j.angBias, j.t2)) / j.kt2;
                    const Vec3 imp = L * j.t2;
                    j.angImpulse += imp;
                    applyAngularImpulse(A, B, IinvA, IinvB, imp);
                }
                // hinge limit (B2): one-sided impulse about the axis (push back into range only).
                if (j.limitState != 0 && j.kAxis > kEpsilon) {
                    const Vec3 wrel = B.angVel - A.angVel;
                    const Real wAxis = glm::dot(wrel, j.axis);
                    Real dL = -(wAxis + j.limitBias) / j.kAxis;
                    const Real old = j.limitImpulse;
                    j.limitImpulse = (j.limitState > 0) ? std::max(old + dL, Real(0))
                                                        : std::min(old + dL, Real(0));
                    dL = j.limitImpulse - old;
                    applyAngularImpulse(A, B, IinvA, IinvB, dL * j.axis);
                }
            } else if (j.type == JointType::Fixed) {
                const Vec3 wrel = B.angVel - A.angVel;
                const Vec3 imp = j.angK * (-(wrel + j.angBias));
                j.angImpulse += imp;
                applyAngularImpulse(A, B, IinvA, IinvB, imp);
            }

            // point-to-point (last)
            {
                const Vec3 vrel = (B.linVel + glm::cross(B.angVel, j.rB))
                                - (A.linVel + glm::cross(A.angVel, j.rA));
                const Vec3 dP = j.pointK * (-(vrel + j.pointBias));
                j.pointImpulse += dP;
                applyImpulse(A, B, IinvA, IinvB, j.rA, j.rB, dP);
            }
        }
    }

    WorldDef def_;
    Real     kSubDt_ = Real(1) / Real(60);   // set per step for the Baumgarte term
    core::ThreadPool* pool_ = nullptr;
    size_t   threshold_ = 4096;
    std::vector<BodyData>         bodies_;
    std::vector<uint32_t>         freeList_;
    std::vector<Constraint>       constraints_;
    std::vector<JointData>        joints_;        // persistent (Phase B)
    std::vector<uint32_t>         jointFreeList_;
    std::vector<JointState>       jointStates_;   // bulk readback (B3), indexed by joint slot
    std::vector<engine::Transform> poses_;
    std::vector<Vec3>             linVelOut_;
    std::vector<Vec3>             angVelOut_;
    std::vector<ContactEvent>     events_;

    // broadphase + narrowphase scratch (reused across steps)
    std::vector<uint32_t>            finiteIdx_;
    std::vector<Aabb>                finiteAabb_;
    std::vector<uint32_t>            planeIdx_;
    std::vector<broadphase::Pair>    pairs_;
    std::vector<std::pair<uint32_t, uint32_t>> candidatePairs_;
    std::vector<PairResult>          perPair_;

    // graph-coloring scratch for the parallel solver
    std::vector<uint32_t>            constraintColor_;
    std::vector<uint64_t>            bodyColorMask_;
    std::vector<uint32_t>            ordered_;      // constraint indices grouped by color
    std::vector<uint32_t>            colorStart_;   // per-color offsets into ordered_
    std::vector<uint32_t>            colorCursor_;  // counting-sort scratch
    uint32_t                         numColors_ = 0;

public:
    void setSubDt(Real h) { kSubDt_ = h; }
};

} // namespace

std::unique_ptr<PhysicsWorld> createPhysicsWorld(Backend backend, const WorldDef& def) {
    (void)backend;   // only Realtime for now
    return std::make_unique<SequentialImpulseWorld>(def);
}

} // namespace engine::physics
