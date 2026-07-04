//
//  featherstone_world.cpp
//  engine::physics / backends / reduced
//
//  Reduced-coordinate PhysicsWorld backend (Phase E). Represents an articulated body by its
//  generalized coordinates q (joint angles + an optional floating 6-DOF base) and steps it with
//  the Articulated-Body Algorithm (ABA), O(n) over the limb tree. Behind the same PhysicsWorld
//  interface as the maximal (Realtime) backend. See notes/investigations/
//  2026-07-04-reduced-coordinate-backend.md.
//
//  Scope now (E0): Revolute (1-DOF) / Fixed (0-DOF) joints, fixed OR floating base, gravity +
//  joint-torque/PD actuators, no contacts. Ball (3-DOF) is E2; contacts are E1.
//
//  Spatial algebra: angular-first 6-vectors [w; v], full 6x6 matrices for transforms/inertias
//  (n is small; explicit matrices are far less bug-prone than block formulas). Link frame at COM.
//  Gravity enters as an explicit per-link force (works uniformly for fixed + floating bases).
//

#include <array>
#include <cmath>
#include <vector>

#include "engine/physics/dynamics/body.h"
#include "engine/physics/world.h"

#include "../backends_internal.h"

namespace engine::physics {
namespace {

// ------------------------------------------------------------------ spatial algebra (6D) -------
struct Vec6 { Real d[6]; };

inline Vec6 v6(const Vec3& ang, const Vec3& lin) {
    return { ang.x, ang.y, ang.z, lin.x, lin.y, lin.z };
}
inline Vec3 v6ang(const Vec6& v) { return Vec3(v.d[0], v.d[1], v.d[2]); }
inline Vec3 v6lin(const Vec6& v) { return Vec3(v.d[3], v.d[4], v.d[5]); }

struct Mat6 {
    Real m[6][6];
    static Mat6 zero() { Mat6 r{}; for (auto& row : r.m) for (Real& x : row) x = 0; return r; }
};

// glm mat3 is column-major: math element (i,j) = M[j][i].
inline Real at3(const Mat3& M, int i, int j) { return M[j][i]; }

inline void setBlock(Mat6& M, int r0, int c0, const Mat3& B, bool neg = false) {
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            M.m[r0 + i][c0 + j] = neg ? -at3(B, i, j) : at3(B, i, j);
}

inline Mat3 skew(const Vec3& v) {
    return Mat3(0, v.z, -v.y,  -v.z, 0, v.x,  v.y, -v.x, 0);   // glm columns of math [[0,-z,y],...]
}

// Plücker MOTION transform from rotation E (parent→child vectors) and translation r (child origin
// in parent frame): X = [[E,0],[-E*skew(r), E]].
inline Mat6 plux(const Mat3& E, const Vec3& r) {
    Mat6 X = Mat6::zero();
    setBlock(X, 0, 0, E);
    setBlock(X, 3, 3, E);
    setBlock(X, 3, 0, E * skew(r), /*neg=*/true);
    return X;
}

inline Mat6 spatialInertia(Real mass, const Mat3& Ic) {   // COM at frame origin
    Mat6 I = Mat6::zero();
    setBlock(I, 0, 0, Ic);
    I.m[3][3] = mass; I.m[4][4] = mass; I.m[5][5] = mass;
    return I;
}

inline Mat6 crm(const Vec6& v) {   // motion cross [[skew(w),0],[skew(vl),skew(w)]]
    const Vec3 w = v6ang(v), vl = v6lin(v);
    Mat6 M = Mat6::zero();
    setBlock(M, 0, 0, skew(w));
    setBlock(M, 3, 3, skew(w));
    setBlock(M, 3, 0, skew(vl));
    return M;
}

inline Vec6 mul(const Mat6& A, const Vec6& x) {
    Vec6 r{};
    for (int i = 0; i < 6; ++i) { Real s = 0; for (int j = 0; j < 6; ++j) s += A.m[i][j] * x.d[j]; r.d[i] = s; }
    return r;
}
inline Mat6 mul(const Mat6& A, const Mat6& B) {
    Mat6 r = Mat6::zero();
    for (int i = 0; i < 6; ++i)
        for (int k = 0; k < 6; ++k) {
            const Real a = A.m[i][k];
            if (a != 0) for (int j = 0; j < 6; ++j) r.m[i][j] += a * B.m[k][j];
        }
    return r;
}
inline Mat6 transpose(const Mat6& A) {
    Mat6 r = Mat6::zero();
    for (int i = 0; i < 6; ++i) for (int j = 0; j < 6; ++j) r.m[i][j] = A.m[j][i];
    return r;
}
inline Mat6 addM(const Mat6& A, const Mat6& B) {
    Mat6 r = Mat6::zero();
    for (int i = 0; i < 6; ++i) for (int j = 0; j < 6; ++j) r.m[i][j] = A.m[i][j] + B.m[i][j];
    return r;
}
inline Mat6 subM(const Mat6& A, const Mat6& B) {
    Mat6 r = Mat6::zero();
    for (int i = 0; i < 6; ++i) for (int j = 0; j < 6; ++j) r.m[i][j] = A.m[i][j] - B.m[i][j];
    return r;
}
inline Vec6 addV(const Vec6& a, const Vec6& b) {
    Vec6 r{}; for (int i = 0; i < 6; ++i) r.d[i] = a.d[i] + b.d[i]; return r;
}
inline Vec6 subV(const Vec6& a, const Vec6& b) {
    Vec6 r{}; for (int i = 0; i < 6; ++i) r.d[i] = a.d[i] - b.d[i]; return r;
}
inline Vec6 scale(const Vec6& a, Real s) {
    Vec6 r{}; for (int i = 0; i < 6; ++i) r.d[i] = a.d[i] * s; return r;
}
inline Real dot(const Vec6& a, const Vec6& b) {
    Real s = 0; for (int i = 0; i < 6; ++i) s += a.d[i] * b.d[i]; return s;
}
inline Mat6 outerOverD(const Vec6& u, Real invD) {
    Mat6 r = Mat6::zero();
    for (int i = 0; i < 6; ++i) for (int j = 0; j < 6; ++j) r.m[i][j] = u.d[i] * u.d[j] * invD;
    return r;
}

// Invert a dense n×n matrix (row-major) via Gauss-Jordan with partial pivoting. Returns false if
// singular. Used to form H⁻¹ for the generalized-coordinate contact solve (n is small).
bool invertDense(const std::vector<Real>& A, int n, std::vector<Real>& inv) {
    std::vector<Real> M(A);
    inv.assign(static_cast<size_t>(n) * n, Real(0));
    for (int i = 0; i < n; ++i) inv[i * n + i] = Real(1);
    for (int col = 0; col < n; ++col) {
        int piv = col;
        for (int r = col + 1; r < n; ++r) if (std::fabs(M[r * n + col]) > std::fabs(M[piv * n + col])) piv = r;
        if (std::fabs(M[piv * n + col]) < Real(1e-12)) return false;
        if (piv != col)
            for (int j = 0; j < n; ++j) { std::swap(M[col * n + j], M[piv * n + j]); std::swap(inv[col * n + j], inv[piv * n + j]); }
        const Real d = M[col * n + col];
        for (int j = 0; j < n; ++j) { M[col * n + j] /= d; inv[col * n + j] /= d; }
        for (int r = 0; r < n; ++r) {
            if (r == col) continue;
            const Real f = M[r * n + col];
            if (f == 0) continue;
            for (int j = 0; j < n; ++j) { M[r * n + j] -= f * M[col * n + j]; inv[r * n + j] -= f * inv[col * n + j]; }
        }
    }
    return true;
}

// Solve A x = b for a 6x6 A (Gaussian elimination, partial pivot). Used for the floating base.
Vec6 solve6(const Mat6& A, const Vec6& b) {
    Real M[6][7];
    for (int i = 0; i < 6; ++i) { for (int j = 0; j < 6; ++j) M[i][j] = A.m[i][j]; M[i][6] = b.d[i]; }
    for (int col = 0; col < 6; ++col) {
        int piv = col;
        for (int r = col + 1; r < 6; ++r) if (std::fabs(M[r][col]) > std::fabs(M[piv][col])) piv = r;
        if (piv != col) for (int j = 0; j < 7; ++j) std::swap(M[col][j], M[piv][j]);
        const Real d = M[col][col];
        if (std::fabs(d) < Real(1e-12)) continue;
        for (int r = 0; r < 6; ++r) {
            if (r == col) continue;
            const Real f = M[r][col] / d;
            for (int j = col; j < 7; ++j) M[r][j] -= f * M[col][j];
        }
    }
    Vec6 x{};
    for (int i = 0; i < 6; ++i) x.d[i] = (std::fabs(M[i][i]) > Real(1e-12)) ? M[i][6] / M[i][i] : Real(0);
    return x;
}

// ------------------------------------------------------------------ forward inertia (COM) ------
Mat3 colliderInertia(const ColliderDesc& c, Real mass) {
    switch (c.type) {
        case ColliderDesc::Type::Sphere:  return solidSphereInertia(mass, c.sphere.radius);
        case ColliderDesc::Type::Box: {
            const Vec3 h2 = c.box.halfExtents * c.box.halfExtents;
            return Mat3(Real(1) / Real(3) * mass * (h2.y + h2.z), 0, 0,
                        0, Real(1) / Real(3) * mass * (h2.x + h2.z), 0,
                        0, 0, Real(1) / Real(3) * mass * (h2.x + h2.y));
        }
        case ColliderDesc::Type::Capsule: {
            const Real r = c.capsule.radius, hh = c.capsule.halfHeight, h = Real(2) * hh;
            const Real iy = Real(0.5) * mass * r * r;
            const Real ip = Real(1) / Real(12) * mass * (Real(3) * r * r + h * h);
            return Mat3(ip, 0, 0, 0, iy, 0, 0, 0, ip);
        }
        default: { const Real i = Real(0.4) * mass; return Mat3(i, 0, 0, 0, i, 0, 0, 0, i); }
    }
}

Mat3 frameFromZ(Vec3 z) {
    z = glm::normalize(z);
    const Vec3 t = (std::fabs(z.x) < Real(0.9)) ? Vec3(1, 0, 0) : Vec3(0, 1, 0);
    const Vec3 x = glm::normalize(glm::cross(t, z));
    const Vec3 y = glm::cross(z, x);
    return Mat3(x, y, z);
}
Mat3 rotZ(Real a) {
    const Real c = std::cos(a), s = std::sin(a);
    return Mat3(c, s, 0,  -s, c, 0,  0, 0, 1);
}

// Two unit tangents spanning the plane perpendicular to unit `n` (for friction).
void basisPerp(const Vec3& n, Vec3& t1, Vec3& t2) {
    const Mat3 F = frameFromZ(n);
    t1 = Vec3(F[0]); t2 = Vec3(F[1]);
}

Quat integrateQuat(const Quat& q, const Vec3& worldAngVel, Real h) {
    const Vec3 wdt = worldAngVel * h;
    const Real ang = glm::length(wdt);
    if (ang < Real(1e-9)) return glm::normalize(q);
    const Quat dq = glm::angleAxis(ang, wdt / ang);
    return glm::normalize(dq * q);
}

// ------------------------------------------------------------------ model ----------------------
struct Link {
    Real       mass = 1;
    Mat3       Ibody{Real(0)};
    Mat6       I = Mat6::zero();
    BodyType   type = BodyType::Dynamic;
    ColliderDesc    collider{};       // for E1 contact detection
    PhysicsMaterial material{};
    int        parent = -1;
    int        jointIndex = -1;
    Vec3       pos0{0}; Quat quat0{1, 0, 0, 0};
    engine::Transform world{};
    Vec6       v{};
};

struct Joint {
    JointType type = JointType::Revolute;
    int   parent = -1, child = -1;
    int   dof = 1;
    Mat3  Rj{Real(1)};
    Vec3  anchorP{0};
    Mat3  R_c_in_j{Real(1)};
    Vec3  r_jc{0};
    Vec6  S{};
    Actuator actuator{};
    Real  q = 0, qd = 0;
    int   qIndex = -1;                // column in the generalized-velocity vector (revolute)
};

// ------------------------------------------------------------------ world ----------------------
class FeatherstoneWorld final : public PhysicsWorld {
public:
    explicit FeatherstoneWorld(const WorldDef& def)
        : gravity_(def.gravity), substeps_(def.substeps > 0 ? def.substeps : 1),
          angularDamping_(def.angularDamping) {}

    BodyHandle createBody(const BodyDef& d) override {
        Link l;
        l.mass = d.mass; l.type = d.type;
        l.collider = d.collider; l.material = d.material;
        l.Ibody = colliderInertia(d.collider, d.mass);
        l.I = spatialInertia(d.mass, l.Ibody);
        l.pos0 = d.position; l.quat0 = glm::normalize(d.orientation);
        l.world.position = l.pos0; l.world.rotation = l.quat0;
        const uint32_t idx = static_cast<uint32_t>(links_.size());
        links_.push_back(l);
        poses_.push_back(l.world);
        linVel_.push_back(Vec3(0)); angVel_.push_back(Vec3(0));
        inited_ = false;
        return BodyHandle{ idx, 1 };
    }

    void destroyBody(BodyHandle) override {}

    void setGravity(Vec3 g) override { gravity_ = g; }

    JointHandle createJoint(const JointDef& d) override {
        if (d.a.index >= links_.size() || d.b.index >= links_.size()) return JointHandle{};
        Joint j;
        j.type = d.type;
        j.parent = static_cast<int>(d.a.index);
        j.child  = static_cast<int>(d.b.index);
        j.actuator = d.actuator;
        j.dof = (d.type == JointType::Revolute) ? 1 : 0;   // Ball (3) is E2

        const Link& P = links_[j.parent];
        const Link& C = links_[j.child];
        const Mat3 Rwp = glm::mat3_cast(P.quat0);
        const Mat3 Rwc = glm::mat3_cast(C.quat0);
        const Mat3 R_cp0 = glm::transpose(Rwp) * Rwc;

        j.anchorP = d.localAnchorA;
        j.Rj = frameFromZ(d.localAxisA);
        j.R_c_in_j = glm::transpose(j.Rj) * R_cp0;
        j.r_jc = -(j.R_c_in_j * d.localAnchorB);

        const Mat6 Xcj = plux(glm::transpose(j.R_c_in_j), j.r_jc);
        j.S = mul(Xcj, v6(Vec3(0, 0, 1), Vec3(0)));

        const uint32_t idx = static_cast<uint32_t>(joints_.size());
        joints_.push_back(j);
        links_[j.child].parent = j.parent;
        links_[j.child].jointIndex = static_cast<int>(idx);
        jointStates_.push_back(JointState{});
        inited_ = false;
        return JointHandle{ idx, 1 };
    }

    void destroyJoint(JointHandle) override {}

    void setJointActuator(JointHandle h, const Actuator& a) override {
        if (h.index < joints_.size()) joints_[h.index].actuator = a;
    }
    void setJointTarget(JointHandle h, Real t) override {
        if (h.index < joints_.size()) joints_[h.index].actuator.target = t;
    }
    void setJointTorque(JointHandle h, Real t) override {
        if (h.index < joints_.size()) joints_[h.index].actuator.torque = t;
    }
    void setJointBallTorque(JointHandle h, Vec3 t) override {
        if (h.index < joints_.size()) joints_[h.index].actuator.ballTorque = t;   // Ball is E2
    }
    void setJointTargets(std::span<const Real> t) override {
        for (size_t i = 0; i < t.size() && i < joints_.size(); ++i) joints_[i].actuator.target = t[i];
    }
    void setJointTorques(std::span<const Real> t) override {
        for (size_t i = 0; i < t.size() && i < joints_.size(); ++i) joints_[i].actuator.torque = t[i];
    }

    JointState jointState(JointHandle h) const override {
        return h.index < jointStates_.size() ? jointStates_[h.index] : JointState{};
    }
    std::span<const JointState> jointStates() const override { return jointStates_; }

    void setBodyState(BodyHandle h, const Vec3& p, const Quat& q, const Vec3& lv,
                      const Vec3& av) override {
        ensureInit();
        if (h.index >= links_.size()) return;
        if (static_cast<int>(h.index) == rootIndex_) {
            basePos_ = p; baseQuat_ = glm::normalize(q);
            const Mat3 Rt = glm::transpose(glm::mat3_cast(baseQuat_));
            baseTwist_ = v6(Rt * av, Rt * lv);   // world twist → base frame
        }
        // Non-root links are defined by their joint q; a general pose-set isn't meaningful here.
    }
    void clearState() override {
        ensureInit();
        for (Joint& j : joints_) { j.q = 0; j.qd = 0; }
        baseTwist_ = Vec6{};
        if (rootIndex_ >= 0) { basePos_ = links_[rootIndex_].pos0; baseQuat_ = links_[rootIndex_].quat0; }
        refreshState();
    }
    void refreshState() override { ensureInit(); updatePoses(); computeVelocities(); writeJointStates(); }

    void step(Real dt) override {
        ensureInit();
        const Real h = dt / static_cast<Real>(substeps_);
        for (int s = 0; s < substeps_; ++s) {
            updatePoses();                          // world poses reflect current q/base (contacts)
            computeAccelerations();                 // fills qddot_ + baseAccel_ (no integration)
            // integrate velocities (semi-implicit).
            for (size_t i = 0; i < joints_.size(); ++i) {
                if (joints_[i].dof != 1) continue;
                joints_[i].qd += qddot_[i] * h;
                if (angularDamping_ > 0) joints_[i].qd *= std::max(Real(0), Real(1) - angularDamping_ * h);
            }
            if (floating_) baseTwist_ = addV(baseTwist_, scale(baseAccel_, h));
            solveContacts(h);                        // E1: correct velocities for contacts
            // integrate positions.
            for (Joint& j : joints_) if (j.dof == 1) j.q += j.qd * h;
            if (floating_) integrateBasePose(h);
        }
        updatePoses();
        computeVelocities();
        writeJointStates();
    }

    std::span<const engine::Transform> poses() const override { return poses_; }
    std::span<const Vec3> linearVelocities() const override { return linVel_; }
    std::span<const Vec3> angularVelocities() const override { return angVel_; }
    engine::Transform pose(BodyHandle h) const override {
        return h.index < poses_.size() ? poses_[h.index] : engine::Transform{};
    }
    std::span<const ContactEvent> contacts() const override { return contacts_; }

private:
    void ensureInit() {
        if (inited_) return;
        // A parentless Static link with no children is environment geometry (e.g. the ground),
        // not the articulation base. The tree root is the parentless link that is Dynamic or has
        // children.
        std::vector<bool> hasChild(links_.size(), false);
        for (const Joint& j : joints_) if (j.parent >= 0) hasChild[j.parent] = true;
        rootIndex_ = -1;
        for (size_t i = 0; i < links_.size(); ++i)
            if (links_[i].parent < 0 && (links_[i].type == BodyType::Dynamic || hasChild[i])) { rootIndex_ = static_cast<int>(i); break; }
        if (rootIndex_ >= 0) {
            floating_ = (links_[rootIndex_].type == BodyType::Dynamic);
            basePos_  = links_[rootIndex_].pos0;
            baseQuat_ = links_[rootIndex_].quat0;
        }
        // Generalized-coordinate layout: base 6 DOF (if floating) then one column per revolute.
        baseDof_ = floating_ ? 6 : 0;
        int q = baseDof_;
        for (Joint& j : joints_) if (j.dof == 1) j.qIndex = q++;
        ndof_ = q;
        inited_ = true;
    }

    bool isFloatingRoot(size_t i) const { return static_cast<int>(i) == rootIndex_ && floating_; }

    void jointRel(const Joint& j, Mat3& R_cp, Vec3& p_cp) const {
        const Mat3 M = j.Rj * rotZ(j.dof == 1 ? j.q : Real(0));
        R_cp = M * j.R_c_in_j;
        p_cp = j.anchorP + M * j.r_jc;
    }
    Mat6 Xup(const Joint& j) const {
        Mat3 R_cp; Vec3 p_cp; jointRel(j, R_cp, p_cp);
        return plux(glm::transpose(R_cp), p_cp);
    }

    // Gravity as an external spatial force in a link's own frame (force at COM, no torque).
    Vec6 gravityForce(Real mass, const Mat3& Rworld) const {
        return v6(Vec3(0), glm::transpose(Rworld) * (mass * gravity_));
    }

    void computeAccelerations() {
        const size_t n = links_.size();
        qddot_.assign(joints_.size(), Real(0));
        std::vector<Mat6> Xup_(n), IA(n);
        std::vector<Vec6> v(n), c(n), pA(n), a(n);
        std::vector<Mat3> Rw(n);
        std::vector<Vec6> U(joints_.size());
        std::vector<Real> D(joints_.size()), u(joints_.size());

        // Pass 1: base → tips (world rotations, velocities, bias forces incl. gravity).
        for (size_t i = 0; i < n; ++i) {
            const Link& L = links_[i];
            if (L.parent < 0) {
                Rw[i] = glm::mat3_cast(L.world.rotation);
                v[i]  = isFloatingRoot(i) ? baseTwist_ : Vec6{};
                c[i]  = Vec6{};
            } else {
                const Joint& j = joints_[L.jointIndex];
                Mat3 R_cp; Vec3 p_cp; jointRel(j, R_cp, p_cp);
                Xup_[i] = plux(glm::transpose(R_cp), p_cp);
                Rw[i]   = Rw[L.parent] * R_cp;
                const Vec6 vJ = (j.dof == 1) ? scale(j.S, j.qd) : Vec6{};
                v[i] = addV(mul(Xup_[i], v[L.parent]), vJ);
                c[i] = (j.dof == 1) ? mul(crm(v[i]), vJ) : Vec6{};
            }
            IA[i] = L.I;
            // pA = crf(v)·I·v − f_grav ,  crf(v) = −crm(v)ᵀ
            const Vec6 bias = scale(mul(transpose(crm(v[i])), mul(L.I, v[i])), Real(-1));
            pA[i] = subV(bias, gravityForce(L.mass, Rw[i]));
        }

        // Pass 2: tips → base (articulated inertia + bias).
        for (size_t ri = 0; ri < n; ++ri) {
            const size_t i = n - 1 - ri;
            const Link& L = links_[i];
            if (L.parent < 0) continue;
            const Joint& j = joints_[L.jointIndex];
            const int ji = L.jointIndex;
            Mat6 Ia = IA[i];
            Vec6 pa = pA[i];
            if (j.dof == 1) {
                U[ji] = mul(IA[i], j.S);
                D[ji] = dot(j.S, U[ji]);
                u[ji] = jointTorque(j) - dot(j.S, pA[i]);
                const Real invD = (std::fabs(D[ji]) > kEps) ? Real(1) / D[ji] : Real(0);
                Ia = subM(IA[i], outerOverD(U[ji], invD));
                pa = addV(addV(pA[i], mul(Ia, c[i])), scale(U[ji], u[ji] * invD));
            }
            const Mat6 XT = transpose(Xup_[i]);
            IA[L.parent] = addM(IA[L.parent], mul(mul(XT, Ia), Xup_[i]));
            pA[L.parent] = addV(pA[L.parent], mul(XT, pa));
        }

        // Pass 3: base → tips (accelerations); integrate rates.
        for (size_t i = 0; i < n; ++i) {
            const Link& L = links_[i];
            if (L.parent < 0) {
                if (isFloatingRoot(i)) { a[i] = solve6(IA[i], scale(pA[i], Real(-1))); baseAccel_ = a[i]; }
                else a[i] = Vec6{};
                continue;
            }
            const Joint& j = joints_[L.jointIndex];
            const int ji = L.jointIndex;
            const Vec6 ap = addV(mul(Xup_[i], a[L.parent]), c[i]);
            if (j.dof == 1) {
                const Real invD = (std::fabs(D[ji]) > kEps) ? Real(1) / D[ji] : Real(0);
                const Real qdd = (u[ji] - dot(U[ji], ap)) * invD;
                qddot_[ji] = qdd;
                a[i] = addV(ap, scale(j.S, qdd));
            } else {
                a[i] = ap;
            }
        }
    }

    // Per-link ^linkX_base transforms (base frame → link frame) + Xup array, for Jacobians/CRBA.
    void forwardTransforms(std::vector<Mat6>& Xup, std::vector<Mat6>& Xbase) const {
        const size_t n = links_.size();
        Xup.assign(n, Mat6::zero());
        Xbase.assign(n, Mat6::zero());
        for (size_t i = 0; i < n; ++i) {
            const Link& L = links_[i];
            if (L.parent < 0) { Xbase[i] = identity6(); }
            else { Xup[i] = this->Xup(joints_[L.jointIndex]); Xbase[i] = mul(Xup[i], Xbase[L.parent]); }
        }
    }

    // Composite-Rigid-Body Algorithm: dense joint-space inertia matrix H (row-major, ndof_×ndof_).
    std::vector<Real> buildMassMatrix() const {
        const size_t n = links_.size();
        std::vector<Mat6> Xup(n, Mat6::zero()), Ic(n);
        for (size_t i = 0; i < n; ++i) {
            Ic[i] = links_[i].I;
            if (links_[i].parent >= 0) Xup[i] = this->Xup(joints_[links_[i].jointIndex]);
        }
        for (size_t ri = 0; ri < n; ++ri) {              // composite inertia, tips → base
            const size_t i = n - 1 - ri;
            if (links_[i].parent >= 0) {
                const Mat6 XT = transpose(Xup[i]);
                Ic[links_[i].parent] = addM(Ic[links_[i].parent], mul(mul(XT, Ic[i]), Xup[i]));
            }
        }
        const int nd = ndof_;
        std::vector<Real> H(static_cast<size_t>(nd) * nd, Real(0));
        if (floating_ && rootIndex_ >= 0)               // base-base block = whole-tree composite
            for (int a = 0; a < 6; ++a) for (int b = 0; b < 6; ++b) H[a * nd + b] = Ic[rootIndex_].m[a][b];

        for (size_t i = 0; i < n; ++i) {                 // per-joint columns + cross terms up the tree
            const Link& L = links_[i];
            if (L.parent < 0 || joints_[L.jointIndex].dof != 1) continue;
            const int qi = joints_[L.jointIndex].qIndex;
            const Vec6 S = joints_[L.jointIndex].S;
            Vec6 F = mul(Ic[i], S);
            H[qi * nd + qi] = dot(S, F);
            int k = static_cast<int>(i);
            while (links_[k].parent >= 0) {
                F = mul(transpose(Xup[k]), F);           // move F to parent frame
                const int p = links_[k].parent;
                if (links_[p].parent < 0) {              // p is the base
                    if (floating_) for (int c = 0; c < 6; ++c) { H[qi * nd + c] = F.d[c]; H[c * nd + qi] = F.d[c]; }
                } else {
                    const Joint& jp = joints_[links_[p].jointIndex];
                    if (jp.dof == 1) { const Real h = dot(F, jp.S); H[qi * nd + jp.qIndex] = h; H[jp.qIndex * nd + qi] = h; }
                }
                k = p;
            }
        }
        return H;
    }

    static Mat6 identity6() { Mat6 I = Mat6::zero(); for (int i = 0; i < 6; ++i) I.m[i][i] = 1; return I; }

    // Kinetic energy from link spatial velocities (½ Σ vᵀ I v) — used to validate H (½ q̇ᵀ H q̇).
    Real spatialKineticEnergy() {
        computeVelocities();
        Real ke = 0;
        for (const Link& L : links_) ke += Real(0.5) * dot(L.v, mul(L.I, L.v));
        return ke;
    }
    Real generalizedVelocity(int dof) const {
        if (dof < baseDof_) return baseTwist_.d[dof];
        for (const Joint& j : joints_) if (j.qIndex == dof) return j.qd;
        return 0;
    }
    // E1: velocity-level contact coupling in generalized coordinates.
    struct Contact { int link; Vec3 point; Vec3 normal; Real pen; Real friction; };

    // Dynamic-link colliders vs static planes (the ground). Sphere/box/capsule supported.
    std::vector<Contact> detectContacts() const {
        std::vector<Contact> out;
        struct WPlane { Vec3 n; Real off; Real fric; };
        std::vector<WPlane> planes;
        for (const Link& s : links_) {
            if (s.type == BodyType::Static && s.collider.type == ColliderDesc::Type::Plane) {
                const Mat3 R = glm::mat3_cast(s.world.rotation);
                const Vec3 n = R * s.collider.plane.normal;
                planes.push_back({ n, s.collider.plane.offset + glm::dot(n, s.world.position), s.material.friction });
            }
        }
        if (planes.empty()) return out;

        auto sphereVsPlanes = [&](int link, const Vec3& c, Real radius, Real mat) {
            for (const WPlane& p : planes) {
                const Real dist = glm::dot(p.n, c) - p.off;
                if (dist < radius)
                    out.push_back({ link, c - p.n * dist, p.n, radius - dist,
                                    std::sqrt(std::max(Real(0), mat * p.fric)) });
            }
        };
        for (size_t i = 0; i < links_.size(); ++i) {
            const Link& L = links_[i];
            if (L.type != BodyType::Dynamic) continue;
            const Vec3 c = L.world.position;
            const Mat3 R = glm::mat3_cast(L.world.rotation);
            const int li = static_cast<int>(i);
            switch (L.collider.type) {
                case ColliderDesc::Type::Sphere:
                    sphereVsPlanes(li, c, L.collider.sphere.radius, L.material.friction); break;
                case ColliderDesc::Type::Capsule: {
                    const Real hh = L.collider.capsule.halfHeight, r = L.collider.capsule.radius;
                    sphereVsPlanes(li, c + R * Vec3(0, hh, 0), r, L.material.friction);
                    sphereVsPlanes(li, c - R * Vec3(0, hh, 0), r, L.material.friction);
                    break;
                }
                case ColliderDesc::Type::Box: {
                    const Vec3 hE = L.collider.box.halfExtents;
                    for (int sx = -1; sx <= 1; sx += 2) for (int sy = -1; sy <= 1; sy += 2) for (int sz = -1; sz <= 1; sz += 2) {
                        const Vec3 corner = c + R * Vec3(sx * hE.x, sy * hE.y, sz * hE.z);
                        for (const WPlane& p : planes) {
                            const Real dist = glm::dot(p.n, corner) - p.off;
                            if (dist < 0) out.push_back({ li, corner, p.n, -dist,
                                                          std::sqrt(std::max(Real(0), L.material.friction * p.fric)) });
                        }
                    }
                    break;
                }
                default: sphereVsPlanes(li, c, Real(0.1), L.material.friction); break;
            }
        }
        return out;
    }

    // Jacobian row (1×ndof): maps generalized velocity → velocity of world `point` (on `link`)
    // projected onto `dir`. Ancestor revolute joints + the floating base contribute.
    std::vector<Real> jacRow(int link, const Vec3& point, const Vec3& dir) const {
        std::vector<Real> row(static_cast<size_t>(ndof_), Real(0));
        int k = link;
        while (links_[k].parent >= 0) {
            const Joint& jk = joints_[links_[k].jointIndex];
            if (jk.dof == 1) {
                const int p = jk.parent;
                const Mat3 Rp = glm::mat3_cast(links_[p].world.rotation);
                const Vec3 axisW = Rp * (jk.Rj * Vec3(0, 0, 1));
                const Vec3 anchorW = links_[p].world.position + Rp * jk.anchorP;
                row[jk.qIndex] += glm::dot(dir, glm::cross(axisW, point - anchorW));
            }
            k = links_[k].parent;
        }
        if (floating_ && rootIndex_ >= 0) {
            const Mat3 Rwb = glm::mat3_cast(links_[rootIndex_].world.rotation);
            const Vec3 comBase = links_[rootIndex_].world.position;
            for (int a = 0; a < 3; ++a) {
                const Vec3 na(Rwb[a]);                                        // world axis a
                row[a]     = glm::dot(dir, glm::cross(na, point - comBase));  // base angular
                row[3 + a] = glm::dot(dir, na);                              // base linear
            }
        }
        return row;
    }

    static std::vector<Real> matVecDense(const std::vector<Real>& M, int n, const std::vector<Real>& x) {
        std::vector<Real> r(static_cast<size_t>(n), Real(0));
        for (int i = 0; i < n; ++i) { Real s = 0; for (int j = 0; j < n; ++j) s += M[i * n + j] * x[j]; r[i] = s; }
        return r;
    }
    static Real dotN(const std::vector<Real>& a, const std::vector<Real>& b) {
        Real s = 0; for (size_t i = 0; i < a.size(); ++i) s += a[i] * b[i]; return s;
    }

    void solveContacts(Real h) {
        const std::vector<Contact> contacts = detectContacts();
        if (contacts.empty()) return;
        const int nd = ndof_;

        std::vector<Real> Hinv;
        if (!invertDense(buildMassMatrix(), nd, Hinv)) return;

        std::vector<Real> qd(static_cast<size_t>(nd), Real(0));          // generalized velocity
        for (int d = 0; d < baseDof_; ++d) qd[d] = baseTwist_.d[d];
        for (const Joint& j : joints_) if (j.dof == 1) qd[j.qIndex] = j.qd;

        struct Row { std::vector<Real> Jn, Jt1, Jt2, JnHi, Jt1Hi, Jt2Hi; Real An, At1, At2, biasN, fric; Real ln = 0, l1 = 0, l2 = 0; };
        std::vector<Row> rows;
        rows.reserve(contacts.size());
        constexpr Real kBeta = Real(0.2), kSlop = Real(0.001);
        for (const Contact& c : contacts) {
            Vec3 t1, t2; basisPerp(c.normal, t1, t2);
            Row r;
            r.Jn = jacRow(c.link, c.point, c.normal);
            r.Jt1 = jacRow(c.link, c.point, t1);
            r.Jt2 = jacRow(c.link, c.point, t2);
            r.JnHi = matVecDense(Hinv, nd, r.Jn);
            r.Jt1Hi = matVecDense(Hinv, nd, r.Jt1);
            r.Jt2Hi = matVecDense(Hinv, nd, r.Jt2);
            r.An = dotN(r.Jn, r.JnHi);
            r.At1 = dotN(r.Jt1, r.Jt1Hi);
            r.At2 = dotN(r.Jt2, r.Jt2Hi);
            r.biasN = -(kBeta / h) * std::max(Real(0), c.pen - kSlop);   // push out penetration
            r.fric = c.friction;
            rows.push_back(std::move(r));
        }

        constexpr int kIters = 20;                                       // sequential-impulse PGS
        for (int it = 0; it < kIters; ++it) {
            for (Row& r : rows) {
                if (r.An > kEps) {                                       // normal (λ ≥ 0)
                    const Real vn = dotN(r.Jn, qd);
                    Real dLam = -(vn + r.biasN) / r.An;
                    const Real newLam = std::max(Real(0), r.ln + dLam);
                    dLam = newLam - r.ln; r.ln = newLam;
                    for (int i = 0; i < nd; ++i) qd[i] += r.JnHi[i] * dLam;
                }
                const Real muN = r.fric * r.ln;                          // Coulomb friction cone
                if (r.At1 > kEps) {
                    Real dl = -dotN(r.Jt1, qd) / r.At1;
                    const Real nl = std::max(-muN, std::min(muN, r.l1 + dl));
                    dl = nl - r.l1; r.l1 = nl;
                    for (int i = 0; i < nd; ++i) qd[i] += r.Jt1Hi[i] * dl;
                }
                if (r.At2 > kEps) {
                    Real dl = -dotN(r.Jt2, qd) / r.At2;
                    const Real nl = std::max(-muN, std::min(muN, r.l2 + dl));
                    dl = nl - r.l2; r.l2 = nl;
                    for (int i = 0; i < nd; ++i) qd[i] += r.Jt2Hi[i] * dl;
                }
            }
        }

        for (int d = 0; d < baseDof_; ++d) baseTwist_.d[d] = qd[d];      // write corrected velocities
        for (Joint& j : joints_) if (j.dof == 1) j.qd = qd[j.qIndex];
    }

    void integrateBasePose(Real h) {
        const Mat3 Rw = glm::mat3_cast(baseQuat_);
        const Vec3 wWorld = Rw * v6ang(baseTwist_);
        const Vec3 vWorld = Rw * v6lin(baseTwist_);
        basePos_ += vWorld * h;
        baseQuat_ = integrateQuat(baseQuat_, wWorld, h);
    }

    Real jointTorque(const Joint& j) const {
        Real tau = 0;
        if (j.actuator.mode == ActuatorMode::Torque) {
            tau = j.actuator.torque;
        } else if (j.actuator.mode == ActuatorMode::PDTarget) {
            tau = j.actuator.kp * (j.actuator.target - j.q) + j.actuator.kd * (j.actuator.targetVel - j.qd);
        }
        if (j.actuator.maxTorque > 0) tau = std::max(-j.actuator.maxTorque, std::min(j.actuator.maxTorque, tau));
        return tau;
    }

    void updatePoses() {
        for (size_t i = 0; i < links_.size(); ++i) {
            Link& L = links_[i];
            if (isFloatingRoot(i)) { L.world.position = basePos_; L.world.rotation = baseQuat_; }
            else if (L.parent < 0) { L.world.position = L.pos0; L.world.rotation = L.quat0; }
            else {
                const Joint& j = joints_[L.jointIndex];
                Mat3 R_cp; Vec3 p_cp; jointRel(j, R_cp, p_cp);
                const Link& P = links_[L.parent];
                L.world.rotation = glm::normalize(P.world.rotation * glm::normalize(glm::quat_cast(R_cp)));
                L.world.position = P.world.position + glm::mat3_cast(P.world.rotation) * p_cp;
            }
            poses_[i] = L.world;
        }
    }

    void computeVelocities() {
        for (size_t i = 0; i < links_.size(); ++i) {
            Link& L = links_[i];
            if (L.parent < 0) { L.v = isFloatingRoot(i) ? baseTwist_ : Vec6{}; }
            else {
                const Joint& j = joints_[L.jointIndex];
                const Vec6 vJ = (j.dof == 1) ? scale(j.S, j.qd) : Vec6{};
                L.v = addV(mul(Xup(j), links_[L.parent].v), vJ);
            }
            const Mat3 Rw = glm::mat3_cast(L.world.rotation);
            angVel_[i] = Rw * v6ang(L.v);
            linVel_[i] = Rw * v6lin(L.v);
        }
    }

    void writeJointStates() {
        for (size_t i = 0; i < joints_.size(); ++i) {
            jointStates_[i].q  = (joints_[i].dof == 1) ? joints_[i].q  : Real(0);
            jointStates_[i].qd = (joints_[i].dof == 1) ? joints_[i].qd : Real(0);
        }
    }

    static constexpr Real kEps = Real(1e-12);

    Vec3 gravity_{0, Real(-9.81), 0};
    int  substeps_ = 1;
    Real angularDamping_ = 0;

    bool inited_ = false;
    int  rootIndex_ = -1;
    bool floating_ = false;
    Vec3 basePos_{0};
    Quat baseQuat_{1, 0, 0, 0};
    Vec6 baseTwist_{};                  // base spatial velocity in the base frame
    Vec6 baseAccel_{};                  // base spatial acceleration (from computeAccelerations)
    std::vector<Real> qddot_;           // per-joint acceleration (from computeAccelerations)
    int  ndof_ = 0;                     // total generalized DOFs (baseDof_ + revolute joints)
    int  baseDof_ = 0;                  // 6 if floating, else 0

    std::vector<Link>  links_;
    std::vector<Joint> joints_;
    std::vector<engine::Transform> poses_;
    std::vector<Vec3>  linVel_, angVel_;
    std::vector<JointState> jointStates_;
    std::vector<ContactEvent> contacts_;
};

} // namespace

std::unique_ptr<PhysicsWorld> createFeatherstoneWorld(const WorldDef& def) {
    return std::make_unique<FeatherstoneWorld>(def);
}

} // namespace engine::physics
