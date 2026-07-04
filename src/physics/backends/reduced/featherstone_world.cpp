//
//  featherstone_world.cpp
//  engine::physics / backends / reduced
//
//  Reduced-coordinate PhysicsWorld backend (Phase E). Articulated body in generalized coordinates
//  (joint DOFs + optional floating 6-DOF base), stepped with the Articulated-Body Algorithm (ABA),
//  O(n). Same PhysicsWorld interface as the maximal (Realtime) backend. See notes/investigations/
//  2026-07-04-reduced-coordinate-backend.md.
//
//  Scope (E0-E2): Fixed (0-DOF) / Revolute (1-DOF) / Ball (3-DOF) rotation joints, fixed OR
//  floating base, gravity + actuators (torque/PD), and contacts vs static planes (CRBA H +
//  generalized-coordinate PGS). Rotation joints share one model: relRot = restRel · locRot, with a
//  motion-subspace column per DOF axis a: S_a = [axis_a; −axis_a × anchorC] (child frame). This
//  reproduces the revolute case exactly and generalizes to the ball's 3 axes.
//
//  Spatial algebra: angular-first 6-vectors [w; v], full 6x6 matrices (n small). Link frame at COM.
//  Gravity is an explicit per-link force (uniform for fixed + floating bases).
//

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "engine/physics/dynamics/body.h"
#include "engine/physics/world.h"

#include "../backends_internal.h"

namespace engine::physics {
namespace {

// ------------------------------------------------------------------ spatial algebra (6D) -------
struct Vec6 { Real d[6]; };

inline Vec6 v6(const Vec3& ang, const Vec3& lin) { return { ang.x, ang.y, ang.z, lin.x, lin.y, lin.z }; }
inline Vec3 v6ang(const Vec6& v) { return Vec3(v.d[0], v.d[1], v.d[2]); }
inline Vec3 v6lin(const Vec6& v) { return Vec3(v.d[3], v.d[4], v.d[5]); }

struct Mat6 {
    Real m[6][6];
    static Mat6 zero() { Mat6 r{}; for (auto& row : r.m) for (Real& x : row) x = 0; return r; }
};

inline Real at3(const Mat3& M, int i, int j) { return M[j][i]; }   // glm col-major → math (i,j)
inline void setBlock(Mat6& M, int r0, int c0, const Mat3& B, bool neg = false) {
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) M.m[r0 + i][c0 + j] = neg ? -at3(B, i, j) : at3(B, i, j);
}
inline Mat3 skew(const Vec3& v) { return Mat3(0, v.z, -v.y,  -v.z, 0, v.x,  v.y, -v.x, 0); }

inline Mat6 plux(const Mat3& E, const Vec3& r) {   // motion transform [[E,0],[-E skew(r),E]]
    Mat6 X = Mat6::zero();
    setBlock(X, 0, 0, E); setBlock(X, 3, 3, E); setBlock(X, 3, 0, E * skew(r), /*neg=*/true);
    return X;
}
inline Mat6 spatialInertia(Real mass, const Mat3& Ic) {   // COM at frame origin
    Mat6 I = Mat6::zero();
    setBlock(I, 0, 0, Ic); I.m[3][3] = mass; I.m[4][4] = mass; I.m[5][5] = mass;
    return I;
}
inline Mat6 crm(const Vec6& v) {   // motion cross [[skew(w),0],[skew(vl),skew(w)]]
    const Vec3 w = v6ang(v), vl = v6lin(v);
    Mat6 M = Mat6::zero(); setBlock(M, 0, 0, skew(w)); setBlock(M, 3, 3, skew(w)); setBlock(M, 3, 0, skew(vl));
    return M;
}
inline Vec6 mul(const Mat6& A, const Vec6& x) {
    Vec6 r{}; for (int i = 0; i < 6; ++i) { Real s = 0; for (int j = 0; j < 6; ++j) s += A.m[i][j] * x.d[j]; r.d[i] = s; } return r;
}
inline Mat6 mul(const Mat6& A, const Mat6& B) {
    Mat6 r = Mat6::zero();
    for (int i = 0; i < 6; ++i) for (int k = 0; k < 6; ++k) { const Real a = A.m[i][k]; if (a != 0) for (int j = 0; j < 6; ++j) r.m[i][j] += a * B.m[k][j]; }
    return r;
}
inline Mat6 transpose(const Mat6& A) { Mat6 r = Mat6::zero(); for (int i = 0; i < 6; ++i) for (int j = 0; j < 6; ++j) r.m[i][j] = A.m[j][i]; return r; }
inline Mat6 addM(const Mat6& A, const Mat6& B) { Mat6 r; for (int i = 0; i < 6; ++i) for (int j = 0; j < 6; ++j) r.m[i][j] = A.m[i][j] + B.m[i][j]; return r; }
inline Mat6 subM(const Mat6& A, const Mat6& B) { Mat6 r; for (int i = 0; i < 6; ++i) for (int j = 0; j < 6; ++j) r.m[i][j] = A.m[i][j] - B.m[i][j]; return r; }
inline Vec6 addV(const Vec6& a, const Vec6& b) { Vec6 r{}; for (int i = 0; i < 6; ++i) r.d[i] = a.d[i] + b.d[i]; return r; }
inline Vec6 scale(const Vec6& a, Real s) { Vec6 r{}; for (int i = 0; i < 6; ++i) r.d[i] = a.d[i] * s; return r; }
inline Real dot(const Vec6& a, const Vec6& b) { Real s = 0; for (int i = 0; i < 6; ++i) s += a.d[i] * b.d[i]; return s; }
inline Mat6 outerScaled(const Vec6& a, const Vec6& b, Real s) {   // (a bᵀ)·s
    Mat6 r; for (int i = 0; i < 6; ++i) for (int j = 0; j < 6; ++j) r.m[i][j] = a.d[i] * b.d[j] * s; return r;
}

// Invert a dense n×n matrix (row-major) via Gauss-Jordan; false if singular. Small n.
bool invertDense(const std::vector<Real>& A, int n, std::vector<Real>& inv) {
    std::vector<Real> M(A);
    inv.assign(static_cast<size_t>(n) * n, Real(0));
    for (int i = 0; i < n; ++i) inv[i * n + i] = Real(1);
    for (int col = 0; col < n; ++col) {
        int piv = col;
        for (int r = col + 1; r < n; ++r) if (std::fabs(M[r * n + col]) > std::fabs(M[piv * n + col])) piv = r;
        if (std::fabs(M[piv * n + col]) < Real(1e-12)) return false;
        if (piv != col) for (int j = 0; j < n; ++j) { std::swap(M[col * n + j], M[piv * n + j]); std::swap(inv[col * n + j], inv[piv * n + j]); }
        const Real d = M[col * n + col];
        for (int j = 0; j < n; ++j) { M[col * n + j] /= d; inv[col * n + j] /= d; }
        for (int r = 0; r < n; ++r) { if (r == col) continue; const Real f = M[r * n + col]; if (f == 0) continue;
            for (int j = 0; j < n; ++j) { M[r * n + j] -= f * M[col * n + j]; inv[r * n + j] -= f * inv[col * n + j]; } }
    }
    return true;
}

Vec6 solve6(const Mat6& A, const Vec6& b) {   // floating-base 6x6 solve
    Real M[6][7];
    for (int i = 0; i < 6; ++i) { for (int j = 0; j < 6; ++j) M[i][j] = A.m[i][j]; M[i][6] = b.d[i]; }
    for (int col = 0; col < 6; ++col) {
        int piv = col;
        for (int r = col + 1; r < 6; ++r) if (std::fabs(M[r][col]) > std::fabs(M[piv][col])) piv = r;
        if (piv != col) for (int j = 0; j < 7; ++j) std::swap(M[col][j], M[piv][j]);
        const Real d = M[col][col]; if (std::fabs(d) < Real(1e-12)) continue;
        for (int r = 0; r < 6; ++r) { if (r == col) continue; const Real f = M[r][col] / d; for (int j = col; j < 7; ++j) M[r][j] -= f * M[col][j]; }
    }
    Vec6 x{}; for (int i = 0; i < 6; ++i) x.d[i] = (std::fabs(M[i][i]) > Real(1e-12)) ? M[i][6] / M[i][i] : Real(0);
    return x;
}

// ------------------------------------------------------------------ small helpers --------------
Mat3 colliderInertia(const ColliderDesc& c, Real mass) {
    switch (c.type) {
        case ColliderDesc::Type::Sphere: return solidSphereInertia(mass, c.sphere.radius);
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
    return Mat3(x, glm::cross(z, x), z);
}
void basisPerp(const Vec3& n, Vec3& t1, Vec3& t2) { const Mat3 F = frameFromZ(n); t1 = Vec3(F[0]); t2 = Vec3(F[1]); }

Quat quatFromRotvec(const Vec3& v) {
    const Real a = glm::length(v);
    return (a < Real(1e-9)) ? Quat(1, 0, 0, 0) : glm::angleAxis(a, v / a);
}
Vec3 quatToRotvec(const Quat& q0) {
    Quat q = q0; if (q.w < 0) q = Quat(-q.w, -q.x, -q.y, -q.z);   // shortest arc
    const Vec3 xyz(q.x, q.y, q.z);
    const Real s = glm::length(xyz);
    if (s < Real(1e-9)) return Vec3(0);
    const Real ang = Real(2) * std::atan2(s, q.w);
    return xyz * (ang / s);
}
Quat integrateQuat(const Quat& q, const Vec3& worldAngVel, Real h) {
    return glm::normalize(quatFromRotvec(worldAngVel * h) * q);
}

// ------------------------------------------------------------------ model ----------------------
struct Link {
    Real       mass = 1;
    Mat3       Ibody{Real(0)};
    Mat6       I = Mat6::zero();
    BodyType   type = BodyType::Dynamic;
    ColliderDesc    collider{};
    PhysicsMaterial material{};
    int        parent = -1;
    int        jointIndex = -1;
    Vec3       pos0{0}; Quat quat0{1, 0, 0, 0};
    engine::Transform world{};
    Vec6       v{};
};

// Unified rotation joint (Fixed 0-DOF, Revolute 1-DOF, Ball 3-DOF). relRot = restRel · locRot.
struct Joint {
    JointType type = JointType::Revolute;
    int   parent = -1, child = -1;
    int   dof = 1;
    Vec3  axis[3]{ Vec3(0, 0, 1), Vec3(0), Vec3(0) };   // child-frame DOF axes
    Vec6  Scol[3]{};                                     // motion subspace columns (child frame)
    Vec3  anchorP{0}, anchorC{0};
    Quat  restRel{1, 0, 0, 0};                           // rest child-in-parent (R_cp0)
    Quat  locRot{1, 0, 0, 0};                            // joint local rotation (child frame)
    Real  q[3]{0, 0, 0};                                 // per-DOF coordinate (revolute[0]=angle)
    Real  qd[3]{0, 0, 0};                                // per-DOF rate (child-frame along axis)
    Actuator actuator{};
    int   qIndex = -1;

    Quat relRot() const { return glm::normalize(restRel * locRot); }
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
        poses_.push_back(l.world); linVel_.push_back(Vec3(0)); angVel_.push_back(Vec3(0));
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
        j.dof = (d.type == JointType::Ball) ? 3 : (d.type == JointType::Revolute ? 1 : 0);
        j.anchorP = d.localAnchorA; j.anchorC = d.localAnchorB;

        const Mat3 Rwp = glm::mat3_cast(links_[j.parent].quat0);
        const Mat3 Rwc = glm::mat3_cast(links_[j.child].quat0);
        const Mat3 R_cp0 = glm::transpose(Rwp) * Rwc;
        j.restRel = glm::normalize(glm::quat_cast(R_cp0));

        if (j.dof == 1) {
            const Mat3 Rj = frameFromZ(d.localAxisA);
            const Mat3 R_c_in_j = glm::transpose(Rj) * R_cp0;
            j.axis[0] = glm::transpose(R_c_in_j) * Vec3(0, 0, 1);       // hinge axis (child frame)
        } else if (j.dof == 3) {
            j.axis[0] = Vec3(1, 0, 0); j.axis[1] = Vec3(0, 1, 0); j.axis[2] = Vec3(0, 0, 1);
        }
        for (int a = 0; a < j.dof; ++a)                                 // S_a = [axis; −axis×anchorC]
            j.Scol[a] = v6(j.axis[a], -glm::cross(j.axis[a], j.anchorC));

        const uint32_t idx = static_cast<uint32_t>(joints_.size());
        joints_.push_back(j);
        links_[j.child].parent = j.parent;
        links_[j.child].jointIndex = static_cast<int>(idx);
        jointStates_.push_back(JointState{});
        inited_ = false;
        return JointHandle{ idx, 1 };
    }
    void destroyJoint(JointHandle) override {}

    void setJointActuator(JointHandle h, const Actuator& a) override { if (h.index < joints_.size()) joints_[h.index].actuator = a; }
    void setJointTarget(JointHandle h, Real t) override { if (h.index < joints_.size()) joints_[h.index].actuator.target = t; }
    void setJointTorque(JointHandle h, Real t) override { if (h.index < joints_.size()) joints_[h.index].actuator.torque = t; }
    void setJointBallTorque(JointHandle h, Vec3 t) override { if (h.index < joints_.size()) joints_[h.index].actuator.ballTorque = t; }
    void setJointTargets(std::span<const Real> t) override { for (size_t i = 0; i < t.size() && i < joints_.size(); ++i) joints_[i].actuator.target = t[i]; }
    void setJointTorques(std::span<const Real> t) override { for (size_t i = 0; i < t.size() && i < joints_.size(); ++i) joints_[i].actuator.torque = t[i]; }

    JointState jointState(JointHandle h) const override { return h.index < jointStates_.size() ? jointStates_[h.index] : JointState{}; }
    std::span<const JointState> jointStates() const override { return jointStates_; }

    void setBodyState(BodyHandle h, const Vec3& p, const Quat& q, const Vec3& lv, const Vec3& av) override {
        ensureInit();
        if (h.index >= links_.size()) return;
        if (static_cast<int>(h.index) == rootIndex_) {
            basePos_ = p; baseQuat_ = glm::normalize(q);
            const Mat3 Rt = glm::transpose(glm::mat3_cast(baseQuat_));
            baseTwist_ = v6(Rt * av, Rt * lv);
        }
    }
    void clearState() override {
        ensureInit();
        for (Joint& j : joints_) { j.locRot = Quat(1, 0, 0, 0); for (int a = 0; a < 3; ++a) j.q[a] = j.qd[a] = 0; }
        baseTwist_ = Vec6{};
        if (rootIndex_ >= 0) { basePos_ = links_[rootIndex_].pos0; baseQuat_ = links_[rootIndex_].quat0; }
        refreshState();
    }
    void refreshState() override { ensureInit(); updatePoses(); computeVelocities(); writeJointStates(); }

    void step(Real dt) override {
        ensureInit();
        const Real h = dt / static_cast<Real>(substeps_);
        for (int s = 0; s < substeps_; ++s) {
            updatePoses();
            computeAccelerations();
            for (Joint& j : joints_) {                          // integrate velocities
                for (int a = 0; a < j.dof; ++a) {
                    j.qd[a] += qddot_[j.qIndex + a] * h;
                    if (angularDamping_ > 0) j.qd[a] *= std::max(Real(0), Real(1) - angularDamping_ * h);
                }
            }
            if (floating_) baseTwist_ = addV(baseTwist_, scale(baseAccel_, h));
            solveContacts(h);
            for (Joint& j : joints_) {                          // integrate positions
                if (j.dof == 0) continue;
                Vec3 wc(0);
                for (int a = 0; a < j.dof; ++a) { wc += j.qd[a] * j.axis[a]; j.q[a] += j.qd[a] * h; }
                j.locRot = glm::normalize(j.locRot * quatFromRotvec(wc * h));
            }
            if (floating_) integrateBasePose(h);
        }
        updatePoses(); computeVelocities(); writeJointStates();
    }

    std::span<const engine::Transform> poses() const override { return poses_; }
    std::span<const Vec3> linearVelocities() const override { return linVel_; }
    std::span<const Vec3> angularVelocities() const override { return angVel_; }
    engine::Transform pose(BodyHandle h) const override { return h.index < poses_.size() ? poses_[h.index] : engine::Transform{}; }
    std::span<const ContactEvent> contacts() const override { return contacts_; }

private:
    void ensureInit() {
        if (inited_) return;
        std::vector<bool> hasChild(links_.size(), false);
        for (const Joint& j : joints_) if (j.parent >= 0) hasChild[j.parent] = true;
        rootIndex_ = -1;
        for (size_t i = 0; i < links_.size(); ++i)
            if (links_[i].parent < 0 && (links_[i].type == BodyType::Dynamic || hasChild[i])) { rootIndex_ = static_cast<int>(i); break; }
        if (rootIndex_ >= 0) { floating_ = (links_[rootIndex_].type == BodyType::Dynamic); basePos_ = links_[rootIndex_].pos0; baseQuat_ = links_[rootIndex_].quat0; }
        baseDof_ = floating_ ? 6 : 0;
        int q = baseDof_;
        for (Joint& j : joints_) { j.qIndex = q; q += j.dof; }
        ndof_ = q;
        // DOF-ancestor tree for the sparse-H factorization: base DOFs chain (0←1←…←5); each joint
        // DOF's parent is the previous DOF in its joint, or the nearest ancestor DOF above it.
        dofParent_.assign(static_cast<size_t>(ndof_), -1);
        for (int k = 1; k < baseDof_; ++k) dofParent_[k] = k - 1;
        for (const Joint& j : joints_) {
            if (j.dof == 0) continue;
            dofParent_[j.qIndex] = ancestorDofOfLink(j.parent);
            for (int a = 1; a < j.dof; ++a) dofParent_[j.qIndex + a] = j.qIndex + a - 1;
        }
        sparseOk_ = true;                                // valid iff every ancestor has a lower index
        for (int i = 0; i < ndof_; ++i) if (dofParent_[i] >= i) { sparseOk_ = false; break; }
        inited_ = true;
    }
    bool isFloatingRoot(size_t i) const { return static_cast<int>(i) == rootIndex_ && floating_; }

    void jointRel(const Joint& j, Mat3& R_cp, Vec3& p_cp) const {
        R_cp = glm::mat3_cast(j.relRot());
        p_cp = j.anchorP - R_cp * j.anchorC;
    }
    Mat6 Xup(const Joint& j) const { Mat3 R_cp; Vec3 p_cp; jointRel(j, R_cp, p_cp); return plux(glm::transpose(R_cp), p_cp); }

    Vec6 gravityForce(Real mass, const Mat3& Rworld) const { return v6(Vec3(0), glm::transpose(Rworld) * (mass * gravity_)); }

    // Generalized actuator forces for a joint's DOFs.
    void jointTorques(const Joint& j, Real tau[3]) const {
        for (int a = 0; a < 3; ++a) tau[a] = 0;
        const Actuator& ac = j.actuator;
        if (j.dof == 1) {
            if (ac.mode == ActuatorMode::Torque) tau[0] = ac.torque;
            else if (ac.mode == ActuatorMode::PDTarget) tau[0] = ac.kp * (ac.target - j.q[0]) + ac.kd * (ac.targetVel - j.qd[0]);
        } else if (j.dof == 3) {
            if (ac.mode == ActuatorMode::Torque) { for (int a = 0; a < 3; ++a) tau[a] = ac.ballTorque[a]; }
            else if (ac.mode == ActuatorMode::PDTarget) {
                const Vec3 e = quatToRotvec(glm::normalize(ac.ballTarget * glm::conjugate(j.locRot)));  // child-frame error
                for (int a = 0; a < 3; ++a) tau[a] = ac.kp * e[a] - ac.kd * j.qd[a];
            }
        }
        if (ac.maxTorque > 0) for (int a = 0; a < j.dof; ++a) tau[a] = std::max(-ac.maxTorque, std::min(ac.maxTorque, tau[a]));
    }

    void computeAccelerations() {
        const size_t n = links_.size(), nj = joints_.size();
        qddot_.assign(static_cast<size_t>(ndof_), Real(0));
        std::vector<Mat6> Xup_(n), IA(n);
        std::vector<Vec6> v(n), c(n), pA(n), a(n);
        std::vector<Mat3> Rw(n);
        std::vector<std::array<Vec6, 3>> Uc(nj);
        std::vector<std::array<Real, 9>> Dinv(nj);
        std::vector<std::array<Real, 3>> uv(nj);

        for (size_t i = 0; i < n; ++i) {                         // Pass 1: base → tips
            const Link& L = links_[i];
            if (L.parent < 0) { Rw[i] = glm::mat3_cast(L.world.rotation); v[i] = isFloatingRoot(i) ? baseTwist_ : Vec6{}; c[i] = Vec6{}; }
            else {
                const Joint& j = joints_[L.jointIndex];
                Mat3 R_cp; Vec3 p_cp; jointRel(j, R_cp, p_cp);
                Xup_[i] = plux(glm::transpose(R_cp), p_cp);
                Rw[i] = Rw[L.parent] * R_cp;
                Vec6 vJ{}; for (int b = 0; b < j.dof; ++b) vJ = addV(vJ, scale(j.Scol[b], j.qd[b]));
                v[i] = addV(mul(Xup_[i], v[L.parent]), vJ);
                c[i] = (j.dof > 0) ? mul(crm(v[i]), vJ) : Vec6{};
            }
            IA[i] = L.I;
            pA[i] = addV(scale(mul(transpose(crm(v[i])), mul(L.I, v[i])), Real(-1)),
                         scale(gravityForce(L.mass, Rw[i]), Real(-1)));   // crf(v)Iv − f_grav
        }

        for (size_t ri = 0; ri < n; ++ri) {                      // Pass 2: tips → base
            const size_t i = n - 1 - ri;
            const Link& L = links_[i];
            if (L.parent < 0) continue;
            const Joint& j = joints_[L.jointIndex];
            const int ji = L.jointIndex, dof = j.dof;
            Mat6 Ia = IA[i]; Vec6 pa = pA[i];
            if (dof > 0) {
                Real tau[3]; jointTorques(j, tau);
                std::vector<Real> D(static_cast<size_t>(dof) * dof, 0);
                for (int aa = 0; aa < dof; ++aa) { Uc[ji][aa] = mul(IA[i], j.Scol[aa]); uv[ji][aa] = tau[aa] - dot(j.Scol[aa], pA[i]); }
                for (int aa = 0; aa < dof; ++aa) for (int bb = 0; bb < dof; ++bb) D[aa * dof + bb] = dot(j.Scol[aa], Uc[ji][bb]);
                std::vector<Real> Di; invertDense(D, dof, Di);
                for (int aa = 0; aa < dof; ++aa) for (int bb = 0; bb < dof; ++bb) Dinv[ji][aa * 3 + bb] = Di[aa * dof + bb];
                for (int aa = 0; aa < dof; ++aa) for (int bb = 0; bb < dof; ++bb) Ia = subM(Ia, outerScaled(Uc[ji][aa], Uc[ji][bb], Dinv[ji][aa * 3 + bb]));
                pa = addV(pA[i], mul(Ia, c[i]));
                for (int aa = 0; aa < dof; ++aa) { Real du = 0; for (int bb = 0; bb < dof; ++bb) du += Dinv[ji][aa * 3 + bb] * uv[ji][bb]; pa = addV(pa, scale(Uc[ji][aa], du)); }
            }
            const Mat6 XT = transpose(Xup_[i]);
            IA[L.parent] = addM(IA[L.parent], mul(mul(XT, Ia), Xup_[i]));
            pA[L.parent] = addV(pA[L.parent], mul(XT, pa));
        }

        for (size_t i = 0; i < n; ++i) {                         // Pass 3: base → tips
            const Link& L = links_[i];
            if (L.parent < 0) { if (isFloatingRoot(i)) { a[i] = solve6(IA[i], scale(pA[i], Real(-1))); baseAccel_ = a[i]; } else a[i] = Vec6{}; continue; }
            const Joint& j = joints_[L.jointIndex];
            const int ji = L.jointIndex, dof = j.dof;
            Vec6 ap = addV(mul(Xup_[i], a[L.parent]), c[i]);
            a[i] = ap;
            for (int aa = 0; aa < dof; ++aa) {
                Real qdd = 0; for (int bb = 0; bb < dof; ++bb) qdd += Dinv[ji][aa * 3 + bb] * (uv[ji][bb] - dot(Uc[ji][bb], ap));
                qddot_[j.qIndex + aa] = qdd;
                a[i] = addV(a[i], scale(j.Scol[aa], qdd));
            }
        }
    }

    // Nearest generalized DOF at or above `link` (skipping DOF-less fixed joints); the base's last
    // DOF (5) if floating, else -1. Used to build the DOF-ancestor tree.
    int ancestorDofOfLink(int link) const {
        int k = link;
        while (k >= 0) {
            if (links_[k].parent < 0) return (floating_ && k == rootIndex_) ? 5 : -1;
            const Joint& jk = joints_[links_[k].jointIndex];
            if (jk.dof > 0) return jk.qIndex + jk.dof - 1;
            k = links_[k].parent;
        }
        return -1;
    }

    // Sparse LDLᵀ factorization of H (row-major, in place): H = M D Mᵀ with M unit-lower-triangular
    // whose fill follows the DOF-ancestor tree — only ancestor entries are touched (Featherstone
    // §6.5), so this is ~O(ndof·depth²) instead of the dense O(ndof³) inverse. Returns false on a
    // non-positive pivot (⇒ caller falls back to the dense inverse).
    bool factorizeSparse(std::vector<Real>& H) const {
        const int n = ndof_;
        for (int k = n - 1; k >= 0; --k) {
            if (H[k * n + k] < Real(1e-12)) return false;
            int i = dofParent_[k];
            while (i >= 0) {
                const Real a = H[k * n + i] / H[k * n + k];
                int j = i;
                while (j >= 0) { H[i * n + j] -= a * H[k * n + j]; j = dofParent_[j]; }
                H[k * n + i] = a;
                i = dofParent_[i];
            }
        }
        return true;
    }
    // Solve H x = b in place (b←x) with the sparse factors from factorizeSparse — sparse forward
    // (M), diagonal (D), and backward (Mᵀ) substitution over ancestor chains only.
    void solveSparse(const std::vector<Real>& LD, std::vector<Real>& b) const {
        const int n = ndof_;
        // Mᵀ q = b : descending, push each finalized b[k] to its ancestors.
        for (int k = n - 1; k >= 0; --k) { int j = dofParent_[k]; while (j >= 0) { b[j] -= LD[k * n + j] * b[k]; j = dofParent_[j]; } }
        for (int i = 0; i < n; ++i) b[i] /= LD[i * n + i];                       // D p = q
        // M x = p : ascending, pull from already-solved ancestors.
        for (int i = 0; i < n; ++i) { int j = dofParent_[i]; while (j >= 0) { b[i] -= LD[i * n + j] * b[j]; j = dofParent_[j]; } }
    }

    // CRBA → dense joint-space inertia H (row-major ndof×ndof).
    std::vector<Real> buildMassMatrix() const {
        const size_t n = links_.size();
        std::vector<Mat6> Xup_(n, Mat6::zero()), Ic(n);
        for (size_t i = 0; i < n; ++i) { Ic[i] = links_[i].I; if (links_[i].parent >= 0) Xup_[i] = Xup(joints_[links_[i].jointIndex]); }
        for (size_t ri = 0; ri < n; ++ri) { const size_t i = n - 1 - ri; if (links_[i].parent >= 0) { const Mat6 XT = transpose(Xup_[i]); Ic[links_[i].parent] = addM(Ic[links_[i].parent], mul(mul(XT, Ic[i]), Xup_[i])); } }
        const int nd = ndof_;
        std::vector<Real> H(static_cast<size_t>(nd) * nd, Real(0));
        if (floating_ && rootIndex_ >= 0) for (int aa = 0; aa < 6; ++aa) for (int bb = 0; bb < 6; ++bb) H[aa * nd + bb] = Ic[rootIndex_].m[aa][bb];
        for (size_t i = 0; i < n; ++i) {
            const Link& L = links_[i];
            if (L.parent < 0 || joints_[L.jointIndex].dof == 0) continue;
            const Joint& j = joints_[L.jointIndex];
            for (int aa = 0; aa < j.dof; ++aa) {
                const int qia = j.qIndex + aa;
                Vec6 F = mul(Ic[i], j.Scol[aa]);
                for (int bb = 0; bb < j.dof; ++bb) H[qia * nd + (j.qIndex + bb)] = dot(j.Scol[bb], F);
                int k = static_cast<int>(i);
                while (links_[k].parent >= 0) {
                    F = mul(transpose(Xup_[k]), F);
                    const int p = links_[k].parent;
                    if (links_[p].parent < 0) { if (floating_) for (int cc = 0; cc < 6; ++cc) { H[qia * nd + cc] = F.d[cc]; H[cc * nd + qia] = F.d[cc]; } }
                    else { const Joint& jp = joints_[links_[p].jointIndex]; for (int cc = 0; cc < jp.dof; ++cc) { const Real h = dot(F, jp.Scol[cc]); H[qia * nd + (jp.qIndex + cc)] = h; H[(jp.qIndex + cc) * nd + qia] = h; } }
                    k = p;
                }
            }
        }
        return H;
    }

    // ---- contacts (E1) ----
    struct Contact { int link; Vec3 point; Vec3 normal; Real pen; Real friction; uint32_t key; };

    // Stable per-contact id (for warm-starting): link | plane | feature.
    static uint32_t contactKey(int link, int plane, int feature) {
        return (static_cast<uint32_t>(link) << 16) | (static_cast<uint32_t>(plane) << 6) | static_cast<uint32_t>(feature);
    }
    std::vector<Contact> detectContacts() const {
        std::vector<Contact> out;
        struct WPlane { Vec3 n; Real off; Real fric; };
        std::vector<WPlane> planes;
        for (const Link& s : links_)
            if (s.type == BodyType::Static && s.collider.type == ColliderDesc::Type::Plane) {
                const Mat3 R = glm::mat3_cast(s.world.rotation); const Vec3 nn = R * s.collider.plane.normal;
                planes.push_back({ nn, s.collider.plane.offset + glm::dot(nn, s.world.position), s.material.friction });
            }
        if (planes.empty()) return out;
        auto sphereVs = [&](int link, const Vec3& c, Real radius, Real mat, int feature) {
            for (size_t pi = 0; pi < planes.size(); ++pi) { const WPlane& p = planes[pi]; const Real dist = glm::dot(p.n, c) - p.off;
                if (dist < radius) out.push_back({ link, c - p.n * dist, p.n, radius - dist, std::sqrt(std::max(Real(0), mat * p.fric)),
                                                   contactKey(link, static_cast<int>(pi), feature) }); }
        };
        for (size_t i = 0; i < links_.size(); ++i) {
            const Link& L = links_[i]; if (L.type != BodyType::Dynamic) continue;
            const Vec3 c = L.world.position; const Mat3 R = glm::mat3_cast(L.world.rotation); const int li = static_cast<int>(i);
            switch (L.collider.type) {
                case ColliderDesc::Type::Sphere: sphereVs(li, c, L.collider.sphere.radius, L.material.friction, 0); break;
                case ColliderDesc::Type::Capsule: { const Real hh = L.collider.capsule.halfHeight, r = L.collider.capsule.radius;
                    sphereVs(li, c + R * Vec3(0, hh, 0), r, L.material.friction, 0); sphereVs(li, c - R * Vec3(0, hh, 0), r, L.material.friction, 1); break; }
                case ColliderDesc::Type::Box: { const Vec3 hE = L.collider.box.halfExtents; int corner = 0;
                    for (int sx = -1; sx <= 1; sx += 2) for (int sy = -1; sy <= 1; sy += 2) for (int sz = -1; sz <= 1; sz += 2, ++corner) {
                        const Vec3 cp = c + R * Vec3(sx * hE.x, sy * hE.y, sz * hE.z);
                        for (size_t pi = 0; pi < planes.size(); ++pi) { const WPlane& p = planes[pi]; const Real dist = glm::dot(p.n, cp) - p.off;
                            if (dist < 0) out.push_back({ li, cp, p.n, -dist, std::sqrt(std::max(Real(0), L.material.friction * p.fric)),
                                                          contactKey(li, static_cast<int>(pi), corner) }); } }
                    break; }
                default: sphereVs(li, c, Real(0.1), L.material.friction, 0); break;
            }
        }
        // #3 Contact-manifold reduction: keep at most the K deepest contacts per (link, plane) —
        // caps pathological counts (deeply-penetrating boxes/convex faces) while preserving support.
        constexpr int kMaxPerManifold = 4;
        if (out.size() > static_cast<size_t>(kMaxPerManifold)) {
            std::stable_sort(out.begin(), out.end(), [](const Contact& a, const Contact& b) {
                const uint32_t ga = a.key >> 6, gb = b.key >> 6;       // group = link|plane
                return ga != gb ? ga < gb : a.pen > b.pen;             // within a group, deepest first
            });
            std::vector<Contact> reduced; reduced.reserve(out.size());
            uint32_t group = 0xFFFFFFFFu; int count = 0;
            for (const Contact& con : out) {
                const uint32_t g = con.key >> 6;
                if (g != group) { group = g; count = 0; }
                if (count++ < kMaxPerManifold) reduced.push_back(con);
            }
            out.swap(reduced);
        }
        return out;
    }

    std::vector<Real> jacRow(int link, const Vec3& point, const Vec3& dir) const {
        std::vector<Real> row(static_cast<size_t>(ndof_), Real(0));
        int k = link;
        while (links_[k].parent >= 0) {
            const Joint& jk = joints_[links_[k].jointIndex];
            const Mat3 Rk = glm::mat3_cast(links_[k].world.rotation);
            const Vec3 anchorW = links_[k].world.position + Rk * jk.anchorC;
            for (int a = 0; a < jk.dof; ++a) { const Vec3 axisW = Rk * jk.axis[a]; row[jk.qIndex + a] += glm::dot(dir, glm::cross(axisW, point - anchorW)); }
            k = links_[k].parent;
        }
        if (floating_ && rootIndex_ >= 0) {
            const Mat3 Rwb = glm::mat3_cast(links_[rootIndex_].world.rotation); const Vec3 comBase = links_[rootIndex_].world.position;
            for (int a = 0; a < 3; ++a) { const Vec3 na(Rwb[a]); row[a] = glm::dot(dir, glm::cross(na, point - comBase)); row[3 + a] = glm::dot(dir, na); }
        }
        return row;
    }
    static Real dotN(const std::vector<Real>& a, const std::vector<Real>& b) { Real s = 0; for (size_t i = 0; i < a.size(); ++i) s += a[i] * b[i]; return s; }

    void solveContacts(Real h) {
        const std::vector<Contact> contacts = detectContacts();
        if (contacts.empty()) return;
        const int nd = ndof_;
        std::vector<Real> H = buildMassMatrix();
        bool sparse = sparseOk_;
        std::vector<Real> Hinv;
        if (sparse) { if (!factorizeSparse(H)) sparse = false; }   // H ← LDLᵀ factors in place
        if (!sparse) { std::vector<Real> Hd = buildMassMatrix(); if (!invertDense(Hd, nd, Hinv)) return; }
        auto applyHinv = [&](const std::vector<Real>& J, std::vector<Real>& out) {
            if (sparse) { out = J; solveSparse(H, out); }
            else { out.assign(static_cast<size_t>(nd), Real(0)); for (int i = 0; i < nd; ++i) { Real s = 0; for (int j = 0; j < nd; ++j) s += Hinv[i * nd + j] * J[j]; out[i] = s; } }
        };


        std::vector<Real> qd(static_cast<size_t>(nd), Real(0));
        for (int d = 0; d < baseDof_; ++d) qd[d] = baseTwist_.d[d];
        for (const Joint& j : joints_) for (int a = 0; a < j.dof; ++a) qd[j.qIndex + a] = j.qd[a];

        struct Row { std::vector<Real> Jn, Jt1, Jt2, JnHi, Jt1Hi, Jt2Hi; Real An, At1, At2, Atc, biasN, fric; uint32_t key; Real ln = 0, l1 = 0, l2 = 0; };
        std::vector<Row> rows; rows.reserve(contacts.size());
        constexpr Real kBeta = Real(0.2), kSlop = Real(0.001);
        for (const Contact& c : contacts) {
            Vec3 t1, t2; basisPerp(c.normal, t1, t2);
            Row r;
            r.Jn = jacRow(c.link, c.point, c.normal); r.Jt1 = jacRow(c.link, c.point, t1); r.Jt2 = jacRow(c.link, c.point, t2);
            applyHinv(r.Jn, r.JnHi); applyHinv(r.Jt1, r.Jt1Hi); applyHinv(r.Jt2, r.Jt2Hi);
            r.An = dotN(r.Jn, r.JnHi); r.At1 = dotN(r.Jt1, r.Jt1Hi); r.At2 = dotN(r.Jt2, r.Jt2Hi);
            r.Atc = dotN(r.Jt1, r.Jt2Hi);                              // #4 friction-block coupling
            r.biasN = -(kBeta / h) * std::max(Real(0), c.pen - kSlop); r.fric = c.friction; r.key = c.key;
            rows.push_back(std::move(r));
        }

        // #2 Warm-start: seed each contact's impulses from the previous substep/step and apply them
        // up front, so the PGS starts near the solution (resting contacts converge in a few iters).
        for (Row& r : rows) {
            const auto it = impulseCache_.find(r.key);
            if (it == impulseCache_.end()) continue;
            r.ln = it->second[0]; r.l1 = it->second[1]; r.l2 = it->second[2];
            for (int i = 0; i < nd; ++i) qd[i] += r.JnHi[i] * r.ln + r.Jt1Hi[i] * r.l1 + r.Jt2Hi[i] * r.l2;
        }

        constexpr int kIters = 12;                                    // fewer iters: warm-started + block friction
        for (int it = 0; it < kIters; ++it) {
            for (Row& r : rows) {
                if (r.An > kEps) {                                    // normal (λ ≥ 0)
                    const Real vn = dotN(r.Jn, qd);
                    Real dl = -(vn + r.biasN) / r.An;
                    const Real nl = std::max(Real(0), r.ln + dl);
                    dl = nl - r.ln; r.ln = nl;
                    for (int i = 0; i < nd; ++i) qd[i] += r.JnHi[i] * dl;
                }
                // #4 friction as a coupled 2×2 block with circular cone projection (radius μλn).
                const Real muN = r.fric * r.ln;
                const Real vt1 = dotN(r.Jt1, qd), vt2 = dotN(r.Jt2, qd);
                const Real det = r.At1 * r.At2 - r.Atc * r.Atc;
                Real d1, d2;
                if (std::fabs(det) > kEps) {
                    d1 = -(r.At2 * vt1 - r.Atc * vt2) / det;
                    d2 = -(r.At1 * vt2 - r.Atc * vt1) / det;
                } else {
                    d1 = (r.At1 > kEps) ? -vt1 / r.At1 : Real(0);
                    d2 = (r.At2 > kEps) ? -vt2 / r.At2 : Real(0);
                }
                Real n1 = r.l1 + d1, n2 = r.l2 + d2;
                const Real mag = std::sqrt(n1 * n1 + n2 * n2);
                if (mag > muN && mag > kEps) { const Real s = muN / mag; n1 *= s; n2 *= s; }
                const Real ad1 = n1 - r.l1, ad2 = n2 - r.l2; r.l1 = n1; r.l2 = n2;
                for (int i = 0; i < nd; ++i) qd[i] += r.Jt1Hi[i] * ad1 + r.Jt2Hi[i] * ad2;
            }
        }

        // Persist impulses for next-substep warm-starting (stale keys drop out).
        std::unordered_map<uint32_t, std::array<Real, 3>> next;
        next.reserve(rows.size());
        for (const Row& r : rows) next[r.key] = { r.ln, r.l1, r.l2 };
        impulseCache_.swap(next);

        for (int d = 0; d < baseDof_; ++d) baseTwist_.d[d] = qd[d];
        for (Joint& j : joints_) for (int a = 0; a < j.dof; ++a) j.qd[a] = qd[j.qIndex + a];
    }

    void integrateBasePose(Real h) {
        const Mat3 Rw = glm::mat3_cast(baseQuat_);
        basePos_ += (Rw * v6lin(baseTwist_)) * h;
        baseQuat_ = integrateQuat(baseQuat_, Rw * v6ang(baseTwist_), h);
    }

    void updatePoses() {
        for (size_t i = 0; i < links_.size(); ++i) {
            Link& L = links_[i];
            if (isFloatingRoot(i)) { L.world.position = basePos_; L.world.rotation = baseQuat_; }
            else if (L.parent < 0) { L.world.position = L.pos0; L.world.rotation = L.quat0; }
            else { const Joint& j = joints_[L.jointIndex]; Mat3 R_cp; Vec3 p_cp; jointRel(j, R_cp, p_cp); const Link& P = links_[L.parent];
                L.world.rotation = glm::normalize(P.world.rotation * j.relRot()); L.world.position = P.world.position + glm::mat3_cast(P.world.rotation) * p_cp; }
            poses_[i] = L.world;
        }
    }
    void computeVelocities() {
        for (size_t i = 0; i < links_.size(); ++i) {
            Link& L = links_[i];
            if (L.parent < 0) L.v = isFloatingRoot(i) ? baseTwist_ : Vec6{};
            else { const Joint& j = joints_[L.jointIndex]; Vec6 vJ{}; for (int a = 0; a < j.dof; ++a) vJ = addV(vJ, scale(j.Scol[a], j.qd[a])); L.v = addV(mul(Xup(j), links_[L.parent].v), vJ); }
            const Mat3 Rw = glm::mat3_cast(L.world.rotation);
            angVel_[i] = Rw * v6ang(L.v); linVel_[i] = Rw * v6lin(L.v);
        }
    }
    void writeJointStates() {
        for (size_t i = 0; i < joints_.size(); ++i) { jointStates_[i].q = (joints_[i].dof == 1) ? joints_[i].q[0] : Real(0); jointStates_[i].qd = (joints_[i].dof == 1) ? joints_[i].qd[0] : Real(0); }
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
    Vec6 baseTwist_{};
    Vec6 baseAccel_{};
    std::vector<Real> qddot_;
    int  ndof_ = 0, baseDof_ = 0;
    std::vector<int> dofParent_;   // DOF-ancestor tree (Featherstone sparse-H): parent DOF or -1
    bool sparseOk_ = false;        // dofParent_ is a valid elimination order (dofParent[i] < i)

    std::vector<Link>  links_;
    std::vector<Joint> joints_;
    std::vector<engine::Transform> poses_;
    std::vector<Vec3>  linVel_, angVel_;
    std::vector<JointState> jointStates_;
    std::vector<ContactEvent> contacts_;
    std::unordered_map<uint32_t, std::array<Real, 3>> impulseCache_;   // warm-start: key → (λn,λt1,λt2)
};

} // namespace

std::unique_ptr<PhysicsWorld> createFeatherstoneWorld(const WorldDef& def) {
    return std::make_unique<FeatherstoneWorld>(def);
}

} // namespace engine::physics
