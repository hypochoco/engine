//
//  linalg.h
//  engine::physics::diff
//
//  Scalar-generic linear algebra for the differentiable dynamics (Phase F1c). Minimal 3D (V3/M3)
//  and 6D spatial (V6/M6) types + the operations the Articulated-Body Algorithm needs, all
//  templated on the scalar so the *same* code runs as `double` (value) or `Dual<N>` (gradients).
//  Orientation is carried as rotation matrices via `rodrigues` (no quaternions) — smooth and
//  normalization-free, which keeps it cleanly differentiable. Mirrors the spatial algebra in
//  src/physics/backends/reduced/featherstone_world.cpp (angular-first 6-vectors [ω; v]).
//

#pragma once

#include "engine/physics/diff/dual.h"

namespace engine::physics::diff {

// ---------------------------------------------------------------- 3D ---------------------------
template <class S> struct V3 { S x{}, y{}, z{}; };
template <class S> struct M3 { S m[3][3]{}; };   // row-major math indexing m[row][col]

template <class S> V3<S> operator+(const V3<S>& a, const V3<S>& b) { return { a.x + b.x, a.y + b.y, a.z + b.z }; }
template <class S> V3<S> operator-(const V3<S>& a, const V3<S>& b) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
template <class S> V3<S> operator*(const S& s, const V3<S>& a) { return { s * a.x, s * a.y, s * a.z }; }
template <class S> S dot(const V3<S>& a, const V3<S>& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
template <class S> V3<S> cross(const V3<S>& a, const V3<S>& b) {
    return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
}

template <class S> M3<S> skew(const V3<S>& v) {
    M3<S> r;
    r.m[0][0] = S(0);  r.m[0][1] = -v.z; r.m[0][2] =  v.y;
    r.m[1][0] =  v.z;  r.m[1][1] = S(0); r.m[1][2] = -v.x;
    r.m[2][0] = -v.y;  r.m[2][1] =  v.x; r.m[2][2] = S(0);
    return r;
}
template <class S> M3<S> identity3() {
    M3<S> r; for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) r.m[i][j] = S(i == j ? 1 : 0); return r;
}
template <class S> M3<S> operator*(const M3<S>& a, const M3<S>& b) {
    M3<S> r;
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) { S s = S(0); for (int k = 0; k < 3; ++k) s = s + a.m[i][k] * b.m[k][j]; r.m[i][j] = s; }
    return r;
}
template <class S> V3<S> operator*(const M3<S>& a, const V3<S>& v) {
    return { a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z,
             a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z,
             a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z };
}
template <class S> M3<S> operator+(const M3<S>& a, const M3<S>& b) {
    M3<S> r; for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) r.m[i][j] = a.m[i][j] + b.m[i][j]; return r;
}
template <class S> M3<S> scaled(const M3<S>& a, const S& s) {
    M3<S> r; for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) r.m[i][j] = a.m[i][j] * s; return r;
}
template <class S> M3<S> transpose(const M3<S>& a) {
    M3<S> r; for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) r.m[i][j] = a.m[j][i]; return r;
}
// Rodrigues: rotation by `angle` about unit `axis` ⇒ I + sinθ K + (1−cosθ) K².
template <class S> M3<S> rodrigues(const V3<S>& axis, const S& angle) {
    using std::sin; using std::cos;
    const M3<S> K = skew(axis);
    const S s = sin(angle), c = cos(angle), omc = S(1) - c;
    return identity3<S>() + scaled(K, s) + scaled(K * K, omc);
}

// ---------------------------------------------------------------- 6D spatial -------------------
template <class S> struct V6 { S d[6]{}; };            // [ω(0..2); v(3..5)]
template <class S> struct M6 { S m[6][6]{}; };

template <class S> V3<S> ang(const V6<S>& v) { return { v.d[0], v.d[1], v.d[2] }; }
template <class S> V3<S> lin(const V6<S>& v) { return { v.d[3], v.d[4], v.d[5] }; }
template <class S> V6<S> makeV6(const V3<S>& w, const V3<S>& v) { return { { w.x, w.y, w.z, v.x, v.y, v.z } }; }

template <class S> void setBlock(M6<S>& M, int r0, int c0, const M3<S>& B, bool neg = false) {
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) M.m[r0 + i][c0 + j] = neg ? -B.m[i][j] : B.m[i][j];
}
// Motion transform [[E,0],[−E·skew(r), E]].
template <class S> M6<S> plux(const M3<S>& E, const V3<S>& r) {
    M6<S> X; setBlock(X, 0, 0, E); setBlock(X, 3, 3, E); setBlock(X, 3, 0, E * skew(r), true); return X;
}
// Motion cross [[skew(ω),0],[skew(v), skew(ω)]].
template <class S> M6<S> crm(const V6<S>& v) {
    M6<S> M; const M3<S> W = skew(ang(v)); setBlock(M, 0, 0, W); setBlock(M, 3, 3, W); setBlock(M, 3, 0, skew(lin(v))); return M;
}
// Spatial inertia with COM at the frame origin: [[Ic,0],[0, m·I]].
template <class S> M6<S> spatialInertia(const S& mass, const M3<S>& Ic) {
    M6<S> I; setBlock(I, 0, 0, Ic); I.m[3][3] = mass; I.m[4][4] = mass; I.m[5][5] = mass; return I;
}
template <class S> V6<S> operator*(const M6<S>& A, const V6<S>& x) {
    V6<S> r; for (int i = 0; i < 6; ++i) { S s = S(0); for (int j = 0; j < 6; ++j) s = s + A.m[i][j] * x.d[j]; r.d[i] = s; } return r;
}
template <class S> M6<S> operator*(const M6<S>& A, const M6<S>& B) {
    M6<S> r; for (int i = 0; i < 6; ++i) for (int j = 0; j < 6; ++j) { S s = S(0); for (int k = 0; k < 6; ++k) s = s + A.m[i][k] * B.m[k][j]; r.m[i][j] = s; } return r;
}
template <class S> M6<S> transpose(const M6<S>& A) {
    M6<S> r; for (int i = 0; i < 6; ++i) for (int j = 0; j < 6; ++j) r.m[i][j] = A.m[j][i]; return r;
}
template <class S> M6<S> operator+(const M6<S>& A, const M6<S>& B) {
    M6<S> r; for (int i = 0; i < 6; ++i) for (int j = 0; j < 6; ++j) r.m[i][j] = A.m[i][j] + B.m[i][j]; return r;
}
template <class S> M6<S> operator-(const M6<S>& A, const M6<S>& B) {
    M6<S> r; for (int i = 0; i < 6; ++i) for (int j = 0; j < 6; ++j) r.m[i][j] = A.m[i][j] - B.m[i][j]; return r;
}
template <class S> V6<S> operator+(const V6<S>& a, const V6<S>& b) {
    V6<S> r; for (int i = 0; i < 6; ++i) r.d[i] = a.d[i] + b.d[i]; return r;
}
template <class S> V6<S> scaled(const V6<S>& a, const S& s) {
    V6<S> r; for (int i = 0; i < 6; ++i) r.d[i] = a.d[i] * s; return r;
}
template <class S> S dot(const V6<S>& a, const V6<S>& b) { S s = S(0); for (int i = 0; i < 6; ++i) s = s + a.d[i] * b.d[i]; return s; }
template <class S> M6<S> outerScaled(const V6<S>& a, const V6<S>& b, const S& s) {   // (a bᵀ)·s
    M6<S> r; for (int i = 0; i < 6; ++i) for (int j = 0; j < 6; ++j) r.m[i][j] = a.d[i] * b.d[j] * s; return r;
}

// ---- lifting double constants into the scalar type S (Dual with zero derivative) --------------
template <class S> V3<S> lift(const V3<double>& v) { return { S(v.x), S(v.y), S(v.z) }; }
template <class S> M3<S> lift(const M3<double>& a) {
    M3<S> r; for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) r.m[i][j] = S(a.m[i][j]); return r;
}

// ---- SO(3) exponential map: rotation matrix from a rotation vector (small-angle guarded) -------
template <class S> M3<S> expSO3(const V3<S>& w) {
    using std::sin; using std::cos; using std::sqrt;
    const M3<S> K = skew(w);
    const S theta = sqrt(dot(w, w));
    if (theta < S(1e-8)) return identity3<S>() + K + scaled(K * K, S(0.5));   // I + K + K²/2
    const S a = sin(theta) / theta;                       // sinθ/θ
    const S b = (S(1) - cos(theta)) / (theta * theta);    // (1−cosθ)/θ²
    return identity3<S>() + scaled(K, a) + scaled(K * K, b);
}

// ---- small dense linear algebra (no pivoting; the ABA's D and base inertia are SPD) -----------
// Inverse of an n×n matrix (row-major, n ≤ 3) via Gauss-Jordan. Used for the ball joint's 3×3 D.
template <class S> void invertSmall(const S* A, int n, S* inv) {
    S M[9]; for (int i = 0; i < n * n; ++i) M[i] = A[i];
    for (int i = 0; i < n; ++i) for (int j = 0; j < n; ++j) inv[i * n + j] = S(i == j ? 1 : 0);
    for (int col = 0; col < n; ++col) {
        const S d = M[col * n + col];
        for (int j = 0; j < n; ++j) { M[col * n + j] = M[col * n + j] / d; inv[col * n + j] = inv[col * n + j] / d; }
        for (int r = 0; r < n; ++r) {
            if (r == col) continue;
            const S f = M[r * n + col];
            for (int j = 0; j < n; ++j) { M[r * n + j] = M[r * n + j] - f * M[col * n + j]; inv[r * n + j] = inv[r * n + j] - f * inv[col * n + j]; }
        }
    }
}
// Solve A x = b for a 6×6 SPD A (floating-base articulated inertia), no pivoting.
template <class S> V6<S> solveM6(const M6<S>& A, const V6<S>& b) {
    S M[6][7];
    for (int i = 0; i < 6; ++i) { for (int j = 0; j < 6; ++j) M[i][j] = A.m[i][j]; M[i][6] = b.d[i]; }
    for (int col = 0; col < 6; ++col) {
        const S d = M[col][col];
        for (int r = 0; r < 6; ++r) {
            if (r == col) continue;
            const S f = M[r][col] / d;
            for (int j = col; j < 7; ++j) M[r][j] = M[r][j] - f * M[col][j];
        }
    }
    V6<S> x; for (int i = 0; i < 6; ++i) x.d[i] = M[i][6] / M[i][i]; return x;
}

// Axial vector of the antisymmetric part of M (inverse of skew for a skew matrix).
template <class S> V3<S> vee(const M3<S>& M) {
    return { S(0.5) * (M.m[2][1] - M.m[1][2]), S(0.5) * (M.m[0][2] - M.m[2][0]), S(0.5) * (M.m[1][0] - M.m[0][1]) };
}

// SO(3) log map: rotation matrix → rotation vector (double; used for tangent-space finite differences).
inline V3<double> logSO3(const M3<double>& R) {
    double cth = 0.5 * (R.m[0][0] + R.m[1][1] + R.m[2][2] - 1.0);
    cth = cth > 1.0 ? 1.0 : (cth < -1.0 ? -1.0 : cth);
    const double theta = std::acos(cth);
    const V3<double> w = vee(R);                          // = sinθ · axis
    if (theta < 1e-8) return w;                           // small angle: log ≈ vee(R)
    const double s = std::sin(theta);
    return (theta / s) * w;
}

template <class S> V6<S> liftV6(const V6<double>& v) { V6<S> r; for (int i = 0; i < 6; ++i) r.d[i] = S(v.d[i]); return r; }

} // namespace engine::physics::diff
