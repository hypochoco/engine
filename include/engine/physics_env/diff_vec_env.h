//
//  diff_vec_env.h
//  engine::physics_env
//
//  CPU vectorized env over the DIFFERENTIABLE ABA + smoothed contact — the host counterpart of the
//  GPU CudaVecEnv, running the SAME templated code (diff::diffSubstep + the shared diff::actionToTau /
//  diff::packDefaultObs in diff/env_ops.h). This is what makes "CPU and CUDA are the same physics"
//  literally true and testable on the Mac (the review's core recommendation,
//  2026-07-08-cuda-port-code-review.md): the CPU RL env and the GPU RL env now share one dynamics +
//  actuation + observation implementation, differing only in float rounding order.
//
//  Contract matches physics_env::VecEnv (flat SoA actions()/observations(), reset/reset_masked/step)
//  so it is a drop-in RL backend. Distinct from physics_env::VecEnv, which runs the maximal/reduced
//  PGS PhysicsWorld backends; DiffVecEnv does NOT use PhysicsWorld — it steps diff::diffSubstep on a
//  per-env diff::DiffState<float>. Obs layout is the Environment::packDefaultObs contract.
//
//  Precision is float (matching the GPU kernel) so CPU↔GPU parity is exact-modulo-FMA.
//

#pragma once

#include <cstdint>
#include <span>
#include <vector>

#include "engine/physics/diff/articulated.h"
#include "engine/physics_env/environment.h"   // EnvConfig

namespace engine::core { class ThreadPool; }

namespace engine::physics_env {

namespace diff = ::engine::physics::diff;

class DiffVecEnv {
public:
    // `pool == nullptr` steps serially (the determinism reference; still valid).
    DiffVecEnv(size_t numEnvs, const EnvConfig& config, engine::core::ThreadPool* pool = nullptr);

    size_t numEnvs() const { return states_.size(); }
    size_t actDim()  const { return actDim_; }
    size_t obsDim()  const { return obsDim_; }

    std::span<float>       actions()            { return actions_; }         // [N*actDim]
    std::span<const float> observations() const { return obs_; }             // [N*obsDim]

    void reset(uint64_t seed);                                   // all envs → authored init, refresh obs
    void resetMasked(std::span<const uint8_t> mask, uint64_t seed);  // reset only flagged envs
    void step();                                                 // actions → tau → substeps → obs

    // Escape hatch (tests / RSI): the per-env diff state.
    const diff::DiffState<float>& state(size_t i) const { return states_[i]; }
    diff::DiffState<float>&       state(size_t i)       { return states_[i]; }

private:
    void packObs(size_t i);
    template <class F> void forEachEnv(F&& f);

    diff::DiffModel               model_;      // host authoring/working model (hd* accessors read it)
    int                           numLinks_ = 0, numDof_ = 0;
    size_t                        actDim_ = 0, obsDim_ = 0;
    int                           substeps_ = 1;
    double                        h_ = 1.0 / 60.0;
    diff::V3<double>              gravity_{ 0, -9.81, 0 };
    int                           actionMode_ = 0;               // 0 Torque, 1 PDTarget
    float                         kp_ = 0, kd_ = 0, maxTorque_ = 0;
    diff::DiffState<float>        init_;                          // authored initial state (per env)

    std::vector<diff::DiffState<float>> states_;
    std::vector<float>                  actions_;                // [N*actDim]
    std::vector<float>                  obs_;                    // [N*obsDim]
    engine::core::ThreadPool*           pool_ = nullptr;
};

} // namespace engine::physics_env
