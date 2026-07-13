//
//  vec_env.h
//  engine::physics_env
//
//  Phase D2: a vectorized batch of N independent `Environment`s stepped in parallel — the ML
//  throughput lever (parallel *worlds*, not intra-world threads). Each env owns a single-threaded
//  `PhysicsWorld`, and `step()` runs them across a `ThreadPool` (no nested parallelFor). State +
//  actions are contiguous SoA batches — `actions()[N*actDim]` (caller writes) and
//  `observations()[N*obsDim]` (default packer output) — so a downstream C-ABI / Python layer can
//  read/write them zero-copy. Per-env determinism is preserved (disjoint envs, fixed per-env seed).
//

#pragma once

#include <cstdint>
#include <memory>
#include <span>
#include <vector>

#include "engine/physics_env/environment.h"
#include "engine/physics_env/vec_env_base.h"

namespace engine::core { class ThreadPool; }

namespace engine::physics_env {

class VecEnv : public IVecEnv {
public:
    // `pool == nullptr` steps serially (still valid; used for the determinism reference).
    VecEnv(size_t numEnvs, const EnvConfig& config, engine::core::ThreadPool* pool = nullptr);

    size_t numEnvs() const override { return envs_.size(); }
    size_t actDim()  const override { return actDim_; }
    size_t obsDim()  const override { return obsDim_; }

    // Contiguous SoA batches. Row i of actions is [i*actDim, (i+1)*actDim).
    std::span<float>        actions()            override { return actions_; }
    std::span<const float>  observations() const override { return obs_; }

    // Reset every env (env i uses seed + i for reproducibility) and refresh observations.
    void reset(uint64_t seed) override;
    // Reset only envs whose mask byte is non-zero (episode boundaries) + refresh their obs.
    void resetMasked(std::span<const uint8_t> mask, uint64_t seed) override;
    // Apply actions() → step all envs (parallel across the pool) → refresh observations().
    void step() override;

    Environment&       env(size_t i)       { return *envs_[i]; }
    const Environment& env(size_t i) const { return *envs_[i]; }

private:
    void packObs(size_t i);
    template <class F> void forEachEnv(F&& f);

    std::vector<std::unique_ptr<Environment>> envs_;
    engine::core::ThreadPool*                 pool_ = nullptr;
    size_t                                    actDim_ = 0;
    size_t                                    obsDim_ = 0;
    std::vector<float>                        actions_;   // [N * actDim]
    std::vector<float>                        obs_;       // [N * obsDim]
};

} // namespace engine::physics_env
