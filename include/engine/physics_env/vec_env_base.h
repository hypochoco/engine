//
//  vec_env_base.h
//  engine::physics_env
//
//  The common vectorized-env interface shared by the three RL backends — physics_env::VecEnv
//  (maximal/reduced PhysicsWorld), DiffVecEnv (CPU differentiable ABA), and CudaVecEnv (GPU
//  differentiable ABA) — so a single EnvConfig can select any of them via createVecEnv (env_factory.h)
//  and downstream code holds one `IVecEnv`. The contract is the flat SoA batch surface: `actions()`
//  (caller writes) + `observations()` (packer output), plus reset / reset_masked / step. Dispatch is
//  once per control step over N envs, so the virtual call is negligible next to the batched work.
//

#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

namespace engine::physics_env {

class IVecEnv {
public:
    virtual ~IVecEnv() = default;

    virtual size_t numEnvs() const = 0;
    virtual size_t actDim()  const = 0;
    virtual size_t obsDim()  const = 0;

    virtual std::span<float>       actions()            = 0;   // host mirror [N*actDim] (caller writes)
    virtual std::span<const float> observations() const = 0;   // host mirror [N*obsDim] (packer output)

    virtual void reset(uint64_t seed) = 0;                                  // reset all envs, refresh obs
    virtual void resetMasked(std::span<const uint8_t> mask, uint64_t seed) = 0;  // reset flagged envs
    virtual void step() = 0;                                                // actions -> step -> obs
};

} // namespace engine::physics_env
