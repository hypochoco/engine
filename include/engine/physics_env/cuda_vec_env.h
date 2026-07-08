//
//  cuda_vec_env.h
//  engine::physics_env
//
//  Phase 4 of the CUDA port: a GPU-resident vectorized env matching the physics_env::VecEnv contract
//  (flat actions()/observations() SoA batches, reset/step) but running the batched reduced +
//  smoothed-contact ABA on the device (Phase 3 BatchedForward) with obs packed on-GPU. The obs layout
//  is byte-for-byte the Environment::packDefaultObs contract:
//      root pos(3) | root quat wxyz(4) | root linVel(3) | root angVel(3)
//      | joint q[ndof] | joint qd[ndof] | per-body contact flags[nBodies]
//  Actuation follows EnvConfig::actionMode (Torque or PDTarget) applied per-DOF on the GPU.
//
//  Buffers live in VRAM; actions()/observations() expose host mirrors (synced in step()/reset()) for
//  the existing contract, and deviceObservations()/deviceActions() expose the device pointers for a
//  future zero-copy PyTorch-GPU path (obs/actions never leaving the device — the review's precondition
//  for the end-to-end win). Reward/termination stay in the RL layer (as for the CPU VecEnv).
//
//  NVIDIA only (compiled under ENGINE_CUDA); the class is not declared otherwise.
//

#pragma once

#if defined(ENGINE_CUDA)

#include <cstdint>
#include <memory>
#include <span>
#include <vector>

#include "engine/physics/cuda/batched_forward.h"
#include "engine/physics_env/environment.h"

namespace engine::physics_env {

namespace diff = ::engine::physics::diff;
namespace cuda = ::engine::physics::cuda;

class CudaVecEnv {
public:
    CudaVecEnv(size_t numEnvs, const EnvConfig& config);
    ~CudaVecEnv();
    CudaVecEnv(const CudaVecEnv&) = delete;
    CudaVecEnv& operator=(const CudaVecEnv&) = delete;

    size_t numEnvs() const { return numEnvs_; }
    size_t actDim()  const { return actDim_; }
    size_t obsDim()  const { return obsDim_; }

    std::span<float>       actions()            { return actions_; }        // host mirror [N*actDim]
    std::span<const float> observations() const { return obs_; }            // host mirror [N*obsDim]

    // Device pointers (VRAM) for a zero-copy GPU-policy path.
    float*       deviceActions()            { return dActions_; }
    const float* deviceObservations() const { return dObs_; }

    void reset(uint64_t seed = 0);                       // reset all envs, refresh obs
    void resetMasked(std::span<const uint8_t> mask, uint64_t seed = 0);  // reset only flagged envs
    void step();                                         // actions() -> tau -> substeps -> pack obs

private:
    void uploadActions();
    void applyActionsToTau();     // action -> tau kernel (Torque / PDTarget)
    void packObs();               // obs kernel -> dObs_ -> obs_ (host)

    size_t numEnvs_ = 0, actDim_ = 0, obsDim_ = 0;
    int    numLinks_ = 0, numDof_ = 0;

    // sim knobs (from EnvConfig::sim)
    int    substeps_ = 8;
    double h_ = 1.0 / 480.0;
    float  gx_ = 0, gy_ = -9.81f, gz_ = 0;
    int    actionMode_ = 0;       // 0 Torque, 1 PDTarget
    float  kp_ = 0, kd_ = 0, maxTorque_ = 0;
    float  rootY0_ = 0;           // authored root height (initial basePos.y)

    std::unique_ptr<diff::FlatModel>      model_;    // host copy (for re-init)
    std::unique_ptr<cuda::BatchedForward> bf_;       // device state/model/tau + substep kernel

    float* dActions_ = nullptr;   // device [N*actDim]
    float* dObs_     = nullptr;   // device [N*obsDim]
    diff::DiffState<float>* dInit_ = nullptr;   // device [N] authored init (for reset / reset_masked)
    uint8_t*                dMask_ = nullptr;   // device [N] reset mask
    std::vector<float> actions_;  // host mirror [N*actDim]
    std::vector<float> obs_;      // host mirror [N*obsDim]
    std::vector<diff::DiffState<float>> initStates_;  // host initial state (per env), for reset
};

} // namespace engine::physics_env

#endif  // ENGINE_CUDA
