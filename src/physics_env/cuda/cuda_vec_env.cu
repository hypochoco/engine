//
//  cuda_vec_env.cu
//  engine::physics_env
//
//  GPU-resident VecEnv (Phase 4; E1/E3 refactor). See cuda_vec_env.h. The action→tau (PD servo) and
//  obs packing are NO LONGER hand-written here — they call the SHARED, model-generic ENGINE_HD free
//  functions `diff::actionToTau` / `diff::packDefaultObs` (diff/env_ops.h), the exact same code the CPU
//  `DiffVecEnv` runs. So CPU and GPU share one actuation + observation implementation (no divergence —
//  the code review's MODERATE finding), just as the ABA is already shared. Stepping reuses
//  BatchedForward (the shared diff::diffSubstep). All state/actions/obs stay in VRAM; host mirrors are
//  synced for the span-based contract (deviceObservations()/deviceActions() expose the zero-copy path).
//

#include <cuda_runtime.h>

#include <algorithm>
#include <stdexcept>
#include <string>

#include "engine/physics_env/cuda_vec_env.h"
#include "engine/physics/diff/env_ops.h"
#include "engine/physics/diff/from_articulation.h"

namespace engine::physics_env {
namespace {

inline void cudaCheck(cudaError_t e, const char* what) {
    if (e != cudaSuccess)
        throw std::runtime_error(std::string("CUDA error in ") + what + ": " + cudaGetErrorString(e));
}

// One thread per env; all three kernels delegate to the shared host/device ops so there is a single
// implementation of actuation / observation / (reset) across CPU and GPU.
__global__ void actionToTauKernel(const diff::FlatModel* md, const diff::DiffState<float>* states,
                                  const float* actions, float* tau, int nEnv, int numDof,
                                  int actionMode, float kp, float kd, float maxTorque) {
    const int e = blockIdx.x * blockDim.x + threadIdx.x;
    if (e >= nEnv) return;
    diff::actionToTau(*md, states[e], actions + static_cast<size_t>(e) * numDof,
                      tau + static_cast<size_t>(e) * numDof, actionMode, kp, kd, maxTorque);
}

__global__ void packObsKernel(const diff::FlatModel* md, const diff::DiffState<float>* states,
                              float* obs, int nEnv, int obsDim) {
    const int e = blockIdx.x * blockDim.x + threadIdx.x;
    if (e >= nEnv) return;
    diff::packDefaultObs(*md, states[e], obs + static_cast<size_t>(e) * obsDim);
}

__global__ void resetMaskedKernel(diff::DiffState<float>* states, const diff::DiffState<float>* init,
                                  const uint8_t* mask, int nEnv) {
    const int e = blockIdx.x * blockDim.x + threadIdx.x;
    if (e >= nEnv) return;
    if (mask[e]) states[e] = init[e];
}

} // namespace

CudaVecEnv::CudaVecEnv(size_t numEnvs, const EnvConfig& config) : numEnvs_(numEnvs) {
    diff::DiffModel md = diff::articulationToDiffModel(config.articulation, diff::DiffContact::All);
    md.linearDamping  = static_cast<double>(config.sim.linearDamping);   // match the reduced backend's damping
    md.angularDamping = static_cast<double>(config.sim.angularDamping);
    md.contactIntegration = config.sim.contactSemiImplicit ? diff::ContactIntegration::SemiImplicit
                                                           : diff::ContactIntegration::Explicit;
    model_ = std::make_unique<diff::FlatModel>(diff::flatten(md));
    numLinks_ = model_->numLinks;
    numDof_   = model_->numDof;
    actDim_   = static_cast<size_t>(numDof_);
    obsDim_   = static_cast<size_t>(13 + 2 * numDof_ + numLinks_);

    substeps_   = config.sim.substeps > 0 ? config.sim.substeps : 1;
    h_          = static_cast<double>(config.sim.controlDt) / substeps_;
    gx_ = (float)config.sim.gravity.x; gy_ = (float)config.sim.gravity.y; gz_ = (float)config.sim.gravity.z;
    actionMode_ = (config.sim.actionMode == ActionMode::PDTarget) ? diff::kActionPDTarget : diff::kActionTorque;
    kp_ = (float)config.sim.kp; kd_ = (float)config.sim.kd; maxTorque_ = (float)config.sim.maxTorque;

    // Initial state: authored root pose (position + orientation), rest joints, zero velocity.
    const int root = diff::rootBodyIndex(md);
    const physics::BodyDef& rb = config.articulation.bodies[static_cast<size_t>(root)];
    diff::DiffState<float> s0 = diff::makeState<float>(md);
    s0.basePos = { (float)rb.position.x, (float)rb.position.y, (float)rb.position.z };
    s0.baseRot = diff::lift<float>(diff::glmToM3(glm::mat3_cast(rb.orientation)));
    rootY0_ = (float)rb.position.y;
    initStates_.assign(numEnvs_, s0);

    bf_ = std::make_unique<cuda::BatchedForward>(*model_, static_cast<int>(numEnvs_));

    cudaCheck(cudaMalloc(&dActions_, sizeof(float) * numEnvs_ * actDim_), "cudaMalloc actions");
    cudaCheck(cudaMalloc(&dObs_,     sizeof(float) * numEnvs_ * obsDim_), "cudaMalloc obs");
    cudaCheck(cudaMalloc(&dInit_,    sizeof(diff::DiffState<float>) * numEnvs_), "cudaMalloc init");
    cudaCheck(cudaMalloc(&dMask_,    sizeof(uint8_t) * numEnvs_), "cudaMalloc mask");
    cudaCheck(cudaMemcpy(dInit_, initStates_.data(), sizeof(diff::DiffState<float>) * numEnvs_,
                         cudaMemcpyHostToDevice), "upload init");
    actions_.assign(numEnvs_ * actDim_, 0.0f);
    obs_.assign(numEnvs_ * obsDim_, 0.0f);

    reset(0);
}

CudaVecEnv::~CudaVecEnv() {
    cudaFree(dActions_);
    cudaFree(dObs_);
    cudaFree(dInit_);
    cudaFree(dMask_);
}

void CudaVecEnv::uploadActions() {
    cudaCheck(cudaMemcpy(dActions_, actions_.data(), sizeof(float) * actions_.size(), cudaMemcpyHostToDevice), "uploadActions");
}

void CudaVecEnv::applyActionsToTau() {
    const int block = 128, grid = (static_cast<int>(numEnvs_) + block - 1) / block;
    actionToTauKernel<<<grid, block>>>(bf_->deviceModel(), bf_->deviceStates(), dActions_, bf_->deviceTau(),
                                       static_cast<int>(numEnvs_), numDof_, actionMode_, kp_, kd_, maxTorque_);
    cudaCheck(cudaGetLastError(), "actionToTauKernel");
}

void CudaVecEnv::packObs() {
    const int block = 128, grid = (static_cast<int>(numEnvs_) + block - 1) / block;
    packObsKernel<<<grid, block>>>(bf_->deviceModel(), bf_->deviceStates(), dObs_,
                                   static_cast<int>(numEnvs_), static_cast<int>(obsDim_));
    cudaCheck(cudaGetLastError(), "packObsKernel");
    cudaCheck(cudaMemcpy(obs_.data(), dObs_, sizeof(float) * obs_.size(), cudaMemcpyDeviceToHost), "downloadObs");
}

void CudaVecEnv::reset(uint64_t /*seed*/) {
    bf_->setStates(initStates_);                 // all envs to the authored initial state
    std::fill(actions_.begin(), actions_.end(), 0.0f);
    cudaCheck(cudaMemset(dActions_, 0, sizeof(float) * actions_.size()), "reset actions");
    packObs();
    bf_->synchronize();
}

void CudaVecEnv::resetMasked(std::span<const uint8_t> mask, uint64_t /*seed*/) {
    // Upload the mask, reset only flagged envs to the (device-resident) authored init, refresh obs.
    const size_t n = std::min(mask.size(), numEnvs_);
    cudaCheck(cudaMemset(dMask_, 0, sizeof(uint8_t) * numEnvs_), "clear mask");
    if (n) cudaCheck(cudaMemcpy(dMask_, mask.data(), sizeof(uint8_t) * n, cudaMemcpyHostToDevice), "upload mask");
    const int block = 128, grid = (static_cast<int>(numEnvs_) + block - 1) / block;
    resetMaskedKernel<<<grid, block>>>(bf_->deviceStates(), dInit_, dMask_, static_cast<int>(numEnvs_));
    cudaCheck(cudaGetLastError(), "resetMaskedKernel");
    packObs();
    bf_->synchronize();
}

void CudaVecEnv::step() {
    uploadActions();
    // Recompute the PD torque from the CURRENT device state every substep (mirrors the CPU DiffVecEnv
    // and the reduced backend). Holding tau fixed across substeps leaves a stale −kd·q̇ term that
    // destabilizes stiff PD-hold on compliant contact. (Perf: this is 2·substeps kernel launches; a
    // future optimization is to fuse actionToTau into the substep kernel — see batched_forward.cu.)
    const diff::V3<double> g{ gx_, gy_, gz_ };
    for (int s = 0; s < substeps_; ++s) {
        applyActionsToTau();
        bf_->step(1, h_, g);
    }
    packObs();
    bf_->synchronize();
}

} // namespace engine::physics_env
