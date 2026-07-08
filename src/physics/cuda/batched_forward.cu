//
//  batched_forward.cu
//  engine::physics::cuda
//
//  Implementation of BatchedForward (see batched_forward.h). The kernel is one thread per env; each
//  loads its DiffState<float> to registers/local, runs `substeps` of the shared diff::diffSubstep
//  reading the broadcast FlatModel from global memory, and writes the state back. No cross-env
//  communication ⇒ embarrassingly parallel, deterministic per env (same-device).
//

#include <cuda_runtime.h>

#include <cstdio>
#include <stdexcept>
#include <string>

#include "engine/physics/cuda/batched_forward.h"

namespace engine::physics::cuda {
namespace {

inline void cudaCheck(cudaError_t e, const char* what) {
    if (e != cudaSuccess)
        throw std::runtime_error(std::string("CUDA error in ") + what + ": " + cudaGetErrorString(e));
}

__global__ void batchedSubstepKernel(const diff::FlatModel* md, diff::DiffState<float>* states,
                                     const float* taus, int nEnv, int ndof, int substeps,
                                     double gx, double gy, double gz, double h) {
    const int e = blockIdx.x * blockDim.x + threadIdx.x;
    if (e >= nEnv) return;
    const diff::V3<double> grav{ gx, gy, gz };
    diff::DiffState<float> st = states[e];                                  // load to registers/local
    const float* tau = taus + static_cast<size_t>(e) * static_cast<size_t>(ndof);
    for (int s = 0; s < substeps; ++s) diff::diffSubstep(*md, st, tau, grav, h);
    states[e] = st;                                                         // write back
}

} // namespace

BatchedForward::BatchedForward(const diff::FlatModel& model, int numEnvs)
    : numEnvs_(numEnvs), numDof_(model.numDof) {
    const size_t nStates = sizeof(diff::DiffState<float>) * static_cast<size_t>(numEnvs_);
    const size_t nTau    = sizeof(float) * static_cast<size_t>(numEnvs_) * static_cast<size_t>(numDof_ > 0 ? numDof_ : 1);
    cudaCheck(cudaMalloc(&dModel_, sizeof(diff::FlatModel)), "cudaMalloc model");
    cudaCheck(cudaMemcpy(dModel_, &model, sizeof(diff::FlatModel), cudaMemcpyHostToDevice), "upload model");
    cudaCheck(cudaMalloc(&dStates_, nStates), "cudaMalloc states");
    cudaCheck(cudaMemset(dStates_, 0, nStates), "memset states");
    cudaCheck(cudaMalloc(&dTau_, nTau), "cudaMalloc tau");
    cudaCheck(cudaMemset(dTau_, 0, nTau), "memset tau");
}

BatchedForward::~BatchedForward() {
    cudaFree(dModel_);
    cudaFree(dStates_);
    cudaFree(dTau_);
}

void BatchedForward::setStates(const std::vector<diff::DiffState<float>>& states) {
    if (static_cast<int>(states.size()) != numEnvs_)
        throw std::runtime_error("BatchedForward::setStates size mismatch");
    cudaCheck(cudaMemcpy(dStates_, states.data(), sizeof(diff::DiffState<float>) * states.size(),
                         cudaMemcpyHostToDevice), "setStates");
}

void BatchedForward::getStates(std::vector<diff::DiffState<float>>& states) const {
    states.resize(static_cast<size_t>(numEnvs_));
    cudaCheck(cudaMemcpy(states.data(), dStates_, sizeof(diff::DiffState<float>) * static_cast<size_t>(numEnvs_),
                         cudaMemcpyDeviceToHost), "getStates");
}

void BatchedForward::setTau(const std::vector<float>& tau) {
    const size_t need = static_cast<size_t>(numEnvs_) * static_cast<size_t>(numDof_);
    if (tau.size() != need) throw std::runtime_error("BatchedForward::setTau size mismatch");
    if (need) cudaCheck(cudaMemcpy(dTau_, tau.data(), sizeof(float) * need, cudaMemcpyHostToDevice), "setTau");
}

void BatchedForward::step(int substeps, double h, const diff::V3<double>& g) {
    const int block = 128;
    const int grid  = (numEnvs_ + block - 1) / block;
    batchedSubstepKernel<<<grid, block>>>(dModel_, dStates_, dTau_, numEnvs_, numDof_, substeps,
                                          g.x, g.y, g.z, h);
    cudaCheck(cudaGetLastError(), "kernel launch");
}

void BatchedForward::synchronize() const { cudaCheck(cudaDeviceSynchronize(), "synchronize"); }

} // namespace engine::physics::cuda
