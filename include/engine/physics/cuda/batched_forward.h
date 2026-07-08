//
//  batched_forward.h
//  engine::physics::cuda
//
//  Phase 3 of the CUDA port: a device-resident batched forward simulator. ONE env per GPU thread,
//  each running the SAME templated reduced + smoothed-contact ABA (diff::diffSubstep) in float — the
//  exact code the CPU runs, instantiated on device with FlatModel (see diff/articulated.h Phase 2).
//
//  The FlatModel (constants) is baked on the host and uploaded ONCE; a device DiffState<float> buffer
//  holds all envs' mutable state (AoS first cut — SoA for coalescing is a later optimization). This
//  class is the raw stepping core + a benchmark seam; the RL-facing obs/action/reward wiring is Phase
//  4 (CudaVecEnv). Header is plain host C++ (no CUDA syntax) so ordinary .cpp TUs can drive it; the
//  implementation lives in batched_forward.cu (compiled by nvcc under ENGINE_CUDA).
//

#pragma once

#include <vector>

#include "engine/physics/diff/flat_model.h"

namespace engine::physics::cuda {

class BatchedForward {
public:
    BatchedForward(const diff::FlatModel& model, int numEnvs);
    ~BatchedForward();
    BatchedForward(const BatchedForward&) = delete;
    BatchedForward& operator=(const BatchedForward&) = delete;

    void setStates(const std::vector<diff::DiffState<float>>& states);   // host -> device (size == numEnvs)
    void getStates(std::vector<diff::DiffState<float>>& states) const;   // device -> host (resized to numEnvs)
    void setTau(const std::vector<float>& tau);                          // numEnvs*numDof floats, row-major [env][dof]

    // Run `substeps` substeps of size `h` (seconds) under gravity `g` on every env — one kernel launch.
    void step(int substeps, double h, const diff::V3<double>& gravity);
    void synchronize() const;

    int numEnvs() const { return numEnvs_; }
    int numDof()  const { return numDof_; }

    // Device buffer accessors — for composing higher-level kernels (e.g. CudaVecEnv action->tau /
    // obs packing) against the same device-resident state without extra copies.
    diff::DiffState<float>* deviceStates()       { return dStates_; }
    const diff::DiffState<float>* deviceStates() const { return dStates_; }
    float*                  deviceTau()          { return dTau_; }
    const diff::FlatModel*  deviceModel()  const { return dModel_; }

private:
    int numEnvs_ = 0;
    int numDof_  = 0;
    diff::FlatModel*        dModel_  = nullptr;   // device, uploaded once
    diff::DiffState<float>* dStates_ = nullptr;   // device [numEnvs]
    float*                  dTau_    = nullptr;   // device [numEnvs*numDof]
};

} // namespace engine::physics::cuda
