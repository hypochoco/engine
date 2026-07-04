//
//  diff_environment.h
//  engine::physics::diff
//
//  Differentiable environment (Phase F3c) — the boundary that mirrors `physics_env::Environment` but
//  runs the SCALAR-generic differentiable ABA, so downstream can get exact gradients. Built from a
//  `physics::ArticulationDef` (e.g. `makeHumanoid()`); holds a plain `double` state advanced by
//  `step()`, and exposes:
//    • `stepJacobian()` — the per-step tangent-space Jacobian ∂s_{t+1}/∂(s_t,a_t) for BPTT;
//    • `rolloutGradient()` — the analytic gradient of a caller's scalar objective over a short
//      rollout w.r.t. the action (forward-mode through the differentiable dynamics), for the
//      analytic policy gradient and as the first-order input to the α-order hybrid.
//  Reset restores the articulation's authored pose. See the differentiable-reduced design note.
//

#pragma once

#include <vector>

#include "engine/physics/diff/from_articulation.h"
#include "engine/physics/diff/jacobian.h"

namespace engine::physics::diff {

class DiffEnvironment {
public:
    explicit DiffEnvironment(const physics::ArticulationDef& def, double footContactRadius = 0.0,
                             V3<double> gravity = { 0, -9.81, 0 }, double controlDt = 1.0 / 60.0, int substeps = 16)
        : model_(articulationToDiffModel(def, footContactRadius)), def_(def), gravity_(gravity),
          controlDt_(controlDt), substeps_(substeps) {
        root_ = rootBodyIndex(model_);
        reset();
    }

    void reset() {
        state_ = makeState<double>(model_);
        const physics::BodyDef& rb = def_.bodies[static_cast<size_t>(root_)];
        state_.basePos = { rb.position.x, rb.position.y, rb.position.z };
        state_.baseRot = glmToM3(glm::mat3_cast(rb.orientation));
        action_.assign(static_cast<size_t>(model_.ndofJoints), 0.0);
    }

    int actionDim() const { return model_.ndofJoints; }
    int substeps() const { return substeps_; }
    double substepDt() const { return controlDt_ / substeps_; }
    void setAction(const std::vector<double>& a) { action_ = a; }

    void step() { const double h = substepDt(); for (int i = 0; i < substeps_; ++i) diffSubstep(model_, state_, action_, gravity_, h); }

    // Per-step tangent Jacobian ∂s_{t+1}/∂(s_t,a_t) for the current (state, action) — full control step.
    StepJacobian jacobian() const { return stepJacobian(model_, state_, action_, gravity_, substepDt(), substeps_); }

    // Analytic gradient of a scalar objective over an `nControlSteps` rollout w.r.t. the first `NA`
    // action components, via forward-mode duals (one pass — all NA seeds at once).
    // `objective(const DiffState<Dual<NA>>&)` must return the scalar as a `Dual<NA>`.
    template <int NA, class Obj>
    std::vector<double> rolloutGradient(const std::vector<double>& action, int nControlSteps, Obj&& objective) const {
        const double h = substepDt();
        std::vector<Dual<NA>> tau(static_cast<size_t>(model_.ndofJoints));
        for (int m = 0; m < model_.ndofJoints; ++m)
            tau[static_cast<size_t>(m)] = (m < NA) ? Dual<NA>::seed(action[static_cast<size_t>(m)], m) : Dual<NA>(action[static_cast<size_t>(m)]);
        DiffState<Dual<NA>> st = liftState<Dual<NA>>(state_);
        for (int c = 0; c < nControlSteps; ++c) for (int i = 0; i < substeps_; ++i) diffSubstep(model_, st, tau, gravity_, h);
        const Dual<NA> o = objective(st);
        std::vector<double> grad(static_cast<size_t>(NA));
        for (int j = 0; j < NA; ++j) grad[static_cast<size_t>(j)] = o.d[j];
        return grad;
    }

    const DiffModel& model() const { return model_; }
    const DiffState<double>& state() const { return state_; }
    std::vector<LinkWorld<double>> links() const { return linkWorld<double>(model_, state_); }
    int rootBody() const { return root_; }

private:
    DiffModel model_;
    physics::ArticulationDef def_;
    int root_ = 0;
    V3<double> gravity_;
    double controlDt_;
    int substeps_;
    DiffState<double> state_;
    std::vector<double> action_;
};

} // namespace engine::physics::diff
