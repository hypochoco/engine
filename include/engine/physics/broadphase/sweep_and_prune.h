//
//  sweep_and_prune.h
//  engine::physics / broadphase
//
//  Single-axis sweep-and-prune (SAP): sort AABBs along the highest-variance axis and sweep an
//  active interval list, emitting only pairs that overlap on that axis (then confirmed in 3D).
//  O(n log n + k) instead of the brute-force O(n²) all-pairs scan — the Phase-2 scaling lever
//  (see notes/investigations/2026-07-03-physics-baseline.md). Deterministic: emitted pairs are
//  sorted by (i, j).
//

#pragma once

#include <cstdint>
#include <span>
#include <utility>
#include <vector>

#include "engine/physics/broadphase/aabb.h"

namespace engine::physics::broadphase {

void sweepAndPrune(std::span<const Aabb> aabbs, std::vector<Pair>& pairs);

} // namespace engine::physics::broadphase
