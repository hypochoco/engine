//
//  uniform_grid.h
//  engine::physics / broadphase
//
//  Uniform spatial-hash grid broadphase. Cell size is derived from the largest AABB, so each
//  box spans at most 2 cells per axis; candidate pairs are bodies sharing a cell. Near-O(n)
//  for many similar-sized objects (the 100k-sphere case) where single-axis SAP degrades to
//  ~O(n^5/3). Deterministic: emitted pairs are sorted + de-duplicated by (i, j).
//

#pragma once

#include <span>
#include <vector>

#include "engine/physics/broadphase/aabb.h"

namespace engine::physics::broadphase {

void uniformGrid(std::span<const Aabb> aabbs, std::vector<Pair>& pairs);

} // namespace engine::physics::broadphase
