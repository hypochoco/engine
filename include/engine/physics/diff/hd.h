//
//  hd.h
//  engine::physics::diff
//
//  Host/device annotation shim for the CUDA port. `ENGINE_HD` expands to `__host__ __device__` only
//  when the translation unit is compiled by nvcc (`__CUDACC__`), and to NOTHING for an ordinary host
//  C++ compile. So annotating the templated diff hot path with `ENGINE_HD` is a strict no-op for the
//  CPU/Mac build (the preprocessed tokens are identical to before) while making the SAME code callable
//  from device code — one implementation, no CPU/GPU divergence. See
//  notes/investigations/physics/2026-07-08-cuda-port-implementation-progress.md (Phase 2).
//

#pragma once

#if defined(__CUDACC__)
  #define ENGINE_HD __host__ __device__
#else
  #define ENGINE_HD
#endif
