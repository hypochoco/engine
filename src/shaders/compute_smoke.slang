//
//  compute_smoke.slang
//  engine shaders
//
//  Minimal compute kernel for the RHI compute smoke test: each thread writes a known function
//  of its dispatch id into a storage buffer, so a headless test can dispatch it and read back.
//  Kernels use 64-wide 1D threadgroups (see CommandList::dispatch convention).
//

RWStructuredBuffer<uint> outBuf;   // buffer(0)

[shader("compute")]
[numthreads(64, 1, 1)]
void computeMain(uint3 tid : SV_DispatchThreadID) {
    outBuf[tid.x] = tid.x * 2u + 1u;
}
