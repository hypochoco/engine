//
//  cluster.slang
//  engine shaders
//
//  Clustered forward+ light binning (RF4b + RF4b-opt). One compute thread per LIGHT (not per
//  froxel): each light computes the small range of froxels its bounding sphere can touch (a tile
//  range in x/y + a depth-slice range), does the precise sphere-vs-froxel-AABB test on just those
//  candidates, and appends its index into the overlapping clusters' lists via an atomic counter.
//
//  This is O(lights × local-froxels) instead of the earlier O(clusters × lights) per-cluster scan,
//  so cost tracks how LOCALIZED the lights are rather than the total cluster count. `lightGrid`
//  (per-cluster counts) MUST be zero-cleared before dispatch (the host does this each frame).
//
//  Froxel grid: CX × CY screen tiles × CZ depth slices (linear in view-space distance). Kernels
//  use 64-wide threadgroups.
//

struct PointLight {
    float3 position;   // world space
    float  radius;
    float3 color;
    float  intensity;
};

struct ClusterParams {
    float4x4 view;      // world -> view
    float4   frustum;   // x=tanHalfFovY  y=aspect  z=zNear  w=zFar
    float4   screen;    // x=screenW  y=screenH
    uint4    gridDim;   // x=CX  y=CY  z=CZ  w=maxLightsPerCluster
    uint4    misc;      // x=lightCount
};

ConstantBuffer<ClusterParams>  cparams;      // buffer 0
StructuredBuffer<PointLight>   clights;       // buffer 1
RWStructuredBuffer<uint>       lightGrid;      // buffer 2 : per-cluster light count (atomic; pre-zeroed)
RWStructuredBuffer<uint>       lightIndices;   // buffer 3 : per-cluster [maxLightsPerCluster] indices

float sliceDist(uint s) {
    float zNear = cparams.frustum.z, zFar = cparams.frustum.w;
    return zNear + (zFar - zNear) * (float(s) / float(cparams.gridDim.z));
}

// View-space AABB of froxel (i,j,k). Camera looks down -Z, so z spans [-d1, -d0].
void froxelAABB(uint i, uint j, uint k, out float3 mn, out float3 mx) {
    uint CX = cparams.gridDim.x, CY = cparams.gridDim.y;
    float ndcX0 = -1.0 + 2.0 * float(i)     / float(CX);
    float ndcX1 = -1.0 + 2.0 * float(i + 1) / float(CX);
    float ndcY0 = -1.0 + 2.0 * float(j)     / float(CY);
    float ndcY1 = -1.0 + 2.0 * float(j + 1) / float(CY);
    float tanH = cparams.frustum.x, aspect = cparams.frustum.y;
    float d0 = sliceDist(k), d1 = sliceDist(k + 1);
    float kx = tanH * aspect, ky = tanH;
    float xs[4] = { ndcX0 * d0 * kx, ndcX1 * d0 * kx, ndcX0 * d1 * kx, ndcX1 * d1 * kx };
    float ys[4] = { ndcY0 * d0 * ky, ndcY1 * d0 * ky, ndcY0 * d1 * ky, ndcY1 * d1 * ky };
    mn = float3(min(min(xs[0], xs[1]), min(xs[2], xs[3])),
                min(min(ys[0], ys[1]), min(ys[2], ys[3])), -d1);
    mx = float3(max(max(xs[0], xs[1]), max(xs[2], xs[3])),
                max(max(ys[0], ys[1]), max(ys[2], ys[3])), -d0);
}

[shader("compute")]
[numthreads(64, 1, 1)]
void binLights(uint3 tid : SV_DispatchThreadID) {
    uint lightCount = cparams.misc.x;
    uint li = tid.x;
    if (li >= lightCount) return;

    uint CX = cparams.gridDim.x, CY = cparams.gridDim.y, CZ = cparams.gridDim.z;
    uint maxPer = cparams.gridDim.w;
    float zNear = cparams.frustum.z, zFar = cparams.frustum.w;
    float tanH = cparams.frustum.x, aspect = cparams.frustum.y;

    PointLight pl = clights[li];
    float3 Lv = mul(cparams.view, float4(pl.position, 1.0)).xyz;
    float  r  = pl.radius;
    float  d  = -Lv.z;                          // view-space distance (positive into the scene)
    if (d + r < zNear || d - r > zFar) return;  // entirely outside the frustum depth range

    // Depth-slice range covered by the sphere (+1 margin: an adjacent froxel's FACE can be within
    // r even when its slice is beyond the axis-aligned d±r range — the precise test below rejects
    // any candidate that doesn't actually overlap, so over-including is free).
    float invRange = float(CZ) / max(zFar - zNear, 1e-6);
    int k0 = (int)floor(((d - r) - zNear) * invRange) - 1;
    int k1 = (int)floor(((d + r) - zNear) * invRange) + 1;
    k0 = clamp(k0, 0, int(CZ) - 1);
    k1 = clamp(k1, 0, int(CZ) - 1);

    // Conservative screen-tile range: widest at the nearest depth the sphere reaches (+1 margin).
    float dmin = max(zNear, d - r);
    float kx = tanH * aspect, ky = tanH;
    float ndcXmin = (Lv.x - r) / (dmin * kx);
    float ndcXmax = (Lv.x + r) / (dmin * kx);
    float ndcYmin = (Lv.y - r) / (dmin * ky);
    float ndcYmax = (Lv.y + r) / (dmin * ky);
    int i0 = clamp((int)floor((clamp(ndcXmin, -1.0, 1.0) + 1.0) * 0.5 * float(CX)) - 1, 0, int(CX) - 1);
    int i1 = clamp((int)floor((clamp(ndcXmax, -1.0, 1.0) + 1.0) * 0.5 * float(CX)) + 1, 0, int(CX) - 1);
    int j0 = clamp((int)floor((clamp(ndcYmin, -1.0, 1.0) + 1.0) * 0.5 * float(CY)) - 1, 0, int(CY) - 1);
    int j1 = clamp((int)floor((clamp(ndcYmax, -1.0, 1.0) + 1.0) * 0.5 * float(CY)) + 1, 0, int(CY) - 1);

    float r2 = r * r;
    for (int k = k0; k <= k1; ++k) {
        for (int j = j0; j <= j1; ++j) {
            for (int i = i0; i <= i1; ++i) {
                float3 mn, mx;
                froxelAABB(uint(i), uint(j), uint(k), mn, mx);
                float3 c = clamp(Lv, mn, mx);
                float3 dd = c - Lv;
                if (dot(dd, dd) <= r2) {
                    uint cluster = uint(i) + uint(j) * CX + uint(k) * CX * CY;
                    uint slot;
                    InterlockedAdd(lightGrid[cluster], 1u, slot);
                    if (slot < maxPer) lightIndices[cluster * maxPer + slot] = li;
                }
            }
        }
    }
}
