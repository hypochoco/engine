//
//  cluster.slang
//  engine shaders
//
//  Clustered forward+ light binning (RF4b). One compute thread per froxel: compute the froxel's
//  view-space AABB from the perspective params, then test every point light (a sphere in view
//  space) against it, appending overlapping light indices into a per-cluster list. The forward
//  shader (mesh.slang) then loops only its froxel's lights instead of all lights.
//
//  Froxel grid: CX x CY screen tiles x CZ depth slices (linear in view-space distance — simple +
//  easy to verify; exponential slicing is a later tuning step). Kernels use 64-wide threadgroups.
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
RWStructuredBuffer<uint>       lightGrid;      // buffer 2 : per-cluster light count
RWStructuredBuffer<uint>       lightIndices;   // buffer 3 : per-cluster [maxLightsPerCluster] indices

// View-space distance (positive) at depth-slice boundary s in [0, CZ] (linear).
float sliceDist(uint s) {
    float zNear = cparams.frustum.z, zFar = cparams.frustum.w;
    return zNear + (zFar - zNear) * (float(s) / float(cparams.gridDim.z));
}

[shader("compute")]
[numthreads(64, 1, 1)]
void binLights(uint3 tid : SV_DispatchThreadID) {
    uint CX = cparams.gridDim.x, CY = cparams.gridDim.y, CZ = cparams.gridDim.z;
    uint maxPer = cparams.gridDim.w;
    uint numClusters = CX * CY * CZ;
    uint cluster = tid.x;
    if (cluster >= numClusters) return;

    uint i = cluster % CX;
    uint j = (cluster / CX) % CY;
    uint k = cluster / (CX * CY);

    // NDC tile bounds [-1,1].
    float ndcX0 = -1.0 + 2.0 * float(i)     / float(CX);
    float ndcX1 = -1.0 + 2.0 * float(i + 1) / float(CX);
    float ndcY0 = -1.0 + 2.0 * float(j)     / float(CY);
    float ndcY1 = -1.0 + 2.0 * float(j + 1) / float(CY);

    float tanH = cparams.frustum.x, aspect = cparams.frustum.y;
    float d0 = sliceDist(k), d1 = sliceDist(k + 1);

    // View-space x = ndcX * d * (tanH*aspect); y = ndcY * d * tanH. Camera looks down -Z, so the
    // froxel spans view z in [-d1, -d0]. Extremes of x,y occur at the slice's near/far distance.
    float kx = tanH * aspect, ky = tanH;
    float xs[4] = { ndcX0 * d0 * kx, ndcX1 * d0 * kx, ndcX0 * d1 * kx, ndcX1 * d1 * kx };
    float ys[4] = { ndcY0 * d0 * ky, ndcY1 * d0 * ky, ndcY0 * d1 * ky, ndcY1 * d1 * ky };
    float3 aabbMin = float3(min(min(xs[0], xs[1]), min(xs[2], xs[3])),
                            min(min(ys[0], ys[1]), min(ys[2], ys[3])), -d1);
    float3 aabbMax = float3(max(max(xs[0], xs[1]), max(xs[2], xs[3])),
                            max(max(ys[0], ys[1]), max(ys[2], ys[3])), -d0);

    uint count = 0;
    uint lightCount = cparams.misc.x;
    for (uint li = 0; li < lightCount && count < maxPer; ++li) {
        PointLight pl = clights[li];
        float3 Lv = mul(cparams.view, float4(pl.position, 1.0)).xyz;   // light in view space
        // sphere vs AABB: squared distance from Lv to the clamped point.
        float3 c = clamp(Lv, aabbMin, aabbMax);
        float3 d = c - Lv;
        float sq = dot(d, d);
        if (sq <= pl.radius * pl.radius) {
            lightIndices[cluster * maxPer + count] = li;
            ++count;
        }
    }
    lightGrid[cluster] = count;
}
