//
//  mesh.slang
//  engine shaders
//
//  Instanced mesh shader. A shared globals uniform (view-projection + a directional light),
//  a per-instance storage buffer (model + normal matrix + material index), and a materials
//  storage buffer. Each instance looks up its material's base color; the fragment applies a
//  simple ambient + Lambert directional light. Consumes the canonical engine::Vertex;
//  column-major matrices (see shaders/CMakeLists.txt).
//

struct Globals {
    float4x4 viewProj;
    float4   lightDir;     // xyz = normalized direction TO the light (world space)
    float4   lightColor;   // rgb = light color * intensity
    float4   ambient;      // rgb = flat ambient term
    float4   params;       // x = point light count; y = clustered? (>0.5); zw reserved
};
ConstantBuffer<Globals> globals;

struct Instance {
    float4x4 model;
    float4x4 normalMatrix;
    uint     materialIndex;
    uint3    _pad;
};
StructuredBuffer<Instance> instances;

struct Material {
    float4 baseColorFactor;
    int    baseColorTexture;   // index into the bindless table (-1 = none); unused until bindless lands
    uint3  _pad;
};
StructuredBuffer<Material> materials;

// Punctual point lights (buffer 3). Matches render::PointLight (vec3+float, vec3+float).
struct PointLight {
    float3 position;
    float  radius;
    float3 color;
    float  intensity;
};
StructuredBuffer<PointLight> pointLights;

// --- Clustered forward+ (optional; used when globals.params.y > 0.5) ---
struct ClusterParams {
    float4x4 view;      // world -> view
    float4   frustum;   // x=tanHalfFovY y=aspect z=zNear w=zFar
    float4   screen;    // x=screenW y=screenH
    uint4    gridDim;   // x=CX y=CY z=CZ w=maxLightsPerCluster
    uint4    misc;      // x=lightCount
};
ConstantBuffer<ClusterParams> cparams;       // buffer 4
StructuredBuffer<uint>        lightGrid;       // buffer 5 : per-cluster count
StructuredBuffer<uint>        lightIndices;    // buffer 6 : per-cluster light index list

struct VSInput {
    float3 position;   // attribute 0
    float3 normal;     // attribute 1
    float2 uv;         // attribute 2
    float3 color;      // attribute 3
};

struct VSOutput {
    float4 position : SV_Position;
    float3 worldPos : POSITION0;   // world-space position for punctual lighting
    float3 normal   : NORMAL;
    float3 color    : COLOR0;
};

[shader("vertex")]
VSOutput vertexMain(VSInput input, uint instanceID : SV_InstanceID) {
    Instance inst = instances[instanceID];
    Material mat  = materials[inst.materialIndex];

    VSOutput output;
    float4 world    = mul(inst.model, float4(input.position, 1.0));
    output.worldPos = world.xyz;
    output.position = mul(globals.viewProj, world);
    output.normal   = mul((float3x3)inst.normalMatrix, input.normal);
    output.color    = input.color * mat.baseColorFactor.rgb;
    return output;
}

// Lambert diffuse + smooth range attenuation for one point light at a surface point.
float3 shadePoint(PointLight pl, float3 N, float3 P) {
    float3 toL   = pl.position - P;
    float  dist2 = dot(toL, toL);
    float  dist  = sqrt(max(dist2, 1e-8));
    float3 Lp    = toL / dist;
    float  ndl   = max(dot(N, Lp), 0.0);
    float  rangeAtt = saturate(1.0 - dist2 / max(pl.radius * pl.radius, 1e-6));
    rangeAtt = rangeAtt * rangeAtt;
    float  atten = rangeAtt / (1.0 + dist2);
    return pl.color * (pl.intensity * ndl * atten);
}

[shader("fragment")]
float4 fragmentMain(VSOutput input) : SV_Target {
    float3 N       = normalize(input.normal);

    // Ambient + directional sun (Lambert).
    float3 L       = normalize(globals.lightDir.xyz);
    float  ndotl   = max(dot(N, L), 0.0);
    float3 lighting = globals.ambient.rgb + globals.lightColor.rgb * ndotl;

    if (globals.params.y > 0.5) {
        // Clustered: find this fragment's froxel and loop only its lights.
        uint CX = cparams.gridDim.x, CY = cparams.gridDim.y, CZ = cparams.gridDim.z;
        uint maxPer = cparams.gridDim.w;
        float viewZ = mul(cparams.view, float4(input.worldPos, 1.0)).z;   // negative
        float d = -viewZ;
        float zNear = cparams.frustum.z, zFar = cparams.frustum.w;
        int kz = (int)floor((d - zNear) / max(zFar - zNear, 1e-6) * (float)CZ);
        kz = clamp(kz, 0, (int)CZ - 1);
        int ix = clamp((int)floor(input.position.x / cparams.screen.x * (float)CX), 0, (int)CX - 1);
        // Framebuffer pixel origin is top-left, but froxel Y follows view-space +Y (up), so flip.
        int iy = clamp((int)floor((1.0 - input.position.y / cparams.screen.y) * (float)CY), 0, (int)CY - 1);
        uint cluster = (uint)ix + (uint)iy * CX + (uint)kz * CX * CY;
        // lightGrid holds the TRUE overlap count (atomic), which can exceed the per-cluster
        // capacity; only the first maxPer indices are stored, so clamp the loop.
        uint count = min(lightGrid[cluster], maxPer);
        for (uint s = 0; s < count; ++s) {
            uint li = lightIndices[cluster * maxPer + s];
            lighting += shadePoint(pointLights[li], N, input.worldPos);
        }
    } else {
        // Loop-all fallback.
        uint count = (uint)globals.params.x;
        for (uint i = 0; i < count; ++i) {
            lighting += shadePoint(pointLights[i], N, input.worldPos);
        }
    }

    return float4(input.color * lighting, 1.0);
}
