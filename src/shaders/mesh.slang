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
    float4x4 lightViewProj; // world -> light clip (directional shadow map)
    float4   lightDir;     // xyz = normalized direction TO the light (world space); w = shadow bias
    float4   lightColor;   // rgb = light color * intensity
    float4   ambient;      // rgb = flat ambient term
    float4   params;       // x = point light count; y = clustered?; z = shadows?; w = shadowMapSize
    float4   camPos;       // xyz = camera world position; w = fog enabled? (>0.5)
    float4   fogColor;     // rgb = base extinction tint (scene-referred / HDR)
    float4   fogInscatter; // rgb = sun in-scatter color (scene-referred)
    float4   fogParams;    // x = density; y = heightFalloff; z = baseY; w = inscatter exponent
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

// Directional shadow map (texture 0 / sampler 0). Sampled with manual PCF (depth formats are not
// filterable, so the sampler is Nearest and we average taps ourselves).
Texture2D    shadowMap;
SamplerState shadowSamp;

// Fraction of the sun reaching `P` (1 = fully lit, 0 = fully shadowed), 3x3 PCF. `ndl` = N·L,
// used for SLOPE-SCALED bias: near 0 bias for surfaces facing the light (kills peter-panning at the
// contact) rising to the max at grazing angles (still prevents self-shadow acne). If the passed
// bias (globals.lightDir.w) is negative, it's treated as a plain CONSTANT bias (for A/B testing).
float sunShadow(float3 P, float ndl) {
    float4 lc = mul(globals.lightViewProj, float4(P, 1.0));
    float3 ndc = lc.xyz / lc.w;
    float2 uv = float2(ndc.x * 0.5 + 0.5, -ndc.y * 0.5 + 0.5);   // clip -> UV (top-left origin)
    if (uv.x < 0.0 || uv.x > 1.0 || uv.y < 0.0 || uv.y > 1.0 || ndc.z > 1.0) return 1.0;
    float curDepth = ndc.z;
    float maxB = abs(globals.lightDir.w);
    float slope = sqrt(max(1.0 - ndl * ndl, 0.0)) / max(ndl, 0.05);   // tan(acos(ndl))
    float bias = (globals.lightDir.w < 0.0) ? maxB
                                            : clamp(maxB * slope, maxB * 0.08, maxB);
    float texel = 1.0 / max(globals.params.w, 1.0);
    float sum = 0.0;
    [unroll] for (int y = -1; y <= 1; ++y)
        [unroll] for (int x = -1; x <= 1; ++x) {
            float d = shadowMap.Sample(shadowSamp, uv + float2(x, y) * texel).r;
            sum += (curDepth - bias > d) ? 0.0 : 1.0;
        }
    return sum / 9.0;
}

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

    // Ambient + directional sun (Lambert), shadowed by the sun shadow map when enabled.
    float3 L       = normalize(globals.lightDir.xyz);
    float  ndotl   = max(dot(N, L), 0.0);
    float  sun     = ndotl;
    if (globals.params.z > 0.5) sun *= sunShadow(input.worldPos, ndotl);
    float3 lighting = globals.ambient.rgb + globals.lightColor.rgb * sun;

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

    float3 litColor = input.color * lighting;

    // Aerial perspective + height fog (optional; camPos.w > 0.5). Blend the lit color toward a
    // sky-consistent fog color that brightens/warms toward the sun (Mie-like in-scatter). Fog is
    // dense low and thins with altitude. Cheap: one length + two exps + a pow.
    if (globals.camPos.w > 0.5) {
        float3 toCam   = input.worldPos - globals.camPos.xyz;
        float  dist    = length(toCam);
        float3 viewDir = toCam / max(dist, 1e-4);
        float  heightK = exp(-max(input.worldPos.y - globals.fogParams.z, 0.0) * globals.fogParams.y);
        float  fogAmt  = saturate(1.0 - exp(-dist * globals.fogParams.x * heightK));
        float  mu      = dot(viewDir, normalize(globals.lightDir.xyz));   // view·sun
        float3 inscat  = pow(saturate(mu), globals.fogParams.w) * globals.fogInscatter.rgb;
        float3 fogCol  = globals.fogColor.rgb + inscat;
        litColor = lerp(litColor, fogCol, fogAmt);
    }

    return float4(litColor, 1.0);
}
