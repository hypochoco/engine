//
//  engine_mesh.slang
//  engine shaders — SHARED MESH CONTRACT (include)
//
//  The binding + vertex contract for the engine's forward mesh pass, so game shaders (foliage,
//  terrain, custom variants) can reuse it without restating buffer/texture slots or re-deriving
//  the lighting. Include it from a shader in the engine's shader include dir (ENGINE_SHADER_INCLUDE_DIR):
//
//      #include "engine_mesh.slang"
//      [shader("vertex")]   VSOutput vertexMain(VSInput i, uint id : SV_InstanceID) { ... standardVertex / custom ... }
//      [shader("fragment")] float4 fragmentMain(VSOutput i, bool ff : SV_IsFrontFace) : SV_Target { return shadeSurface(i, ff); }
//
//  Build the pipeline with Renderer::createMeshPipeline (it owns the formats / layout / sampleCount /
//  bindings) — a game never writes backend (Metal/Vulkan) code, only Slang + a pipeline variant.
//

#pragma once

// ---- Buffer 0: per-view globals ------------------------------------------------------------------
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
    float4   appData;      // opaque per-view app data (render::RenderView::appData). Engine-agnostic:
                           // game shaders (foliage/water/etc.) define its meaning; stock mesh ignores it.
};
ConstantBuffer<Globals> globals;

// Material feature flags (must match render::MaterialFlags). General, content-agnostic surface
// properties — domain effects like vegetation wind belong in game shaders, not the engine.
static const uint FLAG_ALPHA_CUTOUT = 1u;
static const uint FLAG_DOUBLE_SIDED = 2u;

// ---- Buffer 1: per-instance data -----------------------------------------------------------------
struct Instance {
    float4x4 model;
    float4x4 normalMatrix;
    uint     materialIndex;
    uint3    _pad;
};
StructuredBuffer<Instance> instances;

// ---- Buffer 2: materials (metallic-roughness) ----------------------------------------------------
struct Material {
    float4 baseColorFactor;    // rgba (rgb folded into VSOutput.color; a used for cutout)
    float4 emissiveFactor;     // rgb emissive (a unused)
    float  metallicFactor;
    float  roughnessFactor;
    float  alphaCutoff;
    uint   flags;              // FLAG_* bitfield
    int    baseColorTexture;   // bindless-table slot (-1 = none)
    int    normalTexture;      // tangent-space normal map
    int    mrTexture;          // metallic-roughness (glTF pack: G = roughness, B = metallic)
    int    emissiveTexture;
    int    occlusionTexture;   // ambient occlusion (R channel)
    int3   _pad;
};
StructuredBuffer<Material> materials;

// ---- Buffer 3: punctual point lights -------------------------------------------------------------
struct PointLight {
    float3 position;
    float  radius;
    float3 color;
    float  intensity;
};
StructuredBuffer<PointLight> pointLights;

// ---- Buffers 4-6: clustered forward+ (optional; used when globals.params.y > 0.5) ----------------
struct ClusterParams {
    float4x4 view;      // world -> view
    float4   frustum;   // x=tanHalfFovY y=aspect z=zNear w=zFar
    float4   screen;    // x=screenW y=screenH
    uint4    gridDim;   // x=CX y=CY z=CZ w=maxLightsPerCluster
    uint4    misc;      // x=lightCount
};
ConstantBuffer<ClusterParams> cparams;       // buffer 4
StructuredBuffer<uint>        lightGrid;      // buffer 5 : per-cluster count
StructuredBuffer<uint>        lightIndices;   // buffer 6 : per-cluster light index list

// ---- Textures: shadow map (texture/sampler 0) + bindless material table (texture 1..64, sampler 1)
Texture2D    shadowMap;
SamplerState shadowSamp;
Texture2D    materialTextures[64];   // must match Device::kMaxBindlessTextures
SamplerState materialSampler;

// ---- Vertex I/O (matches render::coreVertexLayout) -----------------------------------------------
struct VSInput {
    float3 position;   // attribute 0
    float3 normal;     // attribute 1
    float2 uv;         // attribute 2
    float3 color;      // attribute 3
    float4 tangent;    // attribute 4 (xyz tangent, w handedness; w=0 => no tangent frame)
};

struct VSOutput {
    float4 position : SV_Position;
    float3 worldPos : POSITION0;
    float3 normal   : NORMAL;
    float4 tangent  : TANGENT;
    float2 uv       : TEXCOORD0;
    float3 color    : COLOR0;
    nointerpolation uint materialIndex : TEXCOORD1;
};

// Standard object→clip transform (no per-domain vertex effects). Game vertex shaders can call this
// and then, e.g., displace `worldPos` for wind before re-projecting, or build VSOutput by hand.
VSOutput standardVertex(VSInput input, uint instanceID) {
    Instance inst = instances[instanceID];
    Material mat  = materials[inst.materialIndex];
    VSOutput o;
    float4 world  = mul(inst.model, float4(input.position, 1.0));
    o.worldPos    = world.xyz;
    o.position    = mul(globals.viewProj, world);
    o.normal      = mul((float3x3)inst.normalMatrix, input.normal);
    o.tangent     = float4(mul((float3x3)inst.normalMatrix, input.tangent.xyz), input.tangent.w);
    o.uv          = input.uv;
    o.color       = input.color * mat.baseColorFactor.rgb;
    o.materialIndex = inst.materialIndex;
    return o;
}

// Fraction of the sun reaching `P` (1 = fully lit, 0 = fully shadowed), 3x3 PCF; slope-scaled bias.
float sunShadow(float3 P, float ndl) {
    float4 lc = mul(globals.lightViewProj, float4(P, 1.0));
    float3 ndc = lc.xyz / lc.w;
    float2 uv = float2(ndc.x * 0.5 + 0.5, -ndc.y * 0.5 + 0.5);
    if (uv.x < 0.0 || uv.x > 1.0 || uv.y < 0.0 || uv.y > 1.0 || ndc.z > 1.0) return 1.0;
    float curDepth = ndc.z;
    float maxB = abs(globals.lightDir.w);
    float slope = sqrt(max(1.0 - ndl * ndl, 0.0)) / max(ndl, 0.05);
    float bias = (globals.lightDir.w < 0.0) ? maxB : clamp(maxB * slope, maxB * 0.08, maxB);
    float texel = 1.0 / max(globals.params.w, 1.0);
    float sum = 0.0;
    [unroll] for (int y = -1; y <= 1; ++y)
        [unroll] for (int x = -1; x <= 1; ++x) {
            float d = shadowMap.Sample(shadowSamp, uv + float2(x, y) * texel).r;
            sum += (curDepth - bias > d) ? 0.0 : 1.0;
        }
    return sum / 9.0;
}

// Lambert diffuse + smooth range attenuation for one point light.
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

// Cook-Torrance GGX specular for one directional light (the sun).
float3 ggxSpecular(float3 N, float3 V, float3 L, float3 F0, float roughness, float3 radiance) {
    float NdotL = max(dot(N, L), 0.0);
    if (NdotL <= 0.0) return float3(0.0, 0.0, 0.0);
    float3 H     = normalize(V + L);
    float  NdotV = max(dot(N, V), 1e-4);
    float  NdotH = max(dot(N, H), 0.0);
    float  VdotH = max(dot(V, H), 0.0);
    float  a  = max(roughness * roughness, 1e-3);
    float  a2 = a * a;
    float  dn = (NdotH * NdotH) * (a2 - 1.0) + 1.0;
    float  D  = a2 / max(3.14159265 * dn * dn, 1e-6);
    float  k  = (roughness + 1.0); k = (k * k) / 8.0;
    float  G  = (NdotV / (NdotV * (1.0 - k) + k)) * (NdotL / (NdotL * (1.0 - k) + k));
    float3 F  = F0 + (1.0 - F0) * pow(1.0 - VdotH, 5.0);
    float3 spec = (D * G) * F / max(4.0 * NdotV * NdotL, 1e-4);
    return spec * radiance * NdotL;
}

// Full forward surface shading (albedo/cutout/normal-map/MR/AO/sun+points/GGX/emissive/fog). Both
// the stock mesh shader and game variants call this so shading stays consistent.
float4 shadeSurface(VSOutput input, bool isFrontFace) {
    Material mat = materials[input.materialIndex];

    float4 baseTex = float4(1.0, 1.0, 1.0, 1.0);
    if (mat.baseColorTexture >= 0)
        baseTex = materialTextures[mat.baseColorTexture].Sample(materialSampler, input.uv);
    float3 albedo = input.color * baseTex.rgb;
    float  alpha  = mat.baseColorFactor.a * baseTex.a;

    if ((mat.flags & FLAG_ALPHA_CUTOUT) != 0u && alpha < mat.alphaCutoff) discard;

    float3 N = normalize(input.normal);
    if ((mat.flags & FLAG_DOUBLE_SIDED) != 0u && !isFrontFace) N = -N;
    if (mat.normalTexture >= 0 && input.tangent.w != 0.0) {
        float3 T = normalize(input.tangent.xyz - N * dot(N, input.tangent.xyz));
        float3 B = cross(N, T) * input.tangent.w;
        float3 nTS = materialTextures[mat.normalTexture].Sample(materialSampler, input.uv).xyz * 2.0 - 1.0;
        N = normalize(nTS.x * T + nTS.y * B + nTS.z * N);
    }

    float metallic  = mat.metallicFactor;
    float roughness = mat.roughnessFactor;
    if (mat.mrTexture >= 0) {
        float3 mr = materialTextures[mat.mrTexture].Sample(materialSampler, input.uv).rgb;
        roughness *= mr.g;
        metallic  *= mr.b;
    }

    float ao = 1.0;
    if (mat.occlusionTexture >= 0)
        ao = materialTextures[mat.occlusionTexture].Sample(materialSampler, input.uv).r;

    float3 L     = normalize(globals.lightDir.xyz);
    float  ndotl = max(dot(N, L), 0.0);
    float  sun   = ndotl;
    if (globals.params.z > 0.5) sun *= sunShadow(input.worldPos, ndotl);
    float3 lighting = globals.ambient.rgb * ao + globals.lightColor.rgb * sun;

    if (globals.params.y > 0.5) {
        uint CX = cparams.gridDim.x, CY = cparams.gridDim.y, CZ = cparams.gridDim.z;
        uint maxPer = cparams.gridDim.w;
        float viewZ = mul(cparams.view, float4(input.worldPos, 1.0)).z;
        float d = -viewZ;
        float zNear = cparams.frustum.z, zFar = cparams.frustum.w;
        int kz = (int)floor((d - zNear) / max(zFar - zNear, 1e-6) * (float)CZ);
        kz = clamp(kz, 0, (int)CZ - 1);
        int ix = clamp((int)floor(input.position.x / cparams.screen.x * (float)CX), 0, (int)CX - 1);
        int iy = clamp((int)floor((1.0 - input.position.y / cparams.screen.y) * (float)CY), 0, (int)CY - 1);
        uint cluster = (uint)ix + (uint)iy * CX + (uint)kz * CX * CY;
        uint count = min(lightGrid[cluster], maxPer);
        for (uint s = 0; s < count; ++s) {
            uint li = lightIndices[cluster * maxPer + s];
            lighting += shadePoint(pointLights[li], N, input.worldPos);
        }
    } else {
        uint count = (uint)globals.params.x;
        for (uint i = 0; i < count; ++i)
            lighting += shadePoint(pointLights[i], N, input.worldPos);
    }

    float3 litColor = albedo * lighting;

    bool hasSpec = (mat.mrTexture >= 0) || (metallic > 0.001) || (roughness < 0.999);
    if (hasSpec) {
        litColor *= (1.0 - metallic);
        float3 V  = normalize(globals.camPos.xyz - input.worldPos);
        float3 F0 = lerp(float3(0.04, 0.04, 0.04), albedo, metallic);
        float3 radiance = globals.lightColor.rgb * sun;
        litColor += ggxSpecular(N, V, L, F0, roughness, radiance);
    }

    float3 emissive = mat.emissiveFactor.rgb;
    if (mat.emissiveTexture >= 0)
        emissive *= materialTextures[mat.emissiveTexture].Sample(materialSampler, input.uv).rgb;
    litColor += emissive;

    if (globals.camPos.w > 0.5) {
        float3 toCam   = input.worldPos - globals.camPos.xyz;
        float  dist    = length(toCam);
        float3 viewDir = toCam / max(dist, 1e-4);
        float  heightK = exp(-max(input.worldPos.y - globals.fogParams.z, 0.0) * globals.fogParams.y);
        float  fogAmt  = saturate(1.0 - exp(-dist * globals.fogParams.x * heightK));
        float  mu      = dot(viewDir, normalize(globals.lightDir.xyz));
        float3 inscat  = pow(saturate(mu), globals.fogParams.w) * globals.fogInscatter.rgb;
        float3 fogCol  = globals.fogColor.rgb + inscat;
        litColor = lerp(litColor, fogCol, fogAmt);
    }

    return float4(litColor, 1.0);
}
