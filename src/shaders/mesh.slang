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

struct VSInput {
    float3 position;   // attribute 0
    float3 normal;     // attribute 1
    float2 uv;         // attribute 2
    float3 color;      // attribute 3
};

struct VSOutput {
    float4 position : SV_Position;
    float3 normal   : NORMAL;
    float3 color    : COLOR0;
};

[shader("vertex")]
VSOutput vertexMain(VSInput input, uint instanceID : SV_InstanceID) {
    Instance inst = instances[instanceID];
    Material mat  = materials[inst.materialIndex];

    VSOutput output;
    float4 world    = mul(inst.model, float4(input.position, 1.0));
    output.position = mul(globals.viewProj, world);
    output.normal   = mul((float3x3)inst.normalMatrix, input.normal);
    output.color    = input.color * mat.baseColorFactor.rgb;
    return output;
}

[shader("fragment")]
float4 fragmentMain(VSOutput input) : SV_Target {
    float3 N       = normalize(input.normal);
    float3 L       = normalize(globals.lightDir.xyz);
    float  ndotl   = max(dot(N, L), 0.0);
    float3 lighting = globals.ambient.rgb + globals.lightColor.rgb * ndotl;
    return float4(input.color * lighting, 1.0);
}
