//
//  mesh.slang
//  engine shaders
//
//  Instanced mesh shader. A shared camera (view-projection), a per-instance storage buffer
//  (model + normal matrix + material index), and a materials storage buffer. Each instance
//  looks up its material's base color. Consumes the canonical engine::Vertex; column-major
//  matrices (see shaders/CMakeLists.txt).
//

struct Camera {
    float4x4 viewProj;
};
ConstantBuffer<Camera> camera;

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
    output.position = mul(camera.viewProj, world);
    output.normal   = mul((float3x3)inst.normalMatrix, input.normal);
    output.color    = input.color * mat.baseColorFactor.rgb;
    return output;
}

[shader("fragment")]
float4 fragmentMain(VSOutput input) : SV_Target {
    float3 L = normalize(float3(0.4, 0.6, 0.7));
    float  diffuse = max(dot(normalize(input.normal), L), 0.0) * 0.8 + 0.2;
    return float4(input.color * diffuse, 1.0);
}
