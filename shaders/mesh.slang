//
//  mesh.slang
//  engine shaders
//
//  Instanced mesh shader. A shared camera (view-projection) plus a per-instance storage
//  buffer (model + normal matrix), indexed by SV_InstanceID. Consumes the canonical
//  engine::Vertex; attribute order matches render::coreVertexLayout(). Simple fixed-direction
//  diffuse shading. Column-major matrices (see shaders/CMakeLists.txt) so glm uploads direct.
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
    VSOutput output;
    float4 world    = mul(inst.model, float4(input.position, 1.0));
    output.position = mul(camera.viewProj, world);
    output.normal   = mul((float3x3)inst.normalMatrix, input.normal);
    output.color    = input.color;
    return output;
}

[shader("fragment")]
float4 fragmentMain(VSOutput input) : SV_Target {
    float3 L = normalize(float3(0.4, 0.6, 0.7));
    float  diffuse = max(dot(normalize(input.normal), L), 0.0) * 0.8 + 0.2;
    return float4(input.color * diffuse, 1.0);
}
