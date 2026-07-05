//
//  shadow.slang
//  engine shaders
//
//  Depth-only shadow map pass: renders the instanced scene from the directional light's
//  orthographic point of view, writing only depth (no fragment shader / no color attachment).
//  The forward pass (mesh.slang) then samples this depth to shadow the sun. Uses the same
//  instance buffer + vertex layout as the forward pass.
//

struct ShadowGlobals {
    float4x4 lightViewProj;   // world -> light clip
};
ConstantBuffer<ShadowGlobals> sglobals;    // buffer 0

struct Instance {
    float4x4 model;
    float4x4 normalMatrix;
    uint     materialIndex;
    uint3    _pad;
};
StructuredBuffer<Instance> instances;      // buffer 1

struct VSInput {
    float3 position;   // attribute 0
    float3 normal;     // attribute 1
    float2 uv;         // attribute 2
    float3 color;      // attribute 3
};

[shader("vertex")]
float4 vertexMain(VSInput input, uint instanceID : SV_InstanceID) : SV_Position {
    Instance inst = instances[instanceID];
    float4 world = mul(inst.model, float4(input.position, 1.0));
    return mul(sglobals.lightViewProj, world);
}
