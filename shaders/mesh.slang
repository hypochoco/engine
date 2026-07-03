//
//  mesh.slang
//  engine shaders
//
//  Consumes the canonical engine::Vertex (position/normal/uv/color). Transforms by a camera
//  uniform (MVP + normal matrix) and applies a simple fixed-direction diffuse term. Attribute
//  order matches engine::Vertex field order, which render::coreVertexLayout() mirrors.
//
//  Compiled with column-major matrix layout (see shaders/CMakeLists.txt) so glm matrices
//  upload directly without transposing.
//

struct Uniforms {
    float4x4 mvp;
    float4x4 normalMatrix;
};
ConstantBuffer<Uniforms> uniforms;

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
VSOutput vertexMain(VSInput input) {
    VSOutput output;
    output.position = mul(uniforms.mvp, float4(input.position, 1.0));
    output.normal   = mul((float3x3)uniforms.normalMatrix, input.normal);
    output.color    = input.color;
    return output;
}

[shader("fragment")]
float4 fragmentMain(VSOutput input) : SV_Target {
    float3 L = normalize(float3(0.4, 0.6, 0.7));
    float  diffuse = max(dot(normalize(input.normal), L), 0.0) * 0.8 + 0.2;
    return float4(input.color * diffuse, 1.0);
}
