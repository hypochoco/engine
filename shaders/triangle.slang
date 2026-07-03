//
//  triangle.slang
//  engine shaders
//
//  Minimal single-source shader: pass through clip-space position + vertex color.
//  Compiled by slangc to SPIR-V (Vulkan) and a Metal library (Metal). Entry points are
//  tagged with [shader(...)] so slangc discovers them without -entry/-stage flags.
//

struct VSInput {
    float3 position;
    float3 color;
};

struct VSOutput {
    float4 position : SV_Position;
    float3 color    : COLOR0;
};

[shader("vertex")]
VSOutput vertexMain(VSInput input) {
    VSOutput output;
    output.position = float4(input.position, 1.0);
    output.color    = input.color;
    return output;
}

[shader("fragment")]
float4 fragmentMain(VSOutput input) : SV_Target {
    return float4(input.color, 1.0);
}
