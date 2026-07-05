//
//  tonemap.slang
//  engine shaders
//
//  Fullscreen tone-mapping resolve (RF5): a vertex shader emits a single screen-covering triangle
//  from SV_VertexID (no vertex buffer), and the fragment samples the HDR color target, applies the
//  ACES filmic curve (Narkowicz approximation), and sRGB-encodes to the LDR output. This is the
//  first shader that SAMPLES a texture (Texture2D + SamplerState, explicit non-bindless bindings).
//

Texture2D    hdrTex;    // texture(0)
SamplerState hdrSamp;   // sampler(0)

struct VSOut {
    float4 pos : SV_Position;
    float2 uv  : TEXCOORD0;
};

[shader("vertex")]
VSOut vertexMain(uint id : SV_VertexID) {
    VSOut o;
    // (0,0),(2,0),(0,2) → a triangle that covers the [-1,1] clip square; uv in [0,1] is visible.
    float2 uv = float2((id << 1) & 2, id & 2);
    o.uv  = uv;
    o.pos = float4(uv * float2(2.0, -2.0) + float2(-1.0, 1.0), 0.0, 1.0);  // flip V for top-left origin
    return o;
}

// ACES filmic tone curve (Narkowicz 2015).
float3 aces(float3 x) {
    const float a = 2.51, b = 0.03, c = 2.43, d = 0.59, e = 0.14;
    return saturate((x * (a * x + b)) / (x * (c * x + d) + e));
}

[shader("fragment")]
float4 fragmentMain(VSOut i) : SV_Target {
    float3 hdr    = hdrTex.Sample(hdrSamp, i.uv).rgb;
    float3 mapped = aces(hdr);
    mapped = pow(mapped, float3(1.0 / 2.2));   // sRGB-ish encode for display
    return float4(mapped, 1.0);
}
