//
//  fxaa.slang
//  engine shaders
//
//  Fast approximate anti-aliasing (RF6 AA), a compact luma-FXAA (after Timothy Lottes' FXAA 3.11).
//  A fullscreen pass over the (tone-mapped) LDR image: detect edges by local luma contrast and
//  blend along the edge direction. Catches the shading aliasing MSAA can't (e.g. the HDR sun disc
//  once tone-mapped), for the cost of one texture-tap-heavy fullscreen pass. Operates in the
//  perceptual/gamma space of the LDR input (where FXAA is designed to run).
//

Texture2D    ldrTex;    // texture(0) : the tone-mapped / LDR image to anti-alias
SamplerState ldrSamp;   // sampler(0) : linear clamp

struct VSOut {
    float4 pos : SV_Position;
    float2 uv  : TEXCOORD0;
};

[shader("vertex")]
VSOut vertexMain(uint id : SV_VertexID) {
    VSOut o;
    float2 uv = float2((id << 1) & 2, id & 2);
    o.uv  = uv;
    o.pos = float4(uv * float2(2.0, -2.0) + float2(-1.0, 1.0), 0.0, 1.0);   // flip V (top-left origin)
    return o;
}

float luma(float3 c) { return dot(c, float3(0.299, 0.587, 0.114)); }

[shader("fragment")]
float4 fragmentMain(VSOut i) : SV_Target {
    const float EDGE_THRESHOLD = 0.125;   // relative contrast to treat as an edge
    const float EDGE_MIN       = 0.0625;  // absolute floor (skip near-flat regions)
    const float REDUCE_MUL     = 1.0 / 8.0;
    const float REDUCE_MIN     = 1.0 / 128.0;
    const float SPAN_MAX       = 8.0;

    float w, h;
    ldrTex.GetDimensions(w, h);
    float2 rcp = float2(1.0 / w, 1.0 / h);

    float3 rgbM  = ldrTex.Sample(ldrSamp, i.uv).rgb;
    float3 rgbNW = ldrTex.Sample(ldrSamp, i.uv + float2(-1, -1) * rcp).rgb;
    float3 rgbNE = ldrTex.Sample(ldrSamp, i.uv + float2( 1, -1) * rcp).rgb;
    float3 rgbSW = ldrTex.Sample(ldrSamp, i.uv + float2(-1,  1) * rcp).rgb;
    float3 rgbSE = ldrTex.Sample(ldrSamp, i.uv + float2( 1,  1) * rcp).rgb;

    float lM  = luma(rgbM);
    float lNW = luma(rgbNW), lNE = luma(rgbNE), lSW = luma(rgbSW), lSE = luma(rgbSE);
    float lMin = min(lM, min(min(lNW, lNE), min(lSW, lSE)));
    float lMax = max(lM, max(max(lNW, lNE), max(lSW, lSE)));

    // Flat enough → leave the pixel alone (no blur where there's no edge).
    if (lMax - lMin < max(EDGE_MIN, lMax * EDGE_THRESHOLD))
        return float4(rgbM, 1.0);

    // Edge direction from the luma gradients.
    float2 dir;
    dir.x = -((lNW + lNE) - (lSW + lSE));
    dir.y =  ((lNW + lSW) - (lNE + lSE));
    float dirReduce = max((lNW + lNE + lSW + lSE) * 0.25 * REDUCE_MUL, REDUCE_MIN);
    float rcpDirMin = 1.0 / (min(abs(dir.x), abs(dir.y)) + dirReduce);
    dir = clamp(dir * rcpDirMin, -SPAN_MAX, SPAN_MAX) * rcp;

    // Two-tap and four-tap averages along the edge; pick the four-tap unless it overshoots.
    float3 rgbA = 0.5 * (ldrTex.Sample(ldrSamp, i.uv + dir * (1.0 / 3.0 - 0.5)).rgb +
                         ldrTex.Sample(ldrSamp, i.uv + dir * (2.0 / 3.0 - 0.5)).rgb);
    float3 rgbB = rgbA * 0.5 + 0.25 * (ldrTex.Sample(ldrSamp, i.uv + dir * -0.5).rgb +
                                       ldrTex.Sample(ldrSamp, i.uv + dir *  0.5).rgb);
    float lB = luma(rgbB);
    float3 outc = (lB < lMin || lB > lMax) ? rgbA : rgbB;
    return float4(outc, 1.0);
}
