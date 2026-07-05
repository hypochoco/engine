//
//  sky.slang
//  engine shaders
//
//  Procedural sun-coupled sky (RF6 sky). A fullscreen triangle (from SV_VertexID, no vertex
//  buffer) drawn at the FAR plane (ndc.z = 1) at the END of the forward pass, depth-tested
//  LessEqual with depth-write OFF — so it fills ONLY background pixels (where opaque geometry left
//  depth == far), never overdrawing the scene. Runs in the forward pass's own render pass, so on
//  Apple TBDR it stays on-tile (no HDR store/load round trip).
//
//  The model is cheap and physically-INSPIRED, not accurate (realtime, per the design note): a
//  horizon->zenith gradient warmed toward the sun, dimmed by sun elevation, plus a bright HDR sun
//  disc and a broad forward-scatter glow. ~20 ALU ops/pixel, no LUTs/textures. The sun direction is
//  the same one that drives the directional light + shadows, so everything stays consistent.
//

struct SkyGlobals {
    float4x4 invViewProj;   // clip -> world (inverse of the camera viewProj)
    float4   camPos;        // xyz = camera world position
    float4   sunDir;        // xyz = direction TOWARD the sun (normalize(-light.direction))
    float4   zenith;        // rgb sky color straight up
    float4   horizon;       // rgb sky color at the horizon
    float4   ground;        // rgb below-horizon (haze/ground) color
    float4   sunColor;      // rgb sun disc/glow radiance (scene-referred, may exceed 1)
    float4   params;        // x=cos(sun angular radius) y=glowExponent z=glowStrength w=brightness
};
ConstantBuffer<SkyGlobals> sky;   // buffer 0

struct VSOut {
    float4 pos : SV_Position;
    float2 ndc : TEXCOORD0;   // linear in screen space ⇒ exact per-pixel ray reconstruction
};

[shader("vertex")]
VSOut vertexMain(uint id : SV_VertexID) {
    float2 uv  = float2((id << 1) & 2, id & 2);   // (0,0),(2,0),(0,2)
    float2 ndc = uv * 2.0 - 1.0;                    // (-1,-1),(3,-1),(-1,3) covers the screen
    VSOut o;
    o.pos = float4(ndc, 1.0, 1.0);   // z = 1 : far plane (GLM_FORCE_DEPTH_ZERO_TO_ONE)
    o.ndc = ndc;
    return o;
}

[shader("fragment")]
float4 fragmentMain(VSOut i) : SV_Target {
    // Reconstruct the world-space view ray for this pixel from the inverse viewProj.
    float4 world = mul(sky.invViewProj, float4(i.ndc, 1.0, 1.0));
    float3 dir   = normalize(world.xyz / world.w - sky.camPos.xyz);
    float3 s     = normalize(sky.sunDir.xyz);

    float mu = dot(dir, s);          // view·sun alignment (1 = looking straight at the sun)
    float up = saturate(dir.y);      // 0 at horizon, 1 at zenith

    // Overall day/night dimming from the sun's elevation (floor keeps night from going black).
    float dayScale = 0.15 + 0.85 * saturate(s.y);

    // Vertical gradient: horizon -> zenith (sqrt tightens the bright band near the horizon).
    float3 col = lerp(sky.horizon.rgb, sky.zenith.rgb, sqrt(up));
    // Warm/brighten the horizon toward the sun's azimuth (broad, fades with height).
    float towardSun = pow(saturate(mu * 0.5 + 0.5), 4.0) * (1.0 - up);
    col += sky.horizon.rgb * towardSun * 0.5;
    // Below the horizon: fade to a dim ground/haze color.
    col = lerp(sky.ground.rgb, col, saturate(dir.y * 10.0 + 0.5));
    col *= dayScale;

    // Sun disc (sharp) + broad forward-scatter glow (Mie-like). Scene-referred via sunColor.
    float disc = smoothstep(sky.params.x - 0.0008, sky.params.x, mu);
    float glow = pow(saturate(mu), sky.params.y) * sky.params.z;
    col += sky.sunColor.rgb * (disc + glow) * saturate(s.y * 4.0 + 0.1);   // sun below horizon ⇒ no disc

    return float4(col * sky.params.w, 1.0);
}
