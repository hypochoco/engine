//
//  material_probe.slang
//  engine::tst — proves a game shader can sample the bindless material texture table WITHOUT
//  referencing the shadow map. Regression guard for the texture-array binding contract: the bindless
//  table must land at texture slot 0 (matching the renderer) regardless of shadow usage.
//
#include "engine_mesh.slang"

[shader("vertex")]
VSOutput vertexMain(VSInput input, uint instanceID : SV_InstanceID) {
    return standardVertex(input, instanceID);
}

[shader("fragment")]
float4 fragmentMain(VSOutput input, bool isFrontFace : SV_IsFrontFace) : SV_Target {
    Material mat = materials[input.materialIndex];
    float3 c = input.color;
    if (mat.baseColorTexture >= 0)
        c *= materialTextures[mat.baseColorTexture].Sample(materialSampler, input.uv).rgb;
    return float4(c, 1.0);   // no shadow reference on purpose
}
