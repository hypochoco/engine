//
//  app_data_probe.slang
//  engine::tst — probe shader for the per-view appData channel
//
//  Reuses the shared engine mesh contract and simply outputs globals.appData.rgb, so a headless
//  test can prove that render::RenderView::appData reaches the shader intact. Also exercises the
//  consumer shader-compile path (engine_compile_shaders + ENGINE_SHADER_INCLUDE_DIR) the same way
//  a game (sim-2's foliage) does.
//

#include "engine_mesh.slang"

[shader("vertex")]
VSOutput vertexMain(VSInput input, uint instanceID : SV_InstanceID) {
    return standardVertex(input, instanceID);
}

[shader("fragment")]
float4 fragmentMain(VSOutput input, bool isFrontFace : SV_IsFrontFace) : SV_Target {
    return float4(globals.appData.rgb, 1.0);
}
