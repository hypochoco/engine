//
//  mesh.slang
//  engine shaders
//
//  The stock instanced forward mesh shader. Its entire binding + vertex contract and shading live
//  in the shared include engine_mesh.slang, so game shaders (foliage/terrain/custom) can reuse the
//  same contract. This file is just the two entry points over that contract; build the pipeline via
//  Renderer::createMeshPipeline. Column-major matrices (see shaders/CMakeLists.txt).
//

#include "engine_mesh.slang"

[shader("vertex")]
VSOutput vertexMain(VSInput input, uint instanceID : SV_InstanceID) {
    return standardVertex(input, instanceID);
}

[shader("fragment")]
float4 fragmentMain(VSOutput input, bool isFrontFace : SV_IsFrontFace) : SV_Target {
    return shadeSurface(input, isFrontFace);
}
