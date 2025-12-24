//
//  graphics_instance.cpp
//  paint
//
//  Created by Daniel Cho on 12/21/25.
//

// system for handling model instancing

#include "engine/graphics/graphics.h"

//void Graphics::updateInstanceSSBOs(uint32_t currentFrame,
//                                   std::vector<InstanceSSBO>& instances) {
//    
//    // todo: dirty flags
//    
//    // todo: only copy a range, group copy calls together
//        
//    //    for (size_t i = 0; i < instances.size(); ) {
//    //        if (!dirty[i]) { i++; continue; }
//    //
//    //        size_t start = i;
//    //        while (i < instances.size() && dirty[i]) i++;
//    //        size_t count = i - start;
//    //
//    //        memcpy(
//    //            (char*)instanceStorageBuffersMapped[currentFrame] + start * sizeof(InstanceSSBO),
//    //            instances.data() + start,
//    //            count * sizeof(InstanceSSBO)
//    //        );
//
//    memcpy(instanceStorageBuffersMapped[currentFrame],
//           instances.data(),
//           instances.size() * sizeof(InstanceSSBO));
//    
//}

// note: a little jank, but should be okay for now

void Graphics::addDrawJob(uint32_t meshIndex,
                          uint32_t firstMaterial,
                          uint32_t materialCount,
                          std::vector<glm::mat4> modelMatrices) {
    
    // todo: max entitites
    
    drawJobs.emplace_back(DrawJob {
        true,
        meshIndex,
        firstMaterial,
        materialCount,
        (uint32_t)instanceModelMatrices.size(),
        (uint32_t)modelMatrices.size()
    });
    
    instanceModelMatrices.insert(instanceModelMatrices.end(),
                                 std::make_move_iterator(modelMatrices.begin()),
                                 std::make_move_iterator(modelMatrices.end()));
    
}

void Graphics::copyInstanceToBuffer(uint32_t currentFrame) {
    
    // todo: needs a lot more work
    
    memcpy(instanceStorageBuffersMapped[currentFrame],
           instanceModelMatrices.data(),
           instanceModelMatrices.size() * sizeof(InstanceSSBO));
    
}
