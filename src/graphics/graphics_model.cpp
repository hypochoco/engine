//
//  graphics_model.cpp
//  engine
//
//  Created by Daniel Cho on 12/14/25.
//

#include "engine/graphics/graphics.h"

void Graphics::createVertexBuffer() {
    
    VkDeviceSize bufferSize = sizeof(vertices[0]) * vertices.size();

    VkBuffer stagingBuffer;
    VkDeviceMemory stagingBufferMemory;
    createBuffer(bufferSize,
                 VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                 VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                 stagingBuffer,
                 stagingBufferMemory);

    void* data;
    vkMapMemory(device, stagingBufferMemory, 0, bufferSize, 0, &data);
    memcpy(data, vertices.data(), (size_t) bufferSize);
    vkUnmapMemory(device, stagingBufferMemory);

    createBuffer(bufferSize,
                 VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
                 VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
                 vertexBuffer,
                 vertexBufferMemory);
    copyBuffer(stagingBuffer, vertexBuffer, bufferSize);

    vkDestroyBuffer(device, stagingBuffer, nullptr);
    vkFreeMemory(device, stagingBufferMemory, nullptr);
}

void Graphics::createIndexBuffer() {
    VkDeviceSize bufferSize = sizeof(indices[0]) * indices.size();

    VkBuffer stagingBuffer;
    VkDeviceMemory stagingBufferMemory;
    createBuffer(bufferSize,
                 VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                 VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                 stagingBuffer,
                 stagingBufferMemory);

    void* data;
    vkMapMemory(device, stagingBufferMemory, 0, bufferSize, 0, &data);
    memcpy(data, indices.data(), (size_t) bufferSize);
    vkUnmapMemory(device, stagingBufferMemory);

    createBuffer(bufferSize,
                 VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT,
                 VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
                 indexBuffer,
                 indexBufferMemory);

    copyBuffer(stagingBuffer, indexBuffer, bufferSize);

    vkDestroyBuffer(device, stagingBuffer, nullptr);
    vkFreeMemory(device, stagingBufferMemory, nullptr);
}

void Graphics::createUniformBuffers() {
    
    VkDeviceSize globalBufferSize = sizeof(GlobalUBO);
    VkDeviceSize instanceBufferSize = sizeof(InstanceSSBO) * config.graphicsConfig.MAX_ENTITIES;
    
    globalUniformBuffers.resize(config.graphicsConfig.MAX_FRAMES_IN_FLIGHT);
    globalUniformBuffersMemory.resize(config.graphicsConfig.MAX_FRAMES_IN_FLIGHT);
    globalUniformBuffersMapped.resize(config.graphicsConfig.MAX_FRAMES_IN_FLIGHT);
    
    instanceStorageBuffers.resize(config.graphicsConfig.MAX_FRAMES_IN_FLIGHT);
    instanceStorageBuffersMemory.resize(config.graphicsConfig.MAX_FRAMES_IN_FLIGHT);
    instanceStorageBuffersMapped.resize(config.graphicsConfig.MAX_FRAMES_IN_FLIGHT);
    
    for (size_t i = 0; i < config.graphicsConfig.MAX_FRAMES_IN_FLIGHT; i++) {
        
        createBuffer(globalBufferSize,
                     VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                     VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                     globalUniformBuffers[i],
                     globalUniformBuffersMemory[i]);
        vkMapMemory(device,
                    globalUniformBuffersMemory[i],
                    0,
                    globalBufferSize,
                    0,
                    &globalUniformBuffersMapped[i]);
        
        createBuffer(instanceBufferSize,
                     VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                     VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                     instanceStorageBuffers[i],
                     instanceStorageBuffersMemory[i]);
        vkMapMemory(device,
                    instanceStorageBuffersMemory[i],
                    0,
                    instanceBufferSize,
                    0,
                    &instanceStorageBuffersMapped[i]);
        
    }
    
}

void Graphics::pushModel(ObjData& objData) {
    
    // todo: better material mapping 
        
    Model model;
    
    // push materials
    model.firstMaterialIndex = static_cast<uint32_t>(materials.size());
    model.materialCount = static_cast<uint32_t>(objData.modelMaterials.size());
    materials.reserve(materials.size() + model.materialCount);
    for (const auto& m : objData.modelMaterials) { // note: actual material properties aren't used
        materials.push_back(m);
    }
    
    uint32_t defaultMaterial = 0;
    if (model.materialCount <= 0) { // temp: default material
        defaultMaterial = static_cast<uint32_t>(materials.size());
        materials.emplace_back(Material{"default", 0});
        model.materialCount++;
    }
        
    // convert maps into submeshes
    for (auto& [materialId, uniqueVertices] : objData.uniqueVerticesMap) {
        auto& modelIndices = objData.uniqueIndicesMap[materialId];

        Submesh submesh;
        submesh.firstIndex = static_cast<uint32_t>(indices.size());
        submesh.indexCount = static_cast<uint32_t>(modelIndices.size());
        submesh.firstVertex = static_cast<uint32_t>(vertices.size());
        submesh.vertexCount = static_cast<uint32_t>(uniqueVertices.size());
        
        if (materialId < 0 || materialId >= model.materialCount) {
            submesh.materialIndex = defaultMaterial;
        } else {
            submesh.materialIndex = model.firstMaterialIndex + materialId;
        }
        
        model.submeshes.push_back(submesh);

        // append indices and vertices to global buffers
        indices.insert(indices.end(),
                       std::make_move_iterator(modelIndices.begin()),
                       std::make_move_iterator(modelIndices.end()));
        vertices.insert(vertices.end(),
                        std::make_move_iterator(uniqueVertices.begin()),
                        std::make_move_iterator(uniqueVertices.end()));
    }
    
    models.push_back(model);
    
}
