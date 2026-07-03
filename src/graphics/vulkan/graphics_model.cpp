//
//  graphics_model.cpp
//  engine
//
//  Created by Daniel Cho on 12/14/25.
//

#include "engine/graphics/graphics.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>

// vulkan functions

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
    VkDeviceSize instanceBufferSize = sizeof(InstanceSSBO) * MAX_ENTITIES;
    
    globalUniformBuffers.resize(MAX_FRAMES_IN_FLIGHT);
    globalUniformBuffersMemory.resize(MAX_FRAMES_IN_FLIGHT);
    globalUniformBuffersMapped.resize(MAX_FRAMES_IN_FLIGHT);
    
    instanceStorageBuffers.resize(MAX_FRAMES_IN_FLIGHT);
    instanceStorageBuffersMemory.resize(MAX_FRAMES_IN_FLIGHT);
    instanceStorageBuffersMapped.resize(MAX_FRAMES_IN_FLIGHT);
    
    for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
        
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

// mesh and material functions

void Graphics::loadQuad() {
    
    Mesh mesh;
    mesh.firstVertex = static_cast<uint32_t>(vertices.size());
    mesh.vertexCount = static_cast<uint32_t>(vertices.size() + 4);
    mesh.firstIndex = static_cast<uint32_t>(indices.size());
    mesh.indexCount = static_cast<uint32_t>(indices.size() + 6);
    meshes.push_back(mesh);

    vertices.reserve(vertices.size() + 4);

    vertices.emplace_back( Vertex{ { -1.0f, -1.0f, 0.0f }, {1,1,1}, { 0.0f, 0.0f } } ); // bottom-left
    vertices.emplace_back( Vertex{ {  1.0f, -1.0f, 0.0f }, {1,1,1}, { 1.0f, 0.0f } } ); // bottom-right
    vertices.emplace_back( Vertex{ {  1.0f,  1.0f, 0.0f }, {1,1,1}, { 1.0f, 1.0f } } ); // top-right
    vertices.emplace_back( Vertex{ { -1.0f,  1.0f, 0.0f }, {1,1,1}, { 0.0f, 1.0f } } ); // top-left

    indices.insert(indices.end(), {
        0, 1, 2,
        2, 3, 0
    });
    
}

void Graphics::loadObj(const std::string& modelPath) {
    
    // note: assumes output references are empty
    
    // todo: normals
        
    // load obj
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> objMaterials;
    std::string warn, err;
    if (!tinyobj::LoadObj(&attrib, &shapes, &objMaterials, &warn, &err, modelPath.c_str())) {
        throw std::runtime_error("OBJ load failed: " + err);
    }
    
    // obj processing
    
    std::unordered_map<int, std::vector<Vertex>> uniqueVerticesMap;
    std::unordered_map<int, std::vector<uint32_t>> uniqueIndicesMap;
    
    materials.reserve(objMaterials.size());
    for (const auto& m : objMaterials) { // todo: real material properties
        materials.emplace_back( Material{ 0 } );
    }
    
    for (const auto& shape : shapes) {
        
        size_t indexOffset = 0;
        const auto& mesh = shape.mesh;
        
        for (size_t face = 0; face < mesh.num_face_vertices.size(); ++face) {
            
            const int fv = mesh.num_face_vertices[face];
            const int materialId = mesh.material_ids[face]; // note: -1 if no material
            
            for (int v = 0; v < fv; ++v) {
                    
                const tinyobj::index_t idx = mesh.indices[indexOffset + v];
                Vertex vertex{};
                vertex.pos = {
                    attrib.vertices[3 * idx.vertex_index + 0],
                    attrib.vertices[3 * idx.vertex_index + 1],
                    attrib.vertices[3 * idx.vertex_index + 2]
                };
                
                if (idx.texcoord_index >= 0) {
                    vertex.texCoord = {
                        attrib.texcoords[2 * idx.texcoord_index + 0],
                        1.f - attrib.texcoords[2 * idx.texcoord_index + 1]
                    };
                } else {
                    vertex.texCoord = {0.f, 0.f};
                }
                vertex.color = {1.f, 1.f, 1.f}; // note: default material color
                
                auto& uniqueVertices = uniqueVerticesMap[materialId];
                auto& uniqueIndices = uniqueIndicesMap[materialId];

                auto it = std::find(uniqueVertices.begin(), uniqueVertices.end(), vertex);
                if (it != uniqueVertices.end()) {
                    uniqueIndices.push_back(static_cast<uint32_t>(std::distance(uniqueVertices.begin(), it)));
                } else {
                    uniqueVertices.push_back(vertex);
                    uniqueIndices.push_back(static_cast<uint32_t>(uniqueVertices.size() - 1));
                }
            }
            
            indexOffset += fv;
            
        }
    }
    
    // obj loading
    
    for (auto& [materialId, uniqueVertices] : uniqueVerticesMap) {
        auto& modelIndices = uniqueIndicesMap[materialId];
        
        Mesh mesh;
        mesh.firstVertex = static_cast<uint32_t>(vertices.size());
        mesh.vertexCount = static_cast<uint32_t>(uniqueVertices.size());
        mesh.firstIndex = static_cast<uint32_t>(indices.size());
        mesh.indexCount = static_cast<uint32_t>(modelIndices.size());
        meshes.push_back(mesh);

        indices.insert(indices.end(),
                       std::make_move_iterator(modelIndices.begin()),
                       std::make_move_iterator(modelIndices.end()));
        vertices.insert(vertices.end(),
                        std::make_move_iterator(uniqueVertices.begin()),
                        std::make_move_iterator(uniqueVertices.end()));

    }
    
    // todo: material to mesh mapping
    
}
