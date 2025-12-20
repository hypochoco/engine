//
//  graphics_model.cpp
//  engine
//
//  Created by Daniel Cho on 12/14/25.
//

#include "engine/graphics/graphics.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>

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

ObjData Graphics::loadObj(const std::string& modelPath) {
    
    // note: this currently does not handle normals
    ObjData objData;
        
    // load obj
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> objMaterials;
    std::string warn, err;
    if (!tinyobj::LoadObj(&attrib, &shapes, &objMaterials, &warn, &err, modelPath.c_str())) {
        throw std::runtime_error("OBJ load failed: " + err);
    }
    objData.modelMaterials.reserve(objMaterials.size());
    for (const auto& m : objMaterials) { // note: actual material properties aren't used
        objData.modelMaterials.emplace_back(Material{m.name, 0}); // note: textureIndex=0, should be fixed later
    }
    
    // process vertices and indices
    for (const auto& shape : shapes) {
        
        size_t indexOffset = 0;
        const auto& mesh = shape.mesh;
        
        for (size_t face = 0; face < mesh.num_face_vertices.size(); ++face) {
            
            int fv = mesh.num_face_vertices[face];
            int materialId = mesh.material_ids[face]; // -1 if no material
            
            for (int v = 0; v < fv; ++v) {
                    
                // pos
                tinyobj::index_t idx = mesh.indices[indexOffset + v];
                Vertex vertex{};
                vertex.pos = {
                    attrib.vertices[3 * idx.vertex_index + 0],
                    attrib.vertices[3 * idx.vertex_index + 1],
                    attrib.vertices[3 * idx.vertex_index + 2]
                };
                
                // tex coord + color
                if (idx.texcoord_index >= 0) {
                    vertex.texCoord = {
                        attrib.texcoords[2 * idx.texcoord_index + 0],
                        1.f - attrib.texcoords[2 * idx.texcoord_index + 1]
                    };
                } else {
                    vertex.texCoord = {0.f, 0.f};
                }
                vertex.color = {1.f, 1.f, 1.f}; // note: default material color
                
                // mapping
                auto& uniqueVertices = objData.uniqueVerticesMap[materialId];
                auto& uniqueIndices = objData.uniqueIndicesMap[materialId];

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
    
    return objData;
}

ObjData Graphics::loadCanvasQuad() {
    ObjData objData;

    // Add one dummy material (since your engine expects it)
    objData.modelMaterials.push_back(Material{"canvas", 0});

    // Define 4 vertices
    std::vector<Vertex> verts;
    verts.reserve(4);

    // Canvas quad in NDC-like local space (you will position it with your transform)
    // If you want it in [0,1] space instead, I can change it.

    verts.push_back(Vertex{ { -1.0f, -1.0f, 0.0f }, {1,1,1}, { 0.0f, 0.0f } }); // bottom-left
    verts.push_back(Vertex{ {  1.0f, -1.0f, 0.0f }, {1,1,1}, { 1.0f, 0.0f } }); // bottom-right
    verts.push_back(Vertex{ {  1.0f,  1.0f, 0.0f }, {1,1,1}, { 1.0f, 1.0f } }); // top-right
    verts.push_back(Vertex{ { -1.0f,  1.0f, 0.0f }, {1,1,1}, { 0.0f, 1.0f } }); // top-left

    // Indices for two triangles
    std::vector<uint32_t> inds = {
        0, 1, 2,
        2, 3, 0
    };

    // Store in objData using materialId = 0
    objData.uniqueVerticesMap[0] = verts;
    objData.uniqueIndicesMap[0] = inds;

    return objData;
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
