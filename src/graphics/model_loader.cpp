//
//  model_loader.cpp
//  engine
//
//  Created by Daniel Cho on 12/2/25.
//

#include "engine/graphics/model_loader.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>

ObjData ModelLoader::loadObj(const std::string& modelPath) {
    
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
