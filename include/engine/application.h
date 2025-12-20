//
//  application.h
//  engine
//
//  Created by Daniel Cho on 9/30/25.
//

#pragma once

#include "engine/graphics/graphics.h"

class Application {
    
private:
    Graphics graphics;
    AppConfig& config; // todo: we don't need this
    
    // temp: painting application
        
    VkImage brushTextureImage;
    VkDeviceMemory brushTextureImageMemory;
    VkImageView brushTextureImageView;
    
    VkImage layerTextureImage;
    VkDeviceMemory layerTextureImageMemory;
    VkImageView layerTextureImageView;
    
    VkImage canvasTextureImage;
    VkDeviceMemory canvasTextureImageMemory;
    VkImageView canvasTextureImageView;
    
    VkRenderPass brushRenderPass;
    VkFramebuffer brushFrameBuffer;
    VkDescriptorSetLayout brushDescriptorSetLayout;
    VkDescriptorPool brushDescriptorPool;
    std::vector<VkDescriptorSet> brushDescriptorSets;
    VkPipeline brushPipeline;
    VkPipelineLayout brushPipelineLayout;
        
    VkRenderPass layerRenderPass;
    VkFramebuffer layerFrameBuffer;
    VkDescriptorSetLayout layerDescriptorSetLayout;
    VkDescriptorPool layerDescriptorPool;
    std::vector<VkDescriptorSet> layerDescriptorSets;
    VkPipeline layerPipeline;
    VkPipelineLayout layerPipelineLayout;
    
    InputSystem inputSystem;
    
public:
    Application();
    ~Application();
    void init();
    void draw();
    void run();
    void cleanup();
};
