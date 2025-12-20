//
//  application.h
//  engine
//
//  Created by Daniel Cho on 9/30/25.
//

#pragma once

#include "engine/graphics/graphics.h"

// abstract application

class Application {
    
protected:
    Graphics graphics;
    AppConfig& config; // todo: we don't need this, just grab the static instance
        
public:
    Application();
    ~Application();
    virtual void init() = 0;
    virtual void draw() = 0;
    virtual void run() = 0;
    virtual void cleanup() = 0;
};

// graphics application

class GraphicsApplication : public Application {
    
private:
    
    VkImage modelTextureImage;
    VkDeviceMemory modelTextureImageMemory;
    VkImageView modelTextureImageView;
    
public:
    
    GraphicsApplication();
    ~GraphicsApplication();

    void init();
    void draw();
    void run();
    void cleanup();

};

// paint application

class PaintApplication : public Application {
    
private:
    
    float depth;
    float brushSize;
    
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
    
    PaintApplication();
    ~PaintApplication();

    void init();
    void draw();
    void run();
    void cleanup();

};
