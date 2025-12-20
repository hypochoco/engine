//
//  application.cpp
//  engine
//
//  Created by Daniel Cho on 9/30/25.
//

#include "engine/application.h"

#include <stdexcept>
#include <iostream>

Application::Application() : config(AppConfig::instance()) { // constructor
    // currently empty
}

Application::~Application() { // destructor
    // cleanup here
}

void Application::init() {
    
    graphics.initWindow();
    graphics.initVulkan();
    
    //    // 3d model application
    //
    //    auto textureIndex = graphics.loadTexture("viking_room.png");
    //    auto objData = ModelLoader::loadObj("viking_room.obj");
    //    for (auto& m : objData.modelMaterials) {
    //        m.textureIndex = textureIndex;
    //    }
    //    graphics.pushModel(objData);

    // ---
    
    // digital painting application
    
    // brush
    
    graphics.loadTexture("brush.png",
                         brushTextureImage,
                         brushTextureImageMemory,
                         brushTextureImageView,
                         VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT
                         | VK_IMAGE_USAGE_TRANSFER_SRC_BIT
                         | VK_IMAGE_USAGE_TRANSFER_DST_BIT
                         | VK_IMAGE_USAGE_SAMPLED_BIT,
                         1);
    
    graphics.transitionImageLayout(brushTextureImage,
                                   VK_FORMAT_R8G8B8A8_SRGB,
                                   VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                                   VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                                   1);
        
    // layer
    
    graphics.createTexture(config.paintConfig.CANVAS_WIDTH,
                           config.paintConfig.CANVAS_HEIGHT,
                           layerTextureImage,
                           layerTextureImageMemory,
                           layerTextureImageView,
                           VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT
                           | VK_IMAGE_USAGE_TRANSFER_SRC_BIT
                           | VK_IMAGE_USAGE_TRANSFER_DST_BIT
                           | VK_IMAGE_USAGE_SAMPLED_BIT,
                           1);
    
    graphics.transitionImageLayout(layerTextureImage,
                                   VK_FORMAT_R8G8B8A8_SRGB,
                                   VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                                   VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                                   1);
    
    // canvas texture
            
    graphics.createTexture(config.paintConfig.CANVAS_WIDTH,
                           config.paintConfig.CANVAS_HEIGHT,
                           canvasTextureImage,
                           canvasTextureImageMemory,
                           canvasTextureImageView,
                           VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT
                           | VK_IMAGE_USAGE_TRANSFER_SRC_BIT
                           | VK_IMAGE_USAGE_TRANSFER_DST_BIT
                           | VK_IMAGE_USAGE_SAMPLED_BIT,
                           1);
    
    graphics.transitionImageLayout(canvasTextureImage,
                                   VK_FORMAT_R8G8B8A8_SRGB,
                                   VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                                   VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                                   1);
    
    graphics.textureImages.push_back(canvasTextureImage);
    graphics.textureImageMemories.push_back(canvasTextureImageMemory);
    graphics.textureImageViews.push_back(canvasTextureImageView);
    
    // load canvas quad
    
    auto canvasQuad = Graphics::loadCanvasQuad();
    graphics.pushModel(canvasQuad); // todo: which objs to draw, with which materials
    
    // finish rest of vulkan setup
    
    graphics.initRender(); // vertex buffers + swap chain pipeline

    // brush pipeline
    
    // brush render pass
    
    graphics.createRenderPass(brushRenderPass,
                              VK_ATTACHMENT_LOAD_OP_LOAD);
    
    // brush descriptors
            
    graphics.createFramebuffer(brushFrameBuffer,
                               brushRenderPass,
                               layerTextureImageView,
                               (int) config.paintConfig.CANVAS_WIDTH,
                               (int) config.paintConfig.CANVAS_HEIGHT);
    
    graphics.createDescriptorSetLayout(brushDescriptorSetLayout);
    
    graphics.createDescriptorPool(brushDescriptorPool);
    
    graphics.createDescriptorSets(brushTextureImageView,
                                  brushDescriptorSets,
                                  brushDescriptorSetLayout,
                                  brushDescriptorPool);
    
    // brush graphics pipeline
    
    auto brushVertShaderCode = Graphics::readFile(config.paintConfig.BRUSH_VERT_SHADER_PATH);
    auto brushFragShaderCode = Graphics::readFile(config.paintConfig.BRUSH_FRAG_SHADER_PATH);
    
    VkShaderModule brushVertShaderModule = graphics.createShaderModule(brushVertShaderCode);
    VkShaderModule brushFragShaderModule = graphics.createShaderModule(brushFragShaderCode);

    VkPipelineShaderStageCreateInfo brushVertShaderStageInfo{};
    brushVertShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    brushVertShaderStageInfo.stage = VK_SHADER_STAGE_VERTEX_BIT;
    brushVertShaderStageInfo.module = brushVertShaderModule;
    brushVertShaderStageInfo.pName = "main";

    VkPipelineShaderStageCreateInfo brushFragShaderStageInfo{};
    brushFragShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    brushFragShaderStageInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    brushFragShaderStageInfo.module = brushFragShaderModule;
    brushFragShaderStageInfo.pName = "main";
    
    VkPipelineShaderStageCreateInfo brushShaderStages[] = { brushVertShaderStageInfo, brushFragShaderStageInfo };
    
    VkPushConstantRange pushConstantRange{};
    pushConstantRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
    pushConstantRange.offset = 0;
    pushConstantRange.size = sizeof(float) * 4; // pos(vec2) + size(vec2) => 4 floats
    
    graphics.createPipeline(brushPipeline,
                            brushDescriptorSetLayout,
                            brushPipelineLayout,
                            brushRenderPass,
                            brushShaderStages,
                            pushConstantRange);
    
    vkDestroyShaderModule(graphics.device, brushVertShaderModule, nullptr);
    vkDestroyShaderModule(graphics.device, brushFragShaderModule, nullptr);
    
    // layer pipeline
    
    // layer render pass
    
    graphics.createRenderPass(layerRenderPass,
                              VK_ATTACHMENT_LOAD_OP_CLEAR);
    
    // layer descriptors
            
    graphics.createFramebuffer(layerFrameBuffer,
                               layerRenderPass,
                               canvasTextureImageView,
                               (int) config.paintConfig.CANVAS_WIDTH,
                               (int) config.paintConfig.CANVAS_HEIGHT);
    
    graphics.createDescriptorSetLayout(layerDescriptorSetLayout);
    
    graphics.createDescriptorPool(layerDescriptorPool);
    
    graphics.createDescriptorSets(layerTextureImageView,
                                  layerDescriptorSets,
                                  layerDescriptorSetLayout,
                                  layerDescriptorPool);
    
    // layer graphics pipeline
    
    auto layerVertShaderCode = Graphics::readFile(config.paintConfig.LAYER_VERT_SHADER_PATH);
    auto layerFragShaderCode = Graphics::readFile(config.paintConfig.LAYER_FRAG_SHADER_PATH);
    
    VkShaderModule layerVertShaderModule = graphics.createShaderModule(layerVertShaderCode);
    VkShaderModule layerFragShaderModule = graphics.createShaderModule(layerFragShaderCode);

    VkPipelineShaderStageCreateInfo layerVertShaderStageInfo{};
    layerVertShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    layerVertShaderStageInfo.stage = VK_SHADER_STAGE_VERTEX_BIT;
    layerVertShaderStageInfo.module = layerVertShaderModule;
    layerVertShaderStageInfo.pName = "main";

    VkPipelineShaderStageCreateInfo layerFragShaderStageInfo{};
    layerFragShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    layerFragShaderStageInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    layerFragShaderStageInfo.module = layerFragShaderModule;
    layerFragShaderStageInfo.pName = "main";
    
    VkPipelineShaderStageCreateInfo layerShaderStages[] = { layerVertShaderStageInfo, layerFragShaderStageInfo };
    
    graphics.createPipeline(layerPipeline,
                            layerDescriptorSetLayout,
                            layerPipelineLayout,
                            layerRenderPass,
                            layerShaderStages);
    
    vkDestroyShaderModule(graphics.device, layerVertShaderModule, nullptr);
    vkDestroyShaderModule(graphics.device, layerFragShaderModule, nullptr);
    
    // todo: move camera stuff here
    
    
    
}

void Application::draw() {
    
    uint32_t imageIndex;
    
    graphics.startFrame(imageIndex);
    
    graphics.updateGlobalUBO();
    graphics.updateInstanceSSBOs();
    
    VkCommandBuffer& commandBuffer = graphics.commandBuffers[graphics.currentFrame];
    
    // brush to layer
    
    // screen space to world space
    
    int windowWidth, windowHeight;
    glfwGetWindowSize(graphics.window, &windowWidth, &windowHeight);
    
    float ndcX = (inputSystem.xpos / windowWidth) * 2.0f - 1.0f;
    float ndcY = 1.0f - (inputSystem.ypos / windowHeight) * 2.0f;
    
    float aspect = (float) windowWidth / windowHeight;
    float tanHalfFovy = 0.4142f; // hard coded for 45 deg
    
    struct BrushPC { float pos[2]; float size[2]; } pc;
    pc.pos[0] = ndcX * graphics.depth * tanHalfFovy * aspect;
    pc.pos[1] = ndcY * graphics.depth * tanHalfFovy;
    
    pc.size[0] = graphics.brushSize; // translate to px size
    pc.size[1] = graphics.brushSize;
    
    graphics.draw(commandBuffer,
                  brushRenderPass,
                  brushFrameBuffer,
                  config.paintConfig.CANVAS_WIDTH,
                  config.paintConfig.CANVAS_HEIGHT,
                  brushPipeline,
                  config.paintConfig.CANVAS_WIDTH,
                  config.paintConfig.CANVAS_HEIGHT,
                  config.paintConfig.CANVAS_WIDTH, // could be optimized
                  config.paintConfig.CANVAS_HEIGHT, // could be optimized
                  brushPipelineLayout,
                  brushDescriptorSets,
                  pc);
        
    // layer to canvas
    
    // todo: minimize the number of image transitions
    
    graphics.transitionImageLayout(commandBuffer,
                                   layerTextureImage,
                                   VK_FORMAT_R8G8B8A8_SRGB,
                                   VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                                   VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                                   1);
    
    graphics.transitionImageLayout(commandBuffer,
                                   canvasTextureImage,
                                   VK_FORMAT_R8G8B8A8_SRGB,
                                   VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                                   VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                                   1);

    graphics.draw(commandBuffer,
                  layerRenderPass,
                  layerFrameBuffer,
                  config.paintConfig.CANVAS_WIDTH,
                  config.paintConfig.CANVAS_HEIGHT,
                  layerPipeline,
                  config.paintConfig.CANVAS_WIDTH,
                  config.paintConfig.CANVAS_HEIGHT,
                  config.paintConfig.CANVAS_WIDTH,
                  config.paintConfig.CANVAS_HEIGHT,
                  layerPipelineLayout,
                  layerDescriptorSets);
    
    graphics.transitionImageLayout(commandBuffer,
                                   layerTextureImage,
                                   VK_FORMAT_R8G8B8A8_SRGB,
                                   VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                                   VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                                   1);

    graphics.transitionImageLayout(commandBuffer,
                                   canvasTextureImage,
                                   VK_FORMAT_R8G8B8A8_SRGB,
                                   VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                                   VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                                   1);
    
    // canvas to swapchain
    
    graphics.submitFrame(imageIndex);
    
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
    InputSystem* inputSystem = static_cast<InputSystem*>(glfwGetWindowUserPointer(window));
    if (!inputSystem) return;
    inputSystem->mouse_button_callback(button, action, mods);
}

void mouse_position_callback(GLFWwindow* window, double xpos, double ypos) {
    InputSystem* inputSystem = static_cast<InputSystem*>(glfwGetWindowUserPointer(window));
    if (!inputSystem) return;
    inputSystem->mouse_position_callback(xpos, ypos);
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    InputSystem* inputSystem = static_cast<InputSystem*>(glfwGetWindowUserPointer(window));
    if (!inputSystem) return;
    inputSystem->key_callback(key, action);
}

void Application::run() {
    
    // register inputs
        
    glfwSetWindowUserPointer(graphics.window, &inputSystem);
    
    glfwSetMouseButtonCallback(graphics.window, mouse_button_callback);
    glfwSetCursorPosCallback(graphics.window, mouse_position_callback);
    glfwSetKeyCallback(graphics.window, key_callback);

    // main loop
    
    draw();
    
    // random paint stuff
    graphics.brushSize = 0.25f;
    
    // actual main loop
    while (!glfwWindowShouldClose(graphics.window)) {
        glfwPollEvents();
        if (inputSystem.pressed) { // todo: better architecture
            draw();
            inputSystem.reset();
        } else if (inputSystem.leftBracketPressed) {
            graphics.brushSize += 0.01f;
            inputSystem.reset();
        } else if (inputSystem.rightBracketPressed) {
            if (graphics.brushSize > 0.0f) {
                graphics.brushSize -= 0.01f;
            }
            inputSystem.reset();
        }
    }
    vkDeviceWaitIdle(graphics.device);
    
    // todo: add logging for average draw call timing
        // start time
        // count
        // end time / count
    
}

void Application::cleanup() {
    
    vkDestroyPipeline(graphics.device, brushPipeline, nullptr);
    vkDestroyPipelineLayout(graphics.device, brushPipelineLayout, nullptr);
    vkDestroyRenderPass(graphics.device, brushRenderPass, nullptr);
    
    vkDestroyFramebuffer(graphics.device, brushFrameBuffer, nullptr);
    
    vkDestroyPipeline(graphics.device, layerPipeline, nullptr);
    vkDestroyPipelineLayout(graphics.device, layerPipelineLayout, nullptr);
    vkDestroyRenderPass(graphics.device, layerRenderPass, nullptr);
    
    vkDestroyFramebuffer(graphics.device, layerFrameBuffer, nullptr);
    
    vkDestroyDescriptorPool(graphics.device, brushDescriptorPool, nullptr);
    vkDestroyDescriptorPool(graphics.device, layerDescriptorPool, nullptr);
    
    vkDestroyImageView(graphics.device, brushTextureImageView, nullptr);
    vkDestroyImage(graphics.device, brushTextureImage, nullptr);
    vkFreeMemory(graphics.device, brushTextureImageMemory, nullptr);

    vkDestroyImageView(graphics.device, layerTextureImageView, nullptr);
    vkDestroyImage(graphics.device, layerTextureImage, nullptr);
    vkFreeMemory(graphics.device, layerTextureImageMemory, nullptr);
    
    vkDestroyDescriptorSetLayout(graphics.device, brushDescriptorSetLayout, nullptr);
    vkDestroyDescriptorSetLayout(graphics.device, layerDescriptorSetLayout, nullptr);
    
    graphics.cleanup();

}

int main() { // simple application runner
     try {
         Application app;
         app.init();
         app.run();
         app.cleanup();
     } catch (const std::exception& e) {
         std::cerr << e.what() << std::endl;
         return EXIT_FAILURE;
     }
     return EXIT_SUCCESS;
}
