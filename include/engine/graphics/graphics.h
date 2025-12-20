// //
// //  graphics.h
// //  engine
// //
// //  Created by Daniel Cho on 9/23/25.
// //

#pragma once

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/hash.hpp>

#include <stb_image.h>

#include <iostream>
#include <fstream>
#include <stdexcept>
#include <algorithm>
#include <chrono>
#include <vector>
#include <cstring>
#include <cstdlib>
#include <cstdint>
#include <limits>
#include <array>
#include <optional>
#include <set>
#include <unordered_map>
#include <unordered_set>

#include "engine/config.h"
#include "engine/input_system.h"

// flags

#ifdef NDEBUG
#define ENABLE_VALIDATION_LAYERS false
#else
#define ENABLE_VALIDATION_LAYERS true
#endif

#if defined(__APPLE__) && defined(__MACH__) // macOS or iOS
#define MACOS true
#else
#define MACOS false
#endif

// model structs

struct Vertex {
    glm::vec3 pos;
    glm::vec3 color;
    glm::vec2 texCoord;

    static VkVertexInputBindingDescription getBindingDescription() {
        VkVertexInputBindingDescription bindingDescription{};
        bindingDescription.binding = 0;
        bindingDescription.stride = sizeof(Vertex);
        bindingDescription.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

        return bindingDescription;
    }

    static std::array<VkVertexInputAttributeDescription, 3> getAttributeDescriptions() {
        std::array<VkVertexInputAttributeDescription, 3> attributeDescriptions{};

        attributeDescriptions[0].binding = 0;
        attributeDescriptions[0].location = 0;
        attributeDescriptions[0].format = VK_FORMAT_R32G32B32_SFLOAT;
        attributeDescriptions[0].offset = offsetof(Vertex, pos);

        attributeDescriptions[1].binding = 0;
        attributeDescriptions[1].location = 1;
        attributeDescriptions[1].format = VK_FORMAT_R32G32B32_SFLOAT;
        attributeDescriptions[1].offset = offsetof(Vertex, color);

        attributeDescriptions[2].binding = 0;
        attributeDescriptions[2].location = 2;
        attributeDescriptions[2].format = VK_FORMAT_R32G32_SFLOAT;
        attributeDescriptions[2].offset = offsetof(Vertex, texCoord);

        return attributeDescriptions;
    }

    bool operator==(const Vertex& other) const {
        return pos == other.pos && color == other.color && texCoord == other.texCoord;
    }
};

namespace std {
    template<> struct hash<Vertex> {
        size_t operator()(Vertex const& vertex) const {
            return ((hash<glm::vec3>()(vertex.pos) ^ (hash<glm::vec3>()(vertex.color) << 1)) >> 1) ^ (hash<glm::vec2>()(vertex.texCoord) << 1);
        }
    };
}

struct Material {
    std::string name;
    uint32_t textureIndex;
};

struct ObjData {
    std::vector<Material> modelMaterials;
    std::unordered_map<int, std::vector<Vertex>> uniqueVerticesMap;
    std::unordered_map<int, std::vector<uint32_t>> uniqueIndicesMap;
    
    // todo: normals
    
};

struct Submesh {
    uint32_t firstIndex;
    uint32_t indexCount;
    uint32_t firstVertex;
    uint32_t vertexCount;
    uint32_t materialIndex;
};

struct Model {
    uint32_t firstMaterialIndex;
    uint32_t materialCount;
    std::vector<Submesh> submeshes;
};

struct GlobalUBO {
    alignas(16) glm::mat4 view;
    alignas(16) glm::mat4 proj;
    // todo: lights
};

struct InstanceSSBO {
    alignas(16) glm::mat4 model;
};

// vulkan structs

struct SwapChainSupportDetails {
    VkSurfaceCapabilitiesKHR capabilities;
    std::vector<VkSurfaceFormatKHR> formats;
    std::vector<VkPresentModeKHR> presentModes;
};

struct QueueFamilyIndices {
    std::optional<uint32_t> graphicsFamily;
    std::optional<uint32_t> presentFamily;

    bool isComplete() {
        return graphicsFamily.has_value() && presentFamily.has_value();
    }
};

// graphics class

class Graphics {
    
public:
    
    // public variables
    
    GLFWwindow* window;
    VkDevice device;
        
    std::vector<VkImage> textureImages;
    std::vector<VkDeviceMemory> textureImageMemories;
    std::vector<VkImageView> textureImageViews;
    
    std::vector<VkCommandBuffer> commandBuffers;
    uint32_t currentFrame = 0;
    
    // public functions
    
    Graphics() : config(AppConfig::instance()) {};
    
    static std::vector<char> readFile(const std::string& filename);
    static ObjData loadObj(const std::string& modelPath);
    static ObjData loadCanvasQuad();
    
    void transitionImageLayout(VkCommandBuffer& commandBuffer,
                               VkImage image,
                               VkFormat format,
                               VkImageLayout oldLayout,
                               VkImageLayout newLayout,
                               uint32_t mipLevels);
    void transitionImageLayout(VkImage image,
                               VkFormat format,
                               VkImageLayout oldLayout,
                               VkImageLayout newLayout,
                               uint32_t mipLevels);

    VkShaderModule createShaderModule(const std::vector<char>& code);
    
    void loadTexture(int texWidth,
                     int texHeight,
                     stbi_uc* pixels,
                     VkImage& textureImage,
                     VkDeviceMemory& textureImageMemory,
                     VkImageView& textureImageView,
                     VkImageUsageFlags usage,
                     int mipLevels);
    void loadTexture(std::string texturePath,
                     VkImage& textureImage,
                     VkDeviceMemory& textureImageMemory,
                     VkImageView& textureImageView,
                     VkImageUsageFlags usage,
                     int mipLevels = 0);
    void createTexture(int texWidth,
                       int texHeight,
                       VkImage& textureImage,
                       VkDeviceMemory& textureImageMemory,
                       VkImageView& textureImageView,
                       VkImageUsageFlags usage,
                       int mipLevels = 0);
    
    void pushModel(ObjData& objData);
    
    void initWindow();
    void initVulkan();
    void initRender();
    void run();
    
    void startFrame(uint32_t& imageIndex);
    void submitFrame(uint32_t& imageIndex);
    
    void updateGlobalUBO(uint32_t currentFrame,
                         glm::mat4& view,
                         glm::mat4& proj);
    void updateInstanceSSBOs(uint32_t currentFrame,
                             std::vector<InstanceSSBO>& instances);
    
    void updateGlobalUBO(glm::mat4& view,
                         glm::mat4& proj);
    void updateInstanceSSBOs(std::vector<InstanceSSBO>& instances);
    
    void cleanup();
    
    // custom render functions
    
    void createRenderPass(VkRenderPass& renderPass,
                          VkAttachmentLoadOp loadOp);
    
    void createFramebuffer(VkFramebuffer& frameBuffer,
                           VkRenderPass& renderPass,
                           VkImageView& imageView,
                           int width,
                           int height);
    
    void createDescriptorSetLayout(VkDescriptorSetLayout& descriptorSetLayout);
    
    void createDescriptorPool(VkDescriptorPool& descriptorPool);
    
    void createDescriptorSets(VkImageView& imageView,
                              std::vector<VkDescriptorSet>& descriptorSets,
                              VkDescriptorSetLayout& descriptorSetLayout,
                              VkDescriptorPool& descriptorPool);
    
    void createPipeline(VkPipeline& pipeline,
                        VkDescriptorSetLayout& descriptorSetLayout,
                        VkPipelineLayout& pipelineLayout,
                        VkRenderPass& renderPass,
                        VkPipelineShaderStageCreateInfo* shaderStages);

    void createPipeline(VkPipeline& pipeline,
                        VkDescriptorSetLayout& descriptorSetLayout,
                        VkPipelineLayout& pipelineLayout,
                        VkRenderPass& renderPass,
                        VkPipelineShaderStageCreateInfo* shaderStages,
                        VkPushConstantRange& pushConstantRange);
    
    void draw(VkCommandBuffer& commandBuffer,
              VkRenderPass& renderPass,
              VkFramebuffer& frameBuffer,
              int renderAreaWidth,
              int renderAreaHeight,
              VkPipeline& pipeline,
              int viewportWidth,
              int viewportHeight,
              int scissorWidth,
              int scissorHeight,
              VkPipelineLayout& pipelineLayout,
              std::vector<VkDescriptorSet>& descriptorSets);
    
    template<typename T>
    void draw(VkCommandBuffer& commandBuffer,
                        VkRenderPass& renderPass,
                        VkFramebuffer& frameBuffer,
                        int renderAreaWidth,
                        int renderAreaHeight,
                        VkPipeline& pipeline,
                        int viewportWidth,
                        int viewportHeight,
                        int scissorWidth,
                        int scissorHeight,
                        VkPipelineLayout& pipelineLayout,
                        std::vector<VkDescriptorSet>& descriptorSets,
                        T& pushConstant) {
        
        VkRenderPassBeginInfo renderPassInfo{};
        renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
        renderPassInfo.renderPass = renderPass;
        renderPassInfo.framebuffer = frameBuffer;
        renderPassInfo.renderArea.offset = {0, 0};
        renderPassInfo.renderArea.extent = { (uint32_t)renderAreaWidth, (uint32_t)renderAreaHeight };

        VkClearValue clearPaint = { {0.0f, 0.0f, 0.0f, 0.0f} };
        renderPassInfo.clearValueCount = 1;
        renderPassInfo.pClearValues = &clearPaint;
        
        vkCmdBeginRenderPass(commandBuffer, &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE);
        
        vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
        
        VkViewport viewport{};
        viewport.x = 0.0f;
        viewport.y = 0.0f;
        viewport.width = (float) viewportWidth;
        viewport.height = (float) viewportHeight;
        viewport.minDepth = 0.0f;
        viewport.maxDepth = 1.0f;
        vkCmdSetViewport(commandBuffer, 0, 1, &viewport);

        VkRect2D scissor{};
        scissor.offset = {0, 0};
        scissor.extent = { (uint32_t)scissorWidth, (uint32_t)scissorHeight };
        vkCmdSetScissor(commandBuffer, 0, 1, &scissor);
        
        VkBuffer vertexBuffers[] = { vertexBuffer };
        VkDeviceSize offsets[] = {0};
        vkCmdBindVertexBuffers(commandBuffer, 0, 1, vertexBuffers, offsets);
        vkCmdBindIndexBuffer(commandBuffer, indexBuffer, 0, VK_INDEX_TYPE_UINT32);

        vkCmdBindDescriptorSets(
            commandBuffer,
            VK_PIPELINE_BIND_POINT_GRAPHICS,
            pipelineLayout,
            0,
            1,
            &descriptorSets[currentFrame],
            0,
            nullptr
        );
        
        vkCmdPushConstants(commandBuffer,
                           pipelineLayout,
                           VK_SHADER_STAGE_VERTEX_BIT,
                           0,
                           sizeof(pushConstant),
                           &pushConstant);
        
        vkCmdDrawIndexed(commandBuffer, 6, 1, 0, 0, 0);

        vkCmdEndRenderPass(commandBuffer);
        
    }

private:
    
    // external
    
    AppConfig& config; // todo: not needed
    
    // variables
        
    VkInstance instance;
    VkDebugUtilsMessengerEXT debugMessenger;
    VkSurfaceKHR surface;

    VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
    VkSampleCountFlagBits msaaSamples = VK_SAMPLE_COUNT_1_BIT;

    VkQueue graphicsQueue;
    VkQueue presentQueue;

    VkSwapchainKHR swapChain;
    std::vector<VkImage> swapChainImages;
    VkFormat swapChainImageFormat;
    VkExtent2D swapChainExtent;
    std::vector<VkImageView> swapChainImageViews;
    std::vector<VkFramebuffer> swapChainFramebuffers;
    
    VkRenderPass renderPass;
    VkDescriptorSetLayout descriptorSetLayout;
    VkPipelineLayout pipelineLayout;
    VkPipeline graphicsPipeline;
    
    VkCommandPool commandPool;
    
    VkImage colorImage;
    VkDeviceMemory colorImageMemory;
    VkImageView colorImageView;

    VkImage depthImage;
    VkDeviceMemory depthImageMemory;
    VkImageView depthImageView;
    
    VkSampler textureSampler; // only need one for now
    
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;

    std::vector<Model> models;
    std::vector<Material> materials;
    
    VkBuffer vertexBuffer;
    VkDeviceMemory vertexBufferMemory;
    VkBuffer indexBuffer;
    VkDeviceMemory indexBufferMemory;
    
    GlobalUBO globalUBO;

    std::vector<VkBuffer> globalUniformBuffers;
    std::vector<VkDeviceMemory> globalUniformBuffersMemory;
    std::vector<void*> globalUniformBuffersMapped;

    std::vector<VkBuffer> instanceStorageBuffers;
    std::vector<VkDeviceMemory> instanceStorageBuffersMemory;
    std::vector<void*> instanceStorageBuffersMapped;
    
    VkDescriptorPool descriptorPool;
    std::vector<VkDescriptorSet> descriptorSets;

    std::vector<VkSemaphore> imageAvailableSemaphores;
    std::vector<VkSemaphore> renderFinishedSemaphores;
    std::vector<VkFence> inFlightFences;

    bool framebufferResized = false;
    
    // functions
    
    static VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
                                                        VkDebugUtilsMessageTypeFlagsEXT messageType,
                                                        const VkDebugUtilsMessengerCallbackDataEXT* pCallbackData,
                                                        void* pUserData);
    
    void createBuffer(VkDeviceSize size,
                      VkBufferUsageFlags usage,
                      VkMemoryPropertyFlags properties,
                      VkBuffer& buffer,
                      VkDeviceMemory& bufferMemory);
    VkCommandBuffer beginSingleTimeCommands();
    void endSingleTimeCommands(VkCommandBuffer commandBuffer);
    void copyBuffer(VkBuffer srcBuffer, VkBuffer dstBuffer, VkDeviceSize size);
    uint32_t findMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties);

    void createInstance();
    void populateDebugMessengerCreateInfo(VkDebugUtilsMessengerCreateInfoEXT& createInfo);
    static void framebufferResizeCallback(GLFWwindow* window, int width, int height);
    void setupDebugMessenger();
    void createSurface();
    void pickPhysicalDevice();
    void createLogicalDevice();
    void createSyncObjects();
    
    bool isDeviceSuitable(VkPhysicalDevice device);
    bool checkDeviceExtensionSupport(VkPhysicalDevice device);
    QueueFamilyIndices findQueueFamilies(VkPhysicalDevice device);
    std::vector<const char*> getRequiredExtensions();
    bool checkValidationLayerSupport();

    void createTextureSampler();
    void createImage(uint32_t width,
                     uint32_t height,
                     uint32_t mipLevels,
                     VkSampleCountFlagBits numSamples,
                     VkFormat format,
                     VkImageTiling tiling,
                     VkImageUsageFlags usage,
                     VkMemoryPropertyFlags properties,
                     VkImage& image,
                     VkDeviceMemory& imageMemory);
    void stageTextureImage(int texWidth,
                           int texHeight,
                           VkDeviceSize imageSize,
                           stbi_uc* pixels,
                           uint32_t mipLevels,
                           VkImage& textureImage,
                           VkDeviceMemory& textureImageMemory,
                           VkImageUsageFlags usage);
    void generateMipmaps(VkImage image,
                         VkFormat imageFormat,
                         int32_t texWidth,
                         int32_t texHeight,
                         uint32_t mipLevels);
    VkSampleCountFlagBits getMaxUsableSampleCount();
    void createTextureImageView(uint32_t mipLevels,
                                VkImage& textureImage,
                                VkImageView& textureImageView);
    VkImageView createImageView(VkImage image,
                                VkFormat format,
                                VkImageAspectFlags aspectFlags,
                                uint32_t mipLevels);
    void copyBufferToImage(VkBuffer buffer, VkImage image, uint32_t width, uint32_t height);
    
    void createVertexBuffer();
    void createIndexBuffer();
    void createUniformBuffers();
    
    VkSurfaceFormatKHR chooseSwapSurfaceFormat(const std::vector<VkSurfaceFormatKHR>& availableFormats);
    VkPresentModeKHR chooseSwapPresentMode(const std::vector<VkPresentModeKHR>& availablePresentModes);
    VkExtent2D chooseSwapExtent(const VkSurfaceCapabilitiesKHR& capabilities);
    SwapChainSupportDetails querySwapChainSupport(VkPhysicalDevice device);
    
    void createSwapChain();
    void createSwapChainImageViews();
    void createSwapChainFramebuffers();
    void createSwapChainColorResources();
    VkFormat findDepthFormat();
    void createSwapChainDepthResources();
    void cleanupSwapChain();
    void recreateSwapChain();
    
    void createSwapChainRenderPass();
    void createSwapChainDescriptorSetLayout();
    void createSwapChainGraphicsPipeline(const std::string& vertShaderPath,
                                         const std::string& fragShaderPath);
    void createSwapChainDescriptorPool();
    void createSwapChainDescriptorSets(std::vector<VkImageView>& imageViews);
    
    void createCommandPool();
        VkFormat findSupportedFormat(const std::vector<VkFormat>& candidates,
                                 VkImageTiling tiling,
                                 VkFormatFeatureFlags features);
    void createCommandBuffers();
            
    void recordSwapChainCommandBuffer(VkCommandBuffer commandBuffer, uint32_t imageIndex);
    void recordPaintCommandBuffer(VkCommandBuffer commandBuffer, uint32_t imageIndex);
        
};
