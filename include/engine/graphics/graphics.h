// //
// //  graphics.h
// //  engine
// //
// //  Created by Daniel Cho on 9/23/25.
// //

#pragma once

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

// imports

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

struct Mesh {
    uint32_t firstVertex;
    uint32_t vertexCount;
    
    uint32_t firstIndex;
    uint32_t indexCount;
};

struct Material {
    uint32_t textureIndex;
};

struct GlobalUBO {
    alignas(16) glm::mat4 view;
    alignas(16) glm::mat4 proj;
    // todo: lights
};

struct InstanceSSBO {
    alignas(16) glm::mat4 model;
};

struct DrawJob {
    
    bool visible;
    
    uint32_t meshIndex;
    
    uint32_t firstMaterial;
    uint32_t materialCount;
    
    uint32_t firstInstance;
    uint32_t instanceCount;
    
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
    
    // constants

    static constexpr uint32_t WIDTH = 800;
    static constexpr uint32_t HEIGHT = 600;

    static constexpr int MAX_FRAMES_IN_FLIGHT = 3;
    static constexpr int NUM_TEXTURES = 16;
    static constexpr int MAX_ENTITIES = 16;
    
    // public graphics variables
    
    GLFWwindow* window;
    VkDevice device; // todo: not a fan of exposing this
    uint32_t currentFrame = 0;
    std::vector<VkCommandBuffer> commandBuffers;
    
    bool framebufferResized = false;
    
    // material variables
    
    std::vector<Material> materials;
    
    std::vector<VkImage> textureImages;
    std::vector<VkDeviceMemory> textureImageMemories;
    std::vector<VkImageView> textureImageViews;
    
    // helper functions
    
    static std::vector<char> readFile(const std::string& filename);
    
    VkShaderModule createShaderModule(const std::vector<char>& code);
    void destroyShaderModule(VkShaderModule& shaderModule);
    
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
    
    // material functions
    
    void loadTexture(std::string texturePath,
                     VkImageUsageFlags usage,
                     int mipLevels = 0);
    
    void createTexture(int texWidth,
                       int texHeight,
                       VkImageUsageFlags usage,
                       int mipLevels = 0);

    // model functions
    
    void loadQuad();
    
    void loadObj(const std::string& modelPath);
    
    // instance functions
    
    VkExtent2D getSwapChainExtent();
    
    void updateGlobalUBO(uint32_t currentFrame,
                         glm::mat4& view,
                         glm::mat4& proj);
    
    void addDrawJob(uint32_t meshIndex,
                    uint32_t firstMaterial,
                    uint32_t materialCount,
                    std::vector<glm::mat4> modelMatrices);
    void copyInstanceToBuffer(uint32_t currentFrame);
    
    // public graphics functions
        
    void initWindow();
    
    void setInstance(const VkInstance& instance);
    void createInstance();
    void setSurface(const VkSurfaceKHR& surface);
    void createSurface();
    void pickPhysicalDevice();
    void createLogicalDevice();
    void createSyncObjects();
    
    void createTextureSampler();
    void createSwapChain();
    void createSwapChainImageViews();
    void createSwapChainFramebuffers();
    void createSwapChainColorResources();
    void createSwapChainDepthResources();
    void cleanupSwapChain();
    void recreateSwapChain();
    
    void createSwapChainRenderPass();
    void createSwapChainDescriptorSetLayout();
    void createSwapChainGraphicsPipeline(const std::string& vertShaderPath,
                                         const std::string& fragShaderPath);
    void createSwapChainDescriptorPool();
    void createSwapChainDescriptorSets();
    
    void createCommandPool();
    void createCommandBuffers();
    
    void createVertexBuffer();
    void createIndexBuffer();
    void createUniformBuffers();
    
    void startFrame();
    void submitFrame();
    
    void cleanupVulkan();
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
    
    void createDescriptorPool(VkDescriptorPool& descriptorPool,
                              uint32_t allocation = 1);
    
    void createDescriptorSets(VkImageView& imageView,
                              std::vector<VkDescriptorSet>& descriptorSets,
                              VkDescriptorSetLayout& descriptorSetLayout,
                              VkDescriptorPool& descriptorPool,
                              uint32_t allocation = 1);
    
    void updateDescriptorSet(VkImageView& imageView,
                             VkDescriptorSet& descriptorSet,
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
        
    void recordBeginRenderPass(VkCommandBuffer& commandBuffer,
                               VkRenderPass& renderPass,
                               VkFramebuffer& frameBuffer,
                               const int& renderAreaWidth,
                               const int& renderAreaHeight,
                               VkPipeline& pipeline);
    void recordSetViewport(VkCommandBuffer& commandBuffer,
                           const float& viewportX,
                           const float& viewportY,
                           const float& viewportWidth,
                           const float& viewportHeight);
    void recordSetScissor(VkCommandBuffer& commandBuffer,
                          const int& scissorX,
                          const int& scissorY,
                          const uint32_t& scissorWidth,
                          const uint32_t& scissorHeight);
    void recordBindDescriptorSet(VkCommandBuffer& commandBuffer,
                                 VkPipelineLayout& pipelineLayout,
                                 std::vector<VkDescriptorSet>& descriptorSets);
    void recordPushConstant(VkCommandBuffer& commandBuffer,
                            VkPipelineLayout& pipelineLayout,
                            uint32_t pushConstantSize,
                            void* pushConstant);
    void recordDraw(VkCommandBuffer& commandBuffer);
    void recordEndRenderPass(VkCommandBuffer& commandBuffer);
    
    void deviceWaitIdle();
    void destroyPipeline(VkPipeline& pipeline);
    void destroyPipelineLayout(VkPipelineLayout& pipelineLayout);
    void destroyRenderPass(VkRenderPass& renderPass);
    void destroyFrameBuffer(VkFramebuffer& frameBuffer);
    void destroyDescriptorPool(VkDescriptorPool& descriptorPool);
    void destroyDescriptorSetLayout(VkDescriptorSetLayout& descriptorSetLayout);
    
private:
    
    // graphics variables
        
    VkInstance instance;
    VkDebugUtilsMessengerEXT debugMessenger;
    VkSurfaceKHR surface;

    VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
    VkSampleCountFlagBits msaaSamples = VK_SAMPLE_COUNT_1_BIT;

    VkQueue graphicsQueue;
    VkQueue presentQueue;
    
    VkCommandPool commandPool;
    
    VkSampler textureSampler;
    
    std::vector<VkSemaphore> imageAvailableSemaphores;
    std::vector<VkSemaphore> renderFinishedSemaphores;
    std::vector<VkFence> inFlightFences;
    
    GlobalUBO globalUBO;
    std::vector<VkBuffer> globalUniformBuffers;
    std::vector<VkDeviceMemory> globalUniformBuffersMemory;
    std::vector<void*> globalUniformBuffersMapped;

    // swap chain graphics variables
    
    VkImage colorImage;
    VkDeviceMemory colorImageMemory;
    VkImageView colorImageView;

    VkImage depthImage;
    VkDeviceMemory depthImageMemory;
    VkImageView depthImageView;
    
    VkSwapchainKHR swapChain;
    VkFormat swapChainImageFormat;
    VkExtent2D swapChainExtent;
    std::vector<VkImage> swapChainImages;
    std::vector<VkImageView> swapChainImageViews;
    std::vector<VkFramebuffer> swapChainFramebuffers;

    VkRenderPass renderPass;
    VkDescriptorSetLayout descriptorSetLayout;
    VkPipelineLayout pipelineLayout;
    VkPipeline graphicsPipeline;
    
    VkDescriptorPool descriptorPool;
    std::vector<VkDescriptorSet> descriptorSets;
    
    // mesh variables
    
    std::vector<Mesh> meshes;
    
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;

    VkBuffer vertexBuffer;
    VkDeviceMemory vertexBufferMemory;
    VkBuffer indexBuffer;
    VkDeviceMemory indexBufferMemory;
    
    std::vector<VkBuffer> instanceStorageBuffers;
    std::vector<VkDeviceMemory> instanceStorageBuffersMemory;
    std::vector<void*> instanceStorageBuffersMapped;
    
    // instance variables
    
    uint32_t imageIndex;
    
    std::vector<DrawJob> drawJobs;
    std::vector<glm::mat4> instanceModelMatrices;
        
    // graphics functions
    
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
    
    void populateDebugMessengerCreateInfo(VkDebugUtilsMessengerCreateInfoEXT& createInfo);
    static void framebufferResizeCallback(GLFWwindow* window, int width, int height);
    void setupDebugMessenger();
    
    bool isDeviceSuitable(VkPhysicalDevice device);
    bool checkDeviceExtensionSupport(VkPhysicalDevice device);
    QueueFamilyIndices findQueueFamilies(VkPhysicalDevice device);
    std::vector<const char*> getRequiredExtensions();
    bool checkValidationLayerSupport();

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
    void loadTexture(int texWidth,
                     int texHeight,
                     stbi_uc* pixels,
                     VkImageUsageFlags usage,
                     int mipLevels = 0);
    VkSampleCountFlagBits getMaxUsableSampleCount();
    void createTextureImageView(uint32_t mipLevels,
                                VkImage& textureImage,
                                VkImageView& textureImageView);
    VkImageView createImageView(VkImage image,
                                VkFormat format,
                                VkImageAspectFlags aspectFlags,
                                uint32_t mipLevels);
    void copyBufferToImage(VkBuffer buffer, VkImage image, uint32_t width, uint32_t height);
    
    VkSurfaceFormatKHR chooseSwapSurfaceFormat(const std::vector<VkSurfaceFormatKHR>& availableFormats);
    VkPresentModeKHR chooseSwapPresentMode(const std::vector<VkPresentModeKHR>& availablePresentModes);
    VkExtent2D chooseSwapExtent(const VkSurfaceCapabilitiesKHR& capabilities);
    SwapChainSupportDetails querySwapChainSupport(VkPhysicalDevice device);
    
    VkFormat findDepthFormat();
    
    VkFormat findSupportedFormat(const std::vector<VkFormat>& candidates,
                                 VkImageTiling tiling,
                                 VkFormatFeatureFlags features);
    
    void recordSwapChainCommandBuffer(VkCommandBuffer commandBuffer);
        
};
