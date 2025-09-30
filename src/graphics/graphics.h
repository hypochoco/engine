// //
// //  graphics.h
// //  engine
// //
// //  Created by Daniel Cho on 9/23/25.
// //
// //  minimal vulkan app header
// // 

// #pragma once

// #define GLFW_INCLUDE_VULKAN
// #include <GLFW/glfw3.h>
// #include <vector>

// class VulkanApplication {

// public:
//     VulkanApplication();
    
//     void run() {
//         initWindow();
//         initVulkan();
//         mainLoop();
//         cleanup();
//     }

// private:
//     GLFWwindow* window;

//     VkInstance instance;
//     VkDebugUtilsMessengerEXT debugMessenger;
//     VkSurfaceKHR surface;

//     VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
//     VkDevice device;

//     VkQueue graphicsQueue;
//     VkQueue presentQueue;

//     VkSwapchainKHR swapChain;
//     std::vector<VkImage> swapChainImages;
//     VkFormat swapChainImageFormat;
//     VkExtent2D swapChainExtent;
//     std::vector<VkImageView> swapChainImageViews;
//     std::vector<VkFramebuffer> swapChainFramebuffers;

//     VkRenderPass renderPass;
//     VkPipelineLayout pipelineLayout;
//     VkPipeline graphicsPipeline;

//     VkCommandPool commandPool;
//     std::vector<VkCommandBuffer> commandBuffers;
//     std::vector<VkSemaphore> imageAvailableSemaphores;
//     std::vector<VkSemaphore> renderFinishedSemaphores;
//     std::vector<VkFence> inFlightFences;
//     uint32_t currentFrame = 0;
    
//     bool framebufferResized = false;

// };

