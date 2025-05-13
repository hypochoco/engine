




// no more failures, but no triangle now... 








// #define GLFW_INCLUDE_VULKAN
// #include <GLFW/glfw3.h>
// #include <iostream>
// #include <stdexcept>
// #include <cstdlib>
// #include <vector>

// const uint32_t WIDTH = 800;
// const uint32_t HEIGHT = 600;

// class VulkanApp {
// public:
//     void run() {
//         initWindow();
//         initVulkan();
//         mainLoop();
//         cleanup();
//     }

// private:
//     GLFWwindow* window;
//     VkInstance instance;

//     void initWindow() {
//         glfwInit();
//         glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
//         window = glfwCreateWindow(WIDTH, HEIGHT, "Vulkan Triangle", nullptr, nullptr);
//     }

//     void initVulkan() {
//         createInstance();
//     }

//     void createInstance() {
//         VkApplicationInfo appInfo{};
//         appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
//         appInfo.pApplicationName = "Triangle";
//         appInfo.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
//         appInfo.pEngineName = "No Engine";
//         appInfo.engineVersion = VK_MAKE_VERSION(1, 0, 0);
//         appInfo.apiVersion = VK_API_VERSION_1_3;

//         uint32_t glfwExtensionCount = 0;
//         const char** glfwExtensions = glfwGetRequiredInstanceExtensions(&glfwExtensionCount);

//         std::vector<const char*> extensions(glfwExtensions, glfwExtensions + glfwExtensionCount);

//         // Add the VK_KHR_portability_enumeration extension
//         extensions.push_back(VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME);

//         std::cout << "Required Extensions:" << std::endl;
//         for (const char* ext : extensions) {
//             std::cout << ext << std::endl;
//         }

//         VkInstanceCreateInfo createInfo{};
//         createInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
//         createInfo.pApplicationInfo = &appInfo;
//         createInfo.enabledExtensionCount = static_cast<uint32_t>(extensions.size());
//         createInfo.ppEnabledExtensionNames = extensions.data();
//         createInfo.flags = VK_INSTANCE_CREATE_ENUMERATE_PORTABILITY_BIT_KHR;

//         // Enable validation layers in debug mode
//         const char* validationLayers[] = { "VK_LAYER_KHRONOS_validation" };
//         #ifdef DEBUG
//             createInfo.enabledLayerCount = 1;
//             createInfo.ppEnabledLayerNames = validationLayers;
//         #else
//             createInfo.enabledLayerCount = 0;
//         #endif

//         VkResult result = vkCreateInstance(&createInfo, nullptr, &instance);

//         if (result != VK_SUCCESS) {
//             std::cerr << "Failed to create Vulkan instance. Error code: " << result << std::endl;

//             if (result == VK_ERROR_LAYER_NOT_PRESENT) {
//                 std::cerr << "Validation layer not found. Ensure VK_LAYER_KHRONOS_validation is installed and enabled." << std::endl;
//             } else if (result == VK_ERROR_EXTENSION_NOT_PRESENT) {
//                 std::cerr << "Required extension not found. Ensure VK_KHR_portability_enumeration is enabled." << std::endl;
//             } else if (result == VK_ERROR_INCOMPATIBLE_DRIVER) {
//                 std::cerr << "Incompatible Vulkan driver version." << std::endl;
//             }

//             throw std::runtime_error("Failed to create Vulkan instance!");
//         }

//         std::cout << "Vulkan instance created successfully!" << std::endl;
//     }

//     void mainLoop() {
//         while (!glfwWindowShouldClose(window)) {
//             glfwPollEvents();
//         }
//     }

//     void cleanup() {
//         vkDestroyInstance(instance, nullptr);
//         glfwDestroyWindow(window);
//         glfwTerminate();
//     }
// };

// int main() {
//     VulkanApp app;

//     try {
//         app.run();
//     } catch (const std::exception& e) {
//         std::cerr << e.what() << std::endl;
//         return EXIT_FAILURE;
//     }

//     return EXIT_SUCCESS;
// }











#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>
#include <iostream>
#include <stdexcept>
#include <cstdlib>
#include <vector>

const uint32_t WIDTH = 800;
const uint32_t HEIGHT = 600;

class VulkanApp {
public:
    void run() {
        initWindow();
        initVulkan();
        mainLoop();
        cleanup();
    }

private:
    GLFWwindow* window;
    VkInstance instance;
    VkSurfaceKHR surface;
    VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
    VkDevice device;
    VkQueue graphicsQueue;

    void initWindow() {
        glfwInit();
        glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
        window = glfwCreateWindow(WIDTH, HEIGHT, "Vulkan Triangle", nullptr, nullptr);
    }

    void initVulkan() {
        createInstance();
        createSurface();
        pickPhysicalDevice();
        createLogicalDevice();
    }

    void createInstance() {
        VkApplicationInfo appInfo{};
        appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
        appInfo.pApplicationName = "Triangle";
        appInfo.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
        appInfo.pEngineName = "No Engine";
        appInfo.engineVersion = VK_MAKE_VERSION(1, 0, 0);
        appInfo.apiVersion = VK_API_VERSION_1_3;

        uint32_t glfwExtensionCount = 0;
        const char** glfwExtensions = glfwGetRequiredInstanceExtensions(&glfwExtensionCount);

        std::vector<const char*> extensions(glfwExtensions, glfwExtensions + glfwExtensionCount);

        // Add the VK_KHR_portability_enumeration extension
        extensions.push_back(VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME);

        std::cout << "Required Extensions:" << std::endl;
        for (const char* ext : extensions) {
            std::cout << ext << std::endl;
        }

        VkInstanceCreateInfo createInfo{};
        createInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
        createInfo.pApplicationInfo = &appInfo;
        createInfo.enabledExtensionCount = static_cast<uint32_t>(extensions.size());
        createInfo.ppEnabledExtensionNames = extensions.data();
        createInfo.flags = VK_INSTANCE_CREATE_ENUMERATE_PORTABILITY_BIT_KHR;

        // Enable validation layers in debug mode
        const char* validationLayers[] = { "VK_LAYER_KHRONOS_validation" };
        #ifdef DEBUG
            createInfo.enabledLayerCount = 1;
            createInfo.ppEnabledLayerNames = validationLayers;
        #else
            createInfo.enabledLayerCount = 0;
        #endif

        VkResult result = vkCreateInstance(&createInfo, nullptr, &instance);

        if (result != VK_SUCCESS) {
            std::cerr << "Failed to create Vulkan instance. Error code: " << result << std::endl;

            if (result == VK_ERROR_LAYER_NOT_PRESENT) {
                std::cerr << "Validation layer not found. Ensure VK_LAYER_KHRONOS_validation is installed and enabled." << std::endl;
            } else if (result == VK_ERROR_EXTENSION_NOT_PRESENT) {
                std::cerr << "Required extension not found. Ensure VK_KHR_portability_enumeration is enabled." << std::endl;
            } else if (result == VK_ERROR_INCOMPATIBLE_DRIVER) {
                std::cerr << "Incompatible Vulkan driver version." << std::endl;
            }

            throw std::runtime_error("Failed to create Vulkan instance!");
        }

        std::cout << "Vulkan instance created successfully!" << std::endl;
    }

    void createSurface() {
        if (glfwCreateWindowSurface(instance, window, nullptr, &surface) != VK_SUCCESS) {
            throw std::runtime_error("Failed to create window surface!");
        }
    }

    void pickPhysicalDevice() {
        uint32_t deviceCount = 0;
        vkEnumeratePhysicalDevices(instance, &deviceCount, nullptr);

        if (deviceCount == 0) {
            throw std::runtime_error("Failed to find GPUs with Vulkan support!");
        }

        std::vector<VkPhysicalDevice> devices(deviceCount);
        vkEnumeratePhysicalDevices(instance, &deviceCount, devices.data());

        for (const auto& device : devices) {
            if (isDeviceSuitable(device)) {
                physicalDevice = device;
                break;
            }
        }

        if (physicalDevice == VK_NULL_HANDLE) {
            throw std::runtime_error("Failed to find a suitable GPU!");
        }
    }

    bool isDeviceSuitable(VkPhysicalDevice device) {
        VkPhysicalDeviceProperties deviceProperties;
        vkGetPhysicalDeviceProperties(device, &deviceProperties);

        VkPhysicalDeviceFeatures deviceFeatures;
        vkGetPhysicalDeviceFeatures(device, &deviceFeatures);

        return deviceProperties.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU && deviceFeatures.geometryShader;
    }

    void createLogicalDevice() {
        float queuePriority = 1.0f;

        VkDeviceQueueCreateInfo queueCreateInfo{};
        queueCreateInfo.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
        queueCreateInfo.queueFamilyIndex = 0;
        queueCreateInfo.queueCount = 1;
        queueCreateInfo.pQueuePriorities = &queuePriority;

        VkDeviceCreateInfo createInfo{};
        createInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
        createInfo.queueCreateInfoCount = 1;
        createInfo.pQueueCreateInfos = &queueCreateInfo;

        if (vkCreateDevice(physicalDevice, &createInfo, nullptr, &device) != VK_SUCCESS) {
            throw std::runtime_error("Failed to create logical device!");
        }

        vkGetDeviceQueue(device, 0, 0, &graphicsQueue);
    }

    void mainLoop() {
        while (!glfwWindowShouldClose(window)) {
            glfwPollEvents();
        }
    }

    void cleanup() {
        vkDestroyDevice(device, nullptr);
        vkDestroySurfaceKHR(instance, surface, nullptr);
        vkDestroyInstance(instance, nullptr);
        glfwDestroyWindow(window);
        glfwTerminate();
    }
};

int main() {
    VulkanApp app;

    try {
        app.run();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

