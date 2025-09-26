//
//  vulkan_test.cpp
//  engine
//
//  Created by Daniel Cho on 9/23/25.
//

// minimal vulkan app

#include <vulkan/vulkan.h>
#include <iostream>
#include <vector>

int main() {
   uint32_t apiVersion = 0;
   if (vkEnumerateInstanceVersion(&apiVersion) != VK_SUCCESS) {
       std::cout << "vkEnumerateInstanceVersion failed\n";
       return 1;
   }
   uint32_t major = VK_VERSION_MAJOR(apiVersion);
   uint32_t minor = VK_VERSION_MINOR(apiVersion);
   uint32_t patch = VK_VERSION_PATCH(apiVersion);
   std::cout << "Vulkan loader reports version " << major << "." << minor << "." << patch << "\n";

   // --- Validation layers ---
   std::vector<const char*> layers = {
       "VK_LAYER_KHRONOS_validation"
   };

   // --- Enable portability enumeration on macOS (MoltenVK) ---
   std::vector<const char*> extensions = {
       VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME
   };

   VkApplicationInfo appInfo{};
   appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
   appInfo.pApplicationName = "Vulkan Example";
   appInfo.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
   appInfo.pEngineName = "No Engine";
   appInfo.engineVersion = VK_MAKE_VERSION(1, 0, 0);
   appInfo.apiVersion = VK_API_VERSION_1_2; // at least 1.2 for portability

   VkInstanceCreateInfo instanceInfo{};
   instanceInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
   instanceInfo.pApplicationInfo = &appInfo;
   instanceInfo.enabledExtensionCount = static_cast<uint32_t>(extensions.size());
   instanceInfo.ppEnabledExtensionNames = extensions.data();
   instanceInfo.enabledLayerCount = static_cast<uint32_t>(layers.size());
   instanceInfo.ppEnabledLayerNames = layers.data();
   instanceInfo.flags |= VK_INSTANCE_CREATE_ENUMERATE_PORTABILITY_BIT_KHR;

   VkInstance instance;
   if (vkCreateInstance(&instanceInfo, nullptr, &instance) != VK_SUCCESS) {
       std::cerr << "vkCreateInstance failed\n";
       return 2;
   }

   uint32_t gpuCount = 0;
   vkEnumeratePhysicalDevices(instance, &gpuCount, nullptr);
   std::cout << "Physical devices found: " << gpuCount << "\n";

   if (gpuCount > 0) {
       std::vector<VkPhysicalDevice> devices(gpuCount);
       vkEnumeratePhysicalDevices(instance, &gpuCount, devices.data());
       for (uint32_t i = 0; i < gpuCount; ++i) {
           VkPhysicalDeviceProperties props;
           vkGetPhysicalDeviceProperties(devices[i], &props);
           std::cout << "Device " << i << ": " << props.deviceName << "\n";
       }
   }

   vkDestroyInstance(instance, nullptr);
   return 0;
}
