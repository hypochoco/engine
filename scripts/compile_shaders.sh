#!/bin/bash

rm resources/vert.spv
rm resources/frag.spv

/Users/danielcho/VulkanSDK/1.4.313.0/macOS/bin/glslc src/graphics/shaders/shader.vert -o resources/vert.spv
/Users/danielcho/VulkanSDK/1.4.313.0/macOS/bin/glslc src/graphics/shaders/shader.frag -o resources/frag.spv
