#!/bin/bash

cd src/graphics/shaders

/Users/danielcho/VulkanSDK/1.4.313.0/macOS/bin/glslc shader.vert -o vert.spv
/Users/danielcho/VulkanSDK/1.4.313.0/macOS/bin/glslc shader.frag -o frag.spv
