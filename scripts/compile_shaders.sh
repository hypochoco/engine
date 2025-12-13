#!/bin/bash

rm resources/vert.spv
rm resources/frag.spv

/Users/danielcho/VulkanSDK/1.4.313.0/macOS/bin/glslc src/graphics/shaders/shader.vert -o resources/vert.spv
/Users/danielcho/VulkanSDK/1.4.313.0/macOS/bin/glslc src/graphics/shaders/shader.frag -o resources/frag.spv

rm resources/brush_vert.spv
rm resources/brush_frag.spv

/Users/danielcho/VulkanSDK/1.4.313.0/macOS/bin/glslc src/graphics/shaders/brush_shader.vert -o resources/brush_vert.spv
/Users/danielcho/VulkanSDK/1.4.313.0/macOS/bin/glslc src/graphics/shaders/brush_shader.frag -o resources/brush_frag.spv

rm resources/layer_vert.spv
rm resources/layer_frag.spv

/Users/danielcho/VulkanSDK/1.4.313.0/macOS/bin/glslc src/graphics/shaders/layer_shader.vert -o resources/layer_vert.spv
/Users/danielcho/VulkanSDK/1.4.313.0/macOS/bin/glslc src/graphics/shaders/layer_shader.frag -o resources/layer_frag.spv

