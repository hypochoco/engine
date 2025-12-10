//
//  brush_shader.vert
//  engine
//
//  Created by Daniel Cho on 12/7/25.
//

#version 450

layout(push_constant) uniform BrushPC {
    vec2 pos;      // pixel coords of center on canvas
    vec2 size;     // pixel size (width, height)
    vec2 canvas;   // canvas resolution (width, height)
} pc;

// Input vertex attributes from your canvas quad
layout(location = 0) in vec3 inPosition;  // vertex position from buffer
layout(location = 1) in vec3 inColor;     // vertex color (not used here)
layout(location = 2) in vec2 inTexCoord;  // texture coordinates

layout(location = 0) out vec2 outUV;

void main() {
    // inPosition is in [-1, 1] range from your loadCanvasQuad
    // inTexCoord is already in [0, 1] range
    
    // Scale the quad by the brush size (convert from NDC space to pixel space)
    // The input position is in [-1, 1], so multiply by 0.5 to get [-0.5, 0.5]
    vec2 local = inPosition.xy * 0.5 * pc.size;
    
    // Translate to brush position on canvas
    vec2 pixelPos = pc.pos + local;
    
    // Convert to NDC: (0..canvas) -> (-1..1)
    vec2 ndc = (pixelPos / pc.canvas) * 2.0 - 1.0;
    
    // Flip Y because Vulkan NDC Y is inverted vs. typical texture coords
    ndc.y = -ndc.y;
    
    gl_Position = vec4(ndc, 0.0, 1.0);
    
    // Use the texture coordinates from the vertex buffer
    outUV = inTexCoord;
}
