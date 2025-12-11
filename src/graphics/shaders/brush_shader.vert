//
//  brush_shader.vert
//  engine
//
//  Created by Daniel Cho on 12/7/25.
//

#version 450

layout(push_constant) uniform BrushPC {
    vec2 pos;  // world space coords (x, y)
    vec2 size; // brush pixel size (width, height)
} pc;

layout(location = 0) in vec3 inPosition; // vertex position from buffer
layout(location = 1) in vec3 inColor;    // vertex color (not used here)
layout(location = 2) in vec2 inTexCoord; // texture coordinates

layout(location = 0) out vec2 outUV;

void main() {
    
    // brush quad -> canvas quad
    
    gl_Position = vec4(inPosition.xy * pc.size + pc.pos, 0.0, 1.0);
    
    outUV = inTexCoord;
    
}
