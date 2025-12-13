//
//  layer_shader.vert
//  engine
//
//  Created by Daniel Cho on 12/11/25.
//

#version 450

layout(location = 0) in vec3 inPosition; // brush quad vertex position
layout(location = 1) in vec3 inColor;    // vertex color (not used here) ??
layout(location = 2) in vec2 inTexCoord; // texture coordinates

layout(location = 0) out vec2 outUV;

void main() {
    gl_Position = vec4(inPosition.xy, 0.0, 1.0);
    outUV = inTexCoord;
}

