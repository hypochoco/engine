//
//  layer_shader.frag
//  engine
//
//  Created by Daniel Cho on 12/11/25.
//

#version 450

layout(binding = 0) uniform sampler2D layerTexture;

layout(location = 0) in vec2 outUV;
layout(location = 0) out vec4 outColor;

void main() {
    vec4 c = texture(layerTexture, outUV);
    outColor = c;
}

