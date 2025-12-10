//
//  brush_shader.frag
//  engine
//
//  Created by Daniel Cho on 12/7/25.
//

#version 450

layout(binding = 0) uniform sampler2D brushTex;

layout(location = 0) in vec2 outUV;
layout(location = 0) out vec4 outColor;

void main() {
    vec4 c = texture(brushTex, outUV);
    outColor = c;
//    outColor = vec4(1.0, 0.0, 0.0, c.a);
}
