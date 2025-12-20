#version 450

layout(binding = 2) uniform sampler2D textures[4]; // NUM_TEXTURES

layout(location = 0) in vec3 fragColor;
layout(location = 1) in vec2 fragTexCoord;

layout(location = 0) out vec4 outColor;

layout(push_constant) uniform PushConstants {
    int materialIndex;
} pc;

void main() {
    outColor = texture(textures[pc.materialIndex], fragTexCoord);
}
