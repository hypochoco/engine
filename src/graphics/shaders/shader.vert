#version 450

layout(binding = 0) uniform GlobalUBO {
    mat4 view;
    mat4 proj;
} globalUBO;

layout(set = 0, binding = 1) readonly buffer InstanceSSBO {
    mat4 model[];
} instanceSSBO;

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inColor;
layout(location = 2) in vec2 inTexCoord;

layout(location = 0) out vec3 fragColor;
layout(location = 1) out vec2 fragTexCoord;

layout(push_constant) uniform PushConstants {
    int materialIndex; // which texture to use for this submesh
} pc;

void main() {
    gl_Position = globalUBO.proj * globalUBO.view * instanceSSBO.model[gl_InstanceIndex] * vec4(inPosition, 1.0);
    fragColor = inColor;
    fragTexCoord = inTexCoord;
}
