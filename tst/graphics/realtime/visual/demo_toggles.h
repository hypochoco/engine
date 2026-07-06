//
//  demo_toggles.h
//  engine::tst / graphics visual demos
//
//  Tiny edge-detected keyboard toggle for windowed demos — the quick A/B switch. Poll once per
//  frame with the GLFW window; flips its bool on the key's rising edge and prints the new state.
//  Shared by the graphics visual demos so every demo exposes its features the same way.
//

#pragma once

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <cstdio>

namespace engine::demo {

struct KeyToggle {
    int         key;
    bool        on;
    const char* label;
    bool        prev = false;

    // Returns true on the frame the state flips (so callers can (re)apply the change).
    bool poll(GLFWwindow* window) {
        const bool now  = glfwGetKey(window, key) == GLFW_PRESS;
        const bool edge = now && !prev;
        prev = now;
        if (edge) { on = !on; std::printf("[%s] %s\n", label, on ? "ON" : "OFF"); }
        return edge;
    }
};

} // namespace engine::demo
