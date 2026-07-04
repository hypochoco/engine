//
//  glfw_input.h
//  engine::input_glfw
//
//  GLFW adapter that fills an engine::input::InputState from a GLFW window each frame. Kept in
//  a separate module (engine::input_glfw) so the neutral engine::input stays GLFW-free — a
//  future engine::input_qt adapter would sit alongside this. The window is passed as an opaque
//  `void*` (a GLFWwindow*) so this header stays GLFW-free too.
//

#pragma once

#include "engine/input/input.h"

namespace engine::input {

class GlfwInput {
public:
    // `glfwWindow` is a GLFWwindow* (cast to void*). Installs a scroll callback on it.
    explicit GlfwInput(void* glfwWindow);
    ~GlfwInput();
    GlfwInput(const GlfwInput&)            = delete;
    GlfwInput& operator=(const GlfwInput&) = delete;

    // Snapshot the window's current keyboard/mouse state into `state`. Call once per frame,
    // AFTER glfwPollEvents() (so this frame's scroll events have been delivered).
    void update(InputState& state);

    // Capture the cursor (hidden + locked) for mouse-look, or release it. When captured GLFW
    // reports raw unbounded motion via mouseDelta.
    void setCursorCaptured(bool captured);
    bool cursorCaptured() const { return captured_; }

private:
    void* window_        = nullptr;
    bool  captured_      = false;
    float pendingScroll_ = 0.0f;   // accumulated by the scroll callback between updates
};

} // namespace engine::input
