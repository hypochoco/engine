//
//  glfw_input.cpp
//  engine::input_glfw
//
//  GLFW-backed implementation of the input adapter. Keys and mouse buttons are polled by
//  level (glfwGetKey / glfwGetMouseButton — GLFW maintains current state); the cursor position
//  is polled directly; scroll is event-based, so a per-window callback accumulates wheel ticks
//  which update() drains once per frame.
//

#include "engine/input_glfw/glfw_input.h"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <unordered_map>

namespace engine::input {
namespace {

// Map a window to the float that its scroll callback should accumulate into. Input is
// main-thread-only (a GLFW requirement), so a plain map needs no synchronization. Using this
// instead of the window user-pointer avoids clobbering anyone else's use of it.
std::unordered_map<GLFWwindow*, float*>& scrollTargets() {
    static std::unordered_map<GLFWwindow*, float*> m;
    return m;
}

void scrollCallback(GLFWwindow* w, double /*xoff*/, double yoff) {
    auto& m = scrollTargets();
    if (auto it = m.find(w); it != m.end()) *it->second += static_cast<float>(yoff);
}

int toGlfwKey(Key k) {
    switch (k) {
        case Key::A: return GLFW_KEY_A; case Key::B: return GLFW_KEY_B;
        case Key::C: return GLFW_KEY_C; case Key::D: return GLFW_KEY_D;
        case Key::E: return GLFW_KEY_E; case Key::F: return GLFW_KEY_F;
        case Key::G: return GLFW_KEY_G; case Key::H: return GLFW_KEY_H;
        case Key::I: return GLFW_KEY_I; case Key::J: return GLFW_KEY_J;
        case Key::K: return GLFW_KEY_K; case Key::L: return GLFW_KEY_L;
        case Key::M: return GLFW_KEY_M; case Key::N: return GLFW_KEY_N;
        case Key::O: return GLFW_KEY_O; case Key::P: return GLFW_KEY_P;
        case Key::Q: return GLFW_KEY_Q; case Key::R: return GLFW_KEY_R;
        case Key::S: return GLFW_KEY_S; case Key::T: return GLFW_KEY_T;
        case Key::U: return GLFW_KEY_U; case Key::V: return GLFW_KEY_V;
        case Key::W: return GLFW_KEY_W; case Key::X: return GLFW_KEY_X;
        case Key::Y: return GLFW_KEY_Y; case Key::Z: return GLFW_KEY_Z;
        case Key::Num0: return GLFW_KEY_0; case Key::Num1: return GLFW_KEY_1;
        case Key::Num2: return GLFW_KEY_2; case Key::Num3: return GLFW_KEY_3;
        case Key::Num4: return GLFW_KEY_4; case Key::Num5: return GLFW_KEY_5;
        case Key::Num6: return GLFW_KEY_6; case Key::Num7: return GLFW_KEY_7;
        case Key::Num8: return GLFW_KEY_8; case Key::Num9: return GLFW_KEY_9;
        case Key::Space:        return GLFW_KEY_SPACE;
        case Key::Enter:        return GLFW_KEY_ENTER;
        case Key::Escape:       return GLFW_KEY_ESCAPE;
        case Key::Tab:          return GLFW_KEY_TAB;
        case Key::Backspace:    return GLFW_KEY_BACKSPACE;
        case Key::LeftShift:    return GLFW_KEY_LEFT_SHIFT;
        case Key::RightShift:   return GLFW_KEY_RIGHT_SHIFT;
        case Key::LeftControl:  return GLFW_KEY_LEFT_CONTROL;
        case Key::RightControl: return GLFW_KEY_RIGHT_CONTROL;
        case Key::LeftAlt:      return GLFW_KEY_LEFT_ALT;
        case Key::RightAlt:     return GLFW_KEY_RIGHT_ALT;
        case Key::Left:         return GLFW_KEY_LEFT;
        case Key::Right:        return GLFW_KEY_RIGHT;
        case Key::Up:           return GLFW_KEY_UP;
        case Key::Down:         return GLFW_KEY_DOWN;
        case Key::Count:        break;
    }
    return GLFW_KEY_UNKNOWN;
}

int toGlfwButton(MouseButton b) {
    switch (b) {
        case MouseButton::Left:   return GLFW_MOUSE_BUTTON_LEFT;
        case MouseButton::Right:  return GLFW_MOUSE_BUTTON_RIGHT;
        case MouseButton::Middle: return GLFW_MOUSE_BUTTON_MIDDLE;
        case MouseButton::Count:  break;
    }
    return -1;
}

} // namespace

GlfwInput::GlfwInput(void* glfwWindow) : window_(glfwWindow) {
    auto* w = static_cast<GLFWwindow*>(window_);
    if (!w) return;
    scrollTargets()[w] = &pendingScroll_;
    glfwSetScrollCallback(w, scrollCallback);
}

GlfwInput::~GlfwInput() {
    auto* w = static_cast<GLFWwindow*>(window_);
    if (!w) return;
    scrollTargets().erase(w);
    // The window is typically destroyed right after this; don't touch its callbacks.
}

void GlfwInput::update(InputState& state) {
    auto* w = static_cast<GLFWwindow*>(window_);
    if (!w) return;

    state.newFrame();

    for (std::size_t i = 0; i < static_cast<std::size_t>(Key::Count); ++i) {
        const auto k  = static_cast<Key>(i);
        const int  gk = toGlfwKey(k);
        if (gk != GLFW_KEY_UNKNOWN) state.setKey(k, glfwGetKey(w, gk) == GLFW_PRESS);
    }
    for (std::size_t i = 0; i < static_cast<std::size_t>(MouseButton::Count); ++i) {
        const auto b  = static_cast<MouseButton>(i);
        const int  gb = toGlfwButton(b);
        if (gb >= 0) state.setMouseButton(b, glfwGetMouseButton(w, gb) == GLFW_PRESS);
    }

    double mx = 0.0, my = 0.0;
    glfwGetCursorPos(w, &mx, &my);
    state.setMousePosition(glm::vec2(static_cast<float>(mx), static_cast<float>(my)));

    state.addScroll(pendingScroll_);
    pendingScroll_ = 0.0f;
}

void GlfwInput::setCursorCaptured(bool captured) {
    captured_ = captured;
    auto* w = static_cast<GLFWwindow*>(window_);
    if (!w) return;
    glfwSetInputMode(w, GLFW_CURSOR, captured ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);
}

} // namespace engine::input
