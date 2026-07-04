//
//  input.h
//  engine::input
//
//  A GLFW-free input snapshot. `InputState` is plain data queried by systems/controllers; it
//  works headless or with scripted input (an ML action source can drive it just as well as a
//  window). An adapter (e.g. input_glfw::GlfwInput, or a Qt event handler) fills it each frame;
//  the queries below expose both level state (down) and edge state (pressed/released this frame).
//
//  Adapter contract (works for both polling and event-driven backends):
//    1. Call newFrame() once at the frame boundary — snapshots current→previous, clears deltas.
//    2. Feed state via the mutators below. POLLING backends (GLFW) read current state each frame
//       (setKey/setMouseButton + setMousePosition for the absolute cursor). EVENT backends (Qt)
//       call the same mutators from their callbacks (setKey on press/release, addMouseDelta for
//       relative motion, addScroll on wheel).
//  Absolute cursor motion → setMousePosition (delta derived); relative/raw motion → addMouseDelta.
//

#pragma once

#include <bitset>
#include <cstddef>
#include <cstdint>

#include <glm/vec2.hpp>

namespace engine::input {

// A compact, backend-neutral key set (extend as needed). Values are dense (0..Count) so they
// index bitsets directly; the GLFW adapter maps these to GLFW keycodes.
enum class Key : uint16_t {
    A, B, C, D, E, F, G, H, I, J, K, L, M,
    N, O, P, Q, R, S, T, U, V, W, X, Y, Z,
    Num0, Num1, Num2, Num3, Num4, Num5, Num6, Num7, Num8, Num9,
    Space, Enter, Escape, Tab, Backspace,
    LeftShift, RightShift, LeftControl, RightControl, LeftAlt, RightAlt,
    Left, Right, Up, Down,
    Count
};

enum class MouseButton : uint8_t { Left, Right, Middle, Count };

class InputState {
public:
    // --- level + edge queries ---
    bool keyDown(Key k) const     { return cur_[idx(k)]; }
    bool keyPressed(Key k) const  { return cur_[idx(k)] && !prev_[idx(k)]; }
    bool keyReleased(Key k) const { return !cur_[idx(k)] && prev_[idx(k)]; }

    bool mouseDown(MouseButton b) const     { return mcur_[idx(b)]; }
    bool mousePressed(MouseButton b) const  { return mcur_[idx(b)] && !mprev_[idx(b)]; }
    bool mouseReleased(MouseButton b) const { return !mcur_[idx(b)] && mprev_[idx(b)]; }

    glm::vec2 mousePosition() const { return mousePos_; }
    glm::vec2 mouseDelta() const    { return mouseDelta_; }   // pixels moved since last frame
    float     scrollDelta() const   { return scroll_; }       // wheel ticks this frame

    // --- adapter-facing mutation ---
    // Call once at the start of a frame: snapshot current->previous and clear per-frame deltas.
    void newFrame() {
        prev_       = cur_;
        mprev_      = mcur_;
        mouseDelta_ = glm::vec2(0.0f);
        scroll_     = 0.0f;
    }
    void setKey(Key k, bool down)                 { cur_.set(idx(k), down); }
    void setMouseButton(MouseButton b, bool down) { mcur_.set(idx(b), down); }
    void setMousePosition(glm::vec2 p) {
        if (haveMouse_) mouseDelta_ += p - mousePos_;   // accumulate within a frame
        mousePos_  = p;
        haveMouse_ = true;
    }
    void addMouseDelta(glm::vec2 d) { mouseDelta_ += d; }   // relative/raw motion (event backends)
    void addScroll(float dy) { scroll_ += dy; }

private:
    static constexpr std::size_t idx(Key k)         { return static_cast<std::size_t>(k); }
    static constexpr std::size_t idx(MouseButton b) { return static_cast<std::size_t>(b); }

    std::bitset<static_cast<std::size_t>(Key::Count)>         cur_, prev_;
    std::bitset<static_cast<std::size_t>(MouseButton::Count)> mcur_, mprev_;
    glm::vec2 mousePos_{0.0f};
    glm::vec2 mouseDelta_{0.0f};
    float     scroll_    = 0.0f;
    bool      haveMouse_ = false;   // suppress a huge first-frame delta
};

} // namespace engine::input
