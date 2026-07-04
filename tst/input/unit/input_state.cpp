#include "harness/harness.h"
//
//  input_state.cpp
//  engine::tst — input / unit
//
//  Headless (GLFW-free) tests for InputState edge detection + mouse/scroll deltas. Drives
//  InputState directly the way a scripted/ML input source or an event backend would. The
//  camera controller now lives in engine::controls (see controls/unit/fly_controller.cpp).
//

#include "engine/input/input.h"

using namespace engine::input;

TST_CASE(input, unit, key_edges) {
    InputState in;

    // Frame 1: press W.
    in.newFrame();
    in.setKey(Key::W, true);
    TST_REQUIRE(in.keyDown(Key::W));
    TST_REQUIRE(in.keyPressed(Key::W));       // edge: down now, up before
    TST_REQUIRE(!in.keyReleased(Key::W));

    // Frame 2: still held → no press edge.
    in.newFrame();
    in.setKey(Key::W, true);
    TST_REQUIRE(in.keyDown(Key::W));
    TST_REQUIRE(!in.keyPressed(Key::W));
    TST_REQUIRE(!in.keyReleased(Key::W));

    // Frame 3: release → release edge, one frame only.
    in.newFrame();
    in.setKey(Key::W, false);
    TST_REQUIRE(!in.keyDown(Key::W));
    TST_REQUIRE(in.keyReleased(Key::W));

    in.newFrame();
    in.setKey(Key::W, false);
    TST_REQUIRE(!in.keyReleased(Key::W));
}

TST_CASE(input, unit, mouse_button_edges) {
    InputState in;
    in.newFrame();
    in.setMouseButton(MouseButton::Right, true);
    TST_REQUIRE(in.mouseDown(MouseButton::Right));
    TST_REQUIRE(in.mousePressed(MouseButton::Right));

    in.newFrame();
    in.setMouseButton(MouseButton::Right, true);
    TST_REQUIRE(!in.mousePressed(MouseButton::Right));

    in.newFrame();
    in.setMouseButton(MouseButton::Right, false);
    TST_REQUIRE(in.mouseReleased(MouseButton::Right));
}

TST_CASE(input, unit, mouse_delta_and_scroll) {
    InputState in;

    // First frame establishes the position with no jump.
    in.newFrame();
    in.setMousePosition({100.0f, 100.0f});
    TST_APPROX(in.mouseDelta().x, 0.0f, 1e-6);
    TST_APPROX(in.mouseDelta().y, 0.0f, 1e-6);

    // Next frame: delta is relative to the previous position.
    in.newFrame();
    in.setMousePosition({110.0f, 95.0f});
    TST_APPROX(in.mouseDelta().x, 10.0f, 1e-5);
    TST_APPROX(in.mouseDelta().y, -5.0f, 1e-5);

    // Delta clears on the next frame if the cursor doesn't move.
    in.newFrame();
    in.setMousePosition({110.0f, 95.0f});
    TST_APPROX(in.mouseDelta().x, 0.0f, 1e-5);

    // Scroll accumulates within a frame and resets on newFrame.
    in.newFrame();
    in.addScroll(1.0f);
    in.addScroll(0.5f);
    TST_APPROX(in.scrollDelta(), 1.5f, 1e-6);
    in.newFrame();
    TST_APPROX(in.scrollDelta(), 0.0f, 1e-6);
}

TST_CASE(input, unit, relative_mouse_delta) {
    // Event/raw backends report relative motion; addMouseDelta accumulates it within a frame
    // and clears on newFrame — independent of the absolute setMousePosition path.
    InputState in;
    in.newFrame();
    in.addMouseDelta({3.0f, -2.0f});
    in.addMouseDelta({1.0f, 1.0f});
    TST_APPROX(in.mouseDelta().x, 4.0f, 1e-6);
    TST_APPROX(in.mouseDelta().y, -1.0f, 1e-6);

    in.newFrame();
    TST_APPROX(in.mouseDelta().x, 0.0f, 1e-6);
    TST_APPROX(in.mouseDelta().y, 0.0f, 1e-6);
}
