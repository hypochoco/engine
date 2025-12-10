//
//  input_system.cpp
//  engine
//
//  Created by Daniel Cho on 12/5/25.
//

#include "engine/input_system.h"

#include <iostream>

void InputSystem::reset() {
    pressed = false;
}

void InputSystem::mouse_button_callback(int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
        std::cout << "Left mouse button pressed!" << std::endl;
        pressed = true;
    } else {
        pressed = false; // doesn't actually get called
    }
}

void InputSystem::mouse_position_callback(double xpos, double ypos) {
    this->xpos = xpos;
    this->ypos = ypos;
}
