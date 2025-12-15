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
    leftBracketPressed = false;
    rightBracketPressed = false;
}

void InputSystem::mouse_button_callback(int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
        std::cout << "Left mouse button pressed!" << std::endl;
        pressed = true;
    }
}

void InputSystem::mouse_position_callback(double xpos, double ypos) {
    this->xpos = xpos;
    this->ypos = ypos;
}

void InputSystem::key_callback(int key, int action) {
    if (key == GLFW_KEY_LEFT_BRACKET && action == GLFW_PRESS) {
        leftBracketPressed = true;
    } else if (key == GLFW_KEY_RIGHT_BRACKET && action == GLFW_PRESS) {
        rightBracketPressed = true;
    }
}
