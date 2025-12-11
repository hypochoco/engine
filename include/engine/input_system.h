//
//  input_system.h
//  engine
//
//  Created by Daniel Cho on 12/5/25.
//

#pragma once

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

class InputSystem {
public:
    bool pressed; // mouse
    double xpos = 0;
    double ypos = 0;
    bool leftBracketPressed;
    bool rightBracketPressed;
    
    void reset();
    
    void mouse_button_callback(int button, int action, int mods);
    void mouse_position_callback(double xpos, double ypos);
    void key_callback(int key, int action);
};
