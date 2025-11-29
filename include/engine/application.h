//
//  application.h
//  engine
//
//  Created by Daniel Cho on 9/30/25.
//

#pragma once

#include "engine/graphics/graphics.h"

class Application {
    
private:
    Graphics graphics;
    
public:
    Application();
    ~Application();
    void run();
};
