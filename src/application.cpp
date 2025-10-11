//
//  application.cpp
//  engine
//
//  Created by Daniel Cho on 9/30/25.
//

#include "engine/application.h"

#include <stdexcept>
#include <iostream>

Application::Application() { // constructor
    graphics = Graphics();
}

Application::~Application() { // destructor
    // cleanup here
}

void Application::run() {
    std::cout << "hello world!" << std::endl;
    graphics.run();
}

int main() { // simple application runner
     Application app;
     try {
         app.run();
     } catch (const std::exception& e) {
         std::cerr << e.what() << std::endl;
         return EXIT_FAILURE;
     }
     return EXIT_SUCCESS;
}
