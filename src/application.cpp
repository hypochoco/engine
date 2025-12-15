//
//  application.cpp
//  engine
//
//  Created by Daniel Cho on 9/30/25.
//

#include "engine/application.h"

#include <stdexcept>
#include <iostream>

#include "engine/graphics/model_loader.h"

// // helper for Xcode files not found
//#include <CoreFoundation/CoreFoundation.h>
//#include <filesystem>
//
//std::string getResourcePath(const std::string& name) {
//    CFBundleRef bundle = CFBundleGetMainBundle();
//    CFURLRef url = CFBundleCopyResourceURL(
//        bundle,
//        CFStringCreateWithCString(nullptr, name.c_str(), kCFStringEncodingUTF8),
//        nullptr,
//        nullptr
//    );
//
//    if (!url) {
//        throw std::runtime_error("Resource not found in bundle: " + name);
//    }
//
//    char path[PATH_MAX];
//    CFURLGetFileSystemRepresentation(url, true, (UInt8*)path, PATH_MAX);
//    CFRelease(url);
//
//    return path;
//}

Application::Application() : config(AppConfig::instance()) { // constructor
    // currently empty
}

Application::~Application() { // destructor
    // cleanup here
}

void Application::run() {
    
    graphics.initWindow();
    graphics.initVulkan(); // bad timing here ... 
    
    //    // 3d model application
    //
    //    auto textureIndex = graphics.loadTexture("viking_room.png");
    //    auto objData = ModelLoader::loadObj("viking_room.obj");
    //    for (auto& m : objData.modelMaterials) {
    //        m.textureIndex = textureIndex;
    //    }
    //    graphics.pushModel(objData);

    // digital painting application
    
    graphics.loadBrushTexture("brush.png"); // brush
    graphics.loadLayerTexture(config.paintConfig.CANVAS_WIDTH, // layer
                              config.paintConfig.CANVAS_HEIGHT);
    graphics.loadTexture(config.paintConfig.CANVAS_WIDTH, // canvas texture
                         config.paintConfig.CANVAS_HEIGHT);
    auto canvas = ModelLoader::loadCanvasQuad();
    graphics.pushModel(canvas);
    
    // finish vulkan setup
    graphics.initRender(); // probably should be selected from here on ... depending on the application ...

    // start main loop
    graphics.mainLoop();
    graphics.cleanup();
}

int main() { // simple application runner
     try {
         Application app;
         app.run();
     } catch (const std::exception& e) {
         std::cerr << e.what() << std::endl;
         return EXIT_FAILURE;
     }
     return EXIT_SUCCESS;
}
