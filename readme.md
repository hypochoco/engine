
## Development Notes

**General build:**
- `./scripts/compile.sh` to compile (not meant for Xcode)
- `./scripts/compile_shaders.sh` to compile shaders to resources dir
- `./scripts/run.sh` to run program

For Xcode, run `cmake -G Xcode ..` in the build directory and open Xcode project file.
- For macOS, set macOS bool in `GraphicsConfig` in `config.h`

Xcode specific instructions:
- In `Build Phases`, add compiled files into `Compile Sources`, and then shaders, textures, models, etc. into a `Copy Bundle Resource`
- In `Build Settings`, add dir paths to `Header Search Paths`
    - Note `stb` may have to be set as recursive, unsure
    
## High Level Goals
- Graphics engine
- Physics engine

## Todo
- [ ] Address these vulkan issues:
    - [ ] Run your program now with optimization enabled (e.g. Release mode in Visual Studio and with the -O3 compiler flag for GCC`). This is necessary, because otherwise loading the model will be very slow. 
    
- [ ] layer into canvas ... 
    - [ ] layer visibility 

    
    
- [ ] mouse down vs pressed on painting (brush functions / shaders?)
    - [ ] configure brush scissor for optimization
- [ ] fix the semaphore issue ... 
- [ ] polishing into an app
    - [ ] image sizes, tools select and transform ... 
- [ ] grinding out performance 
- [ ] finally getting some better structure 
- [ ] most of the vulkan is there, just need to clean up a little ... 
- [ ] restructure for layers, canvas, etc. what is a good way to have structure here ... 
    - different classes for digital painting stuff ?? swap chain stuff is pretty similar ... 
    - different classes for different pipelines ?? 
    - `graphics_paint` maybe a better name ... something that establishes the main pipeline into the swapchain, then something else ... 
- [ ] clean up this readme a little ... 



---

## Notes

### Graphics API


graphics api that can support both a game engine and digital painting program
- select render pipelines
- select draw commands
- select select render passes
- etc.

- then use the graphics obj 
- any concerns about multithreading in this option ?? 
    - inner multi threading?

worker stealing
- each have a queue, owner pushes and pops from one end
- theives take from the other side of the queue
- keep everything lock free using an atomic queue or something ... 

debug features like bounding boxes ... 

consider parallel processes and a job system ... 

internally thread physics as well ... 

collection of workers, doing all the jobs, maybe dedicate stuff instead??
    be able to support either method
    multiple workers for different parts of physics ?? 
    
    dependency graph on tasks, even break into sub tasks?
    
    workers (optimize cache efficiency)
        workers have a local queue of similar tasks, steal from each other if idle ... 
        it matters what tasks there are (which frame, etc.)
    










add and remove lights

change lights (colors, position, brightness, etc.)
    types of lights, directional, point, etc.
    
render passes? turning this on or off?
    
    
adding and removing models
    load model
        custom material definitions ...
        return model id (position in vector)
    remove model
        input model id (vector position)
        
same model, but different materials?

    
adding and removing instances
    add instance
        input model id
        return instance id (vector position? will depend on internal representation)
    remove instance
        input instance id
        
change instance model matrices
    transform (rotate, scale, translate)
        input instance id
        
getters (get all instances of a model, etc.)

hierarchical instancing?

cameras 




### Debugging Memory leaks

1. Compile your program with debugging symbols: `g++ -g -o myprogram myprogram.cpp`
2. Run the program under Valgrind: `valgrind --leak-check=full --show-leak-kinds=all ./myprogram`

Analyze the output:
```
definitely lost → memory that was allocated but never freed.
indirectly lost → memory lost because something else pointing to it was lost.
still reachable → memory that wasn’t freed but is still accessible at exit (usually less serious).
```
Valgrind also reports invalid reads/writes, use-after-free, and uninitialized memory usage.

### Project structure

**Folder structure for libraries**
```
project/
├─ include/
│  └─ project/
│      ├─ graphics/
│      │   └─ Renderer.h
│      └─ physics/
│          └─ PhysicsEngine.h
├─ src/
│  ├─ graphics/
│  │   └─ Renderer.cpp
│  └─ physics/
│      └─ PhysicsEngine.cpp
├─ main.cpp
├─ CMakeLists.txt
├─ graphics/
│  └─ CMakeLists.txt
└─ physics/
   └─ CMakeLists.txt
```

Each library gets its own CMakeLists.txt inside its folder.
The top-level CMakeLists.txt orchestrates everything.

**CMake example**
```
Top-level CMakeLists.txt
cmake_minimum_required(VERSION 3.25)
project(MyProject VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(graphics)
add_subdirectory(physics)

add_executable(my_project main.cpp)
target_link_libraries(my_project PRIVATE graphics physics)
target_include_directories(my_project PRIVATE include)
Graphics CMakeLists.txt
add_library(graphics STATIC
    ../src/graphics/Renderer.cpp
)

target_include_directories(graphics PUBLIC ../include)
Physics CMakeLists.txt
add_library(physics STATIC
    ../src/physics/PhysicsEngine.cpp
)

target_include_directories(physics PUBLIC ../include)
```

**Explanation**
Static libraries (STATIC) compile code into .a or .lib files.
Public include directories (target_include_directories(... PUBLIC ../include)) allow other libraries or the main executable to include headers like:
`#include "project/graphics/Renderer.h"`
`#include "project/physics/PhysicsEngine.h"`

Dependencies between libraries: If PhysicsEngine used Renderer, you’d do:
target_link_libraries(physics PUBLIC graphics)
Then any target linking physics automatically gets graphics too.

### Deferred deletion

```
    // pseudocode for deferred deletion
        //struct PendingDeletion {
        //    VkBuffer buffer;
        //    VkDeviceMemory memory;
        //    VkFence fence;
        //};
        //
        //std::vector<PendingDeletion> deletions;
        //
        // // Each frame:
        //for (auto it = deletions.begin(); it != deletions.end(); ) {
        //    if (vkGetFenceStatus(device, it->fence) == VK_SUCCESS) {
        //        vkDestroyBuffer(device, it->buffer, nullptr);
        //        vkFreeMemory(device, it->memory, nullptr);
        //        it = deletions.erase(it);
        //    } else {
        //        ++it;
        //    }
        //}
```
