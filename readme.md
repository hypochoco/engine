
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
    - [ ] Run your program now with optimization enabled (e.g. Release mode with the -O3 compiler flag for GCC`)
    
- [ ] Generalize vulkan features

    - [ ] Organize one time vs per draw ... 
        - [ ] Establish place for bulk calls 

    - [ ] Automatic image layout transitions?
    - [ ] Rendering feature selection?
    - [ ] Fork and then develop separately, give general utils and then allow development
    - [ ] Onboard onto QT
 
- [ ] Application level
    - [ ] Layer visibility 
    - [ ] Configure scissors
    - [ ] Brush functions / mouse down and released
    
    - [ ] Application resizing
    - [ ] Cut tool + transforms
    
---

## Notes

### Graphics API

Multi threading:
- Worker stealing rather than a large queue
- Better cache efficiency
- Tasks have a dependency graph

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
