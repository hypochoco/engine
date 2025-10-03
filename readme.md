
## Developer Notes

General build
- `./scripts/compile.sh` to compile
- `./scripts/compile_shaders.sh` to compile shaders
- `./scripts/run.sh` to run

For Xcode, run `cmake -G Xcode ..` in the build directory and open Xcode project file.
- For macOS, turn on macOS flag / bool in graphics (look into putting this in a config somewhere...)

## High Level Goals
- Graphics engine (additional shader code, compute shaders?)
- Physics engine

## Considerations
- eigen over glm ?

## Todo
- [ ] Restructure vulkan code
    - [x] Add a header class 
    - [ ] Allow additional arbitrary models / textures
    - [ ] Temp class or config with file paths ... 

- [ ] Address these vulkan issues:

    > It should be noted that in a real world application, you're not supposed to actually call vkAllocateMemory for every individual buffer. The maximum number of simultaneous memory allocations is limited by the maxMemoryAllocationCount physical device limit, which may be as low as 4096 even on high end hardware like an NVIDIA GTX 1080. The right way to allocate memory for a large number of objects at the same time is to create a custom allocator that splits up a single allocation among many different objects by using the offset parameters that we've seen in many functions.
        - You can either implement such an allocator yourself, or use the VulkanMemoryAllocator library provided by the GPUOpen initiative. However, for this tutorial it's okay to use a separate allocation for every resource, because we won't come close to hitting any of these limits for now.

    > Using a UBO this way is not the most efficient way to pass frequently changing values to the shader. A more efficient way to pass a small buffer of data to shaders are push constants.

    > Run your program now with optimization enabled (e.g. Release mode in Visual Studio and with the -O3 compiler flag for GCC`). This is necessary, because otherwise loading the model will be very slow. 


## Notes on project structure

2. Folder Structure for Libraries
project/
├─ include/
│  └─ project/
│      ├─ graphics/
│      │   └─ Renderer.hpp
│      └─ physics/
│          └─ PhysicsEngine.hpp
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

Each library gets its own CMakeLists.txt inside its folder.
The top-level CMakeLists.txt orchestrates everything.

3. CMake Example
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

4. How This Works
Static libraries (STATIC) compile code into .a or .lib files.
Public include directories (target_include_directories(... PUBLIC ../include)) allow other libraries or the main executable to include headers like:
`#include "project/graphics/Renderer.hpp"`
`#include "project/physics/PhysicsEngine.hpp"`

Dependencies between libraries: If PhysicsEngine used Renderer, you’d do:
target_link_libraries(physics PUBLIC graphics)
Then any target linking physics automatically gets graphics too.

✅ Benefits
Faster incremental builds: changing Renderer.cpp only rebuilds the graphics library, not everything.
Clear API boundaries: only headers in include/project/... are public.
Modular testing: you can write unit tests per library.
