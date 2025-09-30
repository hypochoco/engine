
## dev
`./scripts/compile.sh` to compile
`./scripts/compile_shaders.sh` to compile shaders
`./scripts/run.sh` to run

## engine goals
- graphics
- physics 

### considerations
- mac optimizations?
    - opencl?, eigen
    - metal shader language

### TODO

Fix this eventually ... 
- It should be noted that in a real world application, you're not supposed to actually call vkAllocateMemory for every individual buffer. The maximum number of simultaneous memory allocations is limited by the maxMemoryAllocationCount physical device limit, which may be as low as 4096 even on high end hardware like an NVIDIA GTX 1080. The right way to allocate memory for a large number of objects at the same time is to create a custom allocator that splits up a single allocation among many different objects by using the offset parameters that we've seen in many functions.
    - You can either implement such an allocator yourself, or use the VulkanMemoryAllocator library provided by the GPUOpen initiative. However, for this tutorial it's okay to use a separate allocation for every resource, because we won't come close to hitting any of these limits for now.

- Using a UBO this way is not the most efficient way to pass frequently changing values to the shader. A more efficient way to pass a small buffer of data to shaders are push constants. We may look at these in a future chapter.

- Run your program now with optimization enabled (e.g. Release mode in Visual Studio and with the -O3 compiler flag for GCC`). This is necessary, because otherwise loading the model will be very slow. You should see something like the following:

### notes
- unit tests ... 


- consider project directories and such...
    - resources, libs, src, test, etc.