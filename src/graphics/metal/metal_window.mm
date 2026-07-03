//
//  metal_window.mm
//  engine::graphics / metal
//
//  Objective-C++ shim: attaches a CAMetalLayer to a GLFW window's NSView. This is the only
//  Objective-C in the engine — setting contentView.layer is an ObjC message, so it is
//  isolated here. Returns the CAMetalLayer as an opaque pointer; the C++ backend reinterprets
//  it as a metal-cpp CA::MetalLayer* (same underlying object).
//
//  Built as MRC (no ARC): we alloc/init the layer (+1) and transfer that ownership to the
//  caller. No metal-cpp here — plain ObjC frameworks only.
//

#define GLFW_EXPOSE_NATIVE_COCOA
#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>

#import <Cocoa/Cocoa.h>
#import <Metal/Metal.h>
#import <QuartzCore/CAMetalLayer.h>

#include <cstdint>

extern "C" void* engine_metal_create_layer(void* glfwWindow, void* mtlDevice,
                                           uint32_t width, uint32_t height) {
    NSWindow* nswindow = glfwGetCocoaWindow(static_cast<GLFWwindow*>(glfwWindow));
    if (!nswindow) return nullptr;

    CAMetalLayer* layer = [[CAMetalLayer alloc] init];   // owned (+1), transferred to caller
    layer.device = (id<MTLDevice>)mtlDevice;
    layer.pixelFormat = MTLPixelFormatBGRA8Unorm;
    layer.framebufferOnly = YES;
    layer.drawableSize = CGSizeMake(width, height);

    NSView* view = [nswindow contentView];
    view.layer = layer;
    view.wantsLayer = YES;

    return static_cast<void*>(layer);
}
