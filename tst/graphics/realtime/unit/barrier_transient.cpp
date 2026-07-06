#include "harness/harness.h"
//
//  barrier_transient.cpp
//  engine::tst
//
//  RF1 smoke test: the two RHI additions the render graph needs.
//   (1) A TRANSIENT depth attachment (Metal MTLStorageModeMemoryless — stays in tile memory)
//       is usable as a real render-pass depth target.
//   (2) CommandList::resourceBarrier is callable at pass boundaries (no-op on auto-tracked
//       Metal; the explicit contract Vulkan will need).
//  We clear a color target to a known color with a transient depth attached, emit barriers
//  around the pass, and read the color back — proving the frame completes correctly.
//

#include <array>
#include <cstddef>
#include <cstdio>
#include <vector>

#include "engine/graphics/rhi/rhi.h"

TST_CASE(graphics, unit, barrier_transient) {
    using namespace engine::rhi;

    Device device = Device::createHeadless({ .enableValidation = false });

    constexpr uint32_t W = 16, H = 16;

    // Color target (Shared so we can read it back).
    TextureHandle colorTex = device.createTexture(
        { .width = W, .height = H, .format = Format::RGBA8Unorm, .usage = TextureUsage::ColorTarget });
    RenderTargetHandle colorRT = device.createRenderTarget(colorTex);

    // TRANSIENT depth target -> Memoryless on Metal (never sampled, never read back).
    TextureHandle depthTex = device.createTexture(
        { .width = W, .height = H, .format = Format::Depth32Float,
          .usage = TextureUsage::DepthTarget, .transient = true });
    RenderTargetHandle depthRT = device.createRenderTarget(depthTex);

    TST_REQUIRE_MSG(colorTex.valid() && depthTex.valid(), "texture creation failed");

    FrameContext frame = device.beginFrame();
    CommandList cl = device.commandList(frame);

    // (2) barrier into RenderTarget state before the pass.
    const std::array<ResourceTransition, 1> toRT = {
        ResourceTransition{ colorTex, ResourceState::Undefined, ResourceState::RenderTarget } };
    cl.resourceBarrier(toRT);

    ColorAttachment ca;
    ca.target = colorRT;
    ca.load = LoadOp::Clear; ca.store = StoreOp::Store;
    ca.clearColor[0] = 0.25f; ca.clearColor[1] = 0.5f; ca.clearColor[2] = 0.75f; ca.clearColor[3] = 1.0f;

    DepthAttachment da;
    da.target = depthRT;
    da.load = LoadOp::Clear; da.store = StoreOp::DontCare; da.clearDepth = 1.0f;

    RenderTargetDesc rtd;
    rtd.color = std::span<const ColorAttachment>(&ca, 1);
    rtd.depth = &da;
    rtd.width = W; rtd.height = H;

    cl.beginRendering(rtd);
    cl.setViewport(0, 0, float(W), float(H));
    cl.endRendering();

    // (2) barrier out to a read/transfer state after the pass.
    const std::array<ResourceTransition, 1> toRead = {
        ResourceTransition{ colorTex, ResourceState::RenderTarget, ResourceState::TransferSrc } };
    cl.resourceBarrier(toRead);

    device.submit(frame, cl);
    device.endFrame(std::move(frame));

    std::vector<std::byte> pixels(static_cast<size_t>(W) * H * 4);
    device.readback(colorTex, pixels);

    // Center pixel should be the clear color (~64,128,191,255), tolerant of rounding.
    const size_t c = (static_cast<size_t>(H / 2) * W + (W / 2)) * 4;
    const int r = int(std::to_integer<uint8_t>(pixels[c + 0]));
    const int g = int(std::to_integer<uint8_t>(pixels[c + 1]));
    const int b = int(std::to_integer<uint8_t>(pixels[c + 2]));
    const int a = int(std::to_integer<uint8_t>(pixels[c + 3]));
    std::printf("transient-depth clear center rgba = %d %d %d %d\n", r, g, b, a);

    auto near = [](int v, int t) { return v >= t - 2 && v <= t + 2; };
    TST_REQUIRE_MSG(near(r, 64) && near(g, 128) && near(b, 191) && near(a, 255),
                    "clear color with transient depth + barriers incorrect");

    device.destroy(depthRT);
    device.destroy(depthTex);
    device.destroy(colorRT);
    device.destroy(colorTex);
    std::printf("barrier + transient depth ok\n");
}
