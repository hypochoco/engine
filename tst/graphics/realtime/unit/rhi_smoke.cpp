#include "harness/harness.h"
//
//  rhi_smoke.cpp
//  engine::tst
//
//  Smoke test for the RHI backend: create a headless Device and a buffer via the
//  handle-addressed pool, then destroy it. Proves metal-cpp integration + the RHI Device
//  compile/link/run on this machine. No window required.
//

#include <array>
#include <cstddef>
#include <cstdio>

#include "engine/graphics/rhi/rhi.h"

TST_CASE(graphics, unit, rhi_smoke) {
    using namespace engine::rhi;

    std::printf("rhi backend: %s\n", backendName());

    Device device = Device::createHeadless({ .enableValidation = false });

    // Upload a few floats into a vertex buffer.
    const std::array<float, 6> data = {0, 1, 2, 3, 4, 5};
    const auto bytes = std::as_bytes(std::span<const float>(data));

    BufferHandle buf = device.createBuffer(
        { .size = bytes.size_bytes(), .usage = BufferUsage::Vertex, .memory = MemoryMode::CpuToGpu },
        bytes);

    if (!buf.valid()) {
        std::printf("FAIL: buffer handle invalid\n");
        TST_REQUIRE_MSG(false, "setup/verification failed");
    }
    std::printf("buffer ok: index=%u generation=%u\n", buf.index, buf.generation);

    device.destroy(buf);
    std::printf("rhi smoke ok\n");
}
