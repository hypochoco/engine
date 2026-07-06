#include "harness/harness.h"
//
//  compute_smoke.cpp
//  engine::tst
//
//  RF4b step 1: RHI compute support. Dispatches a kernel that writes outBuf[i] = i*2+1 into a
//  storage buffer, then reads it back — proving createComputePipeline + the compute encoder path
//  (beginCompute/bindPipeline/bindResources/dispatch/endCompute) work headless on Metal.
//

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <span>
#include <string>
#include <vector>

#include "engine/graphics/rhi/rhi.h"

namespace {
std::vector<std::byte> readFileBin(const std::string& path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) return {};
    const auto size = static_cast<std::streamsize>(f.tellg());
    f.seekg(0);
    std::vector<std::byte> data(static_cast<size_t>(size));
    f.read(reinterpret_cast<char*>(data.data()), size);
    return data;
}
}

TST_CASE(graphics, unit, compute_smoke) {
    using namespace engine::rhi;

    Device device = Device::createHeadless({});

    const std::string metallib = std::string(ENGINE_SHADER_DIR) + "/compute_smoke.metallib";
    const auto blob = readFileBin(metallib);
    if (blob.empty()) { std::printf("FAIL: read %s\n", metallib.c_str()); TST_REQUIRE_MSG(false, "setup failed"); }

    ShaderHandle cs = device.createShader(blob, ShaderStage::Compute);
    TST_REQUIRE_MSG(cs.valid(), "compute shader load failed");
    PipelineHandle pipe = device.createComputePipeline({ .compute = cs });
    TST_REQUIRE_MSG(pipe.valid(), "compute pipeline creation failed");

    constexpr uint32_t N = 128;   // 2 threadgroups of 64
    BufferHandle out = device.createBuffer(
        { .size = N * sizeof(uint32_t), .usage = BufferUsage::Storage, .memory = MemoryMode::CpuToGpu });

    // Pre-fill with a sentinel so we can tell the kernel actually wrote.
    if (uint32_t* p = static_cast<uint32_t*>(device.map(out)))
        for (uint32_t i = 0; i < N; ++i) p[i] = 0xDEADBEEFu;

    FrameContext frame = device.beginFrame();
    CommandList cl = device.commandList(frame);
    cl.beginCompute();
    cl.bindPipeline(pipe);
    BufferBinding bb{ .binding = 0, .buffer = out };
    ResourceBindings rb; rb.buffers = std::span<const BufferBinding>(&bb, 1);
    cl.bindResources(rb);
    cl.dispatch(N / 64, 1, 1);
    cl.endCompute();
    device.submit(frame, cl);
    device.endFrame(std::move(frame));

    const uint32_t* result = static_cast<const uint32_t*>(device.map(out));
    TST_REQUIRE_MSG(result != nullptr, "map readback failed");

    bool ok = true;
    for (uint32_t i = 0; i < N; ++i) {
        if (result[i] != i * 2u + 1u) { ok = false; std::printf("mismatch at %u: got %u want %u\n", i, result[i], i * 2u + 1u); break; }
    }
    std::printf("compute wrote [0]=%u [1]=%u [127]=%u\n", result[0], result[1], result[127]);
    TST_REQUIRE_MSG(ok, "compute kernel output incorrect");

    device.destroy(out);
    std::printf("compute smoke ok\n");
}
