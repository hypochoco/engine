#include "harness/harness.h"
//
//  cluster_binning.cpp
//  engine::tst
//
//  RF4b: verifies the clustered forward+ light-binning compute pass (cluster.slang). Sets up a
//  known camera + three point lights (one in-frustum, one far to the side, one beyond zFar),
//  dispatches the binning kernel, reads back the per-cluster light lists, and checks:
//    - GPU per-cluster counts match a CPU re-implementation of the froxel AABB + sphere test
//      (proves the kernel executed correctly across all clusters/indices), and
//    - the in-frustum light is binned into >=1 cluster while the two out-of-frustum lights are
//      culled from EVERY cluster (proves the culling is meaningful).
//

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <span>
#include <string>
#include <vector>

#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "engine/graphics/rhi/rhi.h"
#include "engine/graphics/render/render_view.h"

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

// CPU mirror of cluster.slang's ClusterParams (std140/constant-buffer layout).
struct ClusterParams {
    glm::mat4  view;
    glm::vec4  frustum;   // tanHalfFovY, aspect, zNear, zFar
    glm::vec4  screen;    // W, H
    glm::uvec4 gridDim;   // CX, CY, CZ, maxLightsPerCluster
    glm::uvec4 misc;      // lightCount
};
}

TST_CASE(graphics, integration, cluster_binning) {
    using namespace engine;
    using namespace engine::rhi;

    Device device = Device::createHeadless({});

    const std::string metallib = std::string(ENGINE_SHADER_DIR) + "/cluster.metallib";
    const auto blob = readFileBin(metallib);
    if (blob.empty()) { std::printf("FAIL: read %s\n", metallib.c_str()); TST_REQUIRE_MSG(false, "setup failed"); }
    ShaderHandle cs = device.createShader(blob, ShaderStage::Compute);
    PipelineHandle pipe = device.createComputePipeline({ .compute = cs });
    TST_REQUIRE_MSG(pipe.valid(), "compute pipeline creation failed");

    // Camera at origin looking down -Z ⇒ view space == world space (simple to reason about).
    const uint32_t CX = 8, CY = 8, CZ = 8, MAXP = 16;
    const uint32_t numClusters = CX * CY * CZ;
    const float tanH = std::tan(glm::radians(60.0f) * 0.5f), aspect = 1.0f, zNear = 0.5f, zFar = 50.0f;

    ClusterParams cp;
    cp.view = glm::lookAt(glm::vec3(0), glm::vec3(0, 0, -1), glm::vec3(0, 1, 0));
    cp.frustum = { tanH, aspect, zNear, zFar };
    cp.screen  = { 512.0f, 512.0f, 0.0f, 0.0f };
    cp.gridDim = { CX, CY, CZ, MAXP };

    std::vector<render::PointLight> lights = {
        { {0.0f, 0.0f, -10.0f}, 2.0f, {1, 1, 1}, 1.0f },   // L0 in frustum, ~10 ahead
        { {100.0f, 0.0f, -10.0f}, 2.0f, {1, 1, 1}, 1.0f }, // L1 far to the side (outside frustum)
        { {0.0f, 0.0f, -100.0f}, 2.0f, {1, 1, 1}, 1.0f },  // L2 beyond zFar
    };
    cp.misc = { (uint32_t)lights.size(), 0, 0, 0 };

    BufferHandle paramsBuf = device.createBuffer(
        { .size = sizeof(ClusterParams), .usage = BufferUsage::Uniform, .memory = MemoryMode::CpuToGpu },
        std::as_bytes(std::span<const ClusterParams>(&cp, 1)));
    BufferHandle lightsBuf = device.createBuffer(
        { .size = lights.size() * sizeof(render::PointLight), .usage = BufferUsage::Storage, .memory = MemoryMode::CpuToGpu },
        std::as_bytes(std::span<const render::PointLight>(lights)));
    BufferHandle gridBuf = device.createBuffer(
        { .size = numClusters * sizeof(uint32_t), .usage = BufferUsage::Storage, .memory = MemoryMode::CpuToGpu });
    BufferHandle idxBuf = device.createBuffer(
        { .size = numClusters * MAXP * sizeof(uint32_t), .usage = BufferUsage::Storage, .memory = MemoryMode::CpuToGpu });

    FrameContext frame = device.beginFrame();
    CommandList cl = device.commandList(frame);
    cl.beginCompute();
    cl.bindPipeline(pipe);
    BufferBinding bb[4] = {
        { .binding = 0, .buffer = paramsBuf }, { .binding = 1, .buffer = lightsBuf },
        { .binding = 2, .buffer = gridBuf },   { .binding = 3, .buffer = idxBuf },
    };
    ResourceBindings rb; rb.buffers = std::span<const BufferBinding>(bb, 4);
    cl.bindResources(rb);
    cl.dispatch((numClusters + 63) / 64, 1, 1);
    cl.endCompute();
    device.submit(frame, cl);
    device.endFrame(std::move(frame));

    const uint32_t* grid = static_cast<const uint32_t*>(device.map(gridBuf));
    const uint32_t* idx  = static_cast<const uint32_t*>(device.map(idxBuf));
    TST_REQUIRE_MSG(grid && idx, "readback failed");

    // CPU reference: the exact froxel AABB + sphere test the shader does.
    auto sliceDist = [&](uint32_t s) { return zNear + (zFar - zNear) * (float(s) / float(CZ)); };
    auto cpuCount = [&](uint32_t cluster) {
        uint32_t i = cluster % CX, j = (cluster / CX) % CY, k = cluster / (CX * CY);
        float nx0 = -1 + 2.0f * i / CX, nx1 = -1 + 2.0f * (i + 1) / CX;
        float ny0 = -1 + 2.0f * j / CY, ny1 = -1 + 2.0f * (j + 1) / CY;
        float d0 = sliceDist(k), d1 = sliceDist(k + 1), kx = tanH * aspect, ky = tanH;
        float xs[4] = { nx0 * d0 * kx, nx1 * d0 * kx, nx0 * d1 * kx, nx1 * d1 * kx };
        float ys[4] = { ny0 * d0 * ky, ny1 * d0 * ky, ny0 * d1 * ky, ny1 * d1 * ky };
        glm::vec3 mn(std::min({xs[0],xs[1],xs[2],xs[3]}), std::min({ys[0],ys[1],ys[2],ys[3]}), -d1);
        glm::vec3 mx(std::max({xs[0],xs[1],xs[2],xs[3]}), std::max({ys[0],ys[1],ys[2],ys[3]}), -d0);
        uint32_t c = 0;
        for (auto& pl : lights) {
            glm::vec3 Lv = glm::vec3(cp.view * glm::vec4(pl.position, 1.0f));
            glm::vec3 cl2 = glm::clamp(Lv, mn, mx), dd = cl2 - Lv;
            if (glm::dot(dd, dd) <= pl.radius * pl.radius && c < MAXP) ++c;
        }
        return c;
    };

    uint32_t mismatches = 0, totalBinned = 0, l0clusters = 0;
    bool sawL1 = false, sawL2 = false;
    for (uint32_t c = 0; c < numClusters; ++c) {
        if (grid[c] != cpuCount(c)) { if (mismatches < 5) std::printf("cluster %u: gpu=%u cpu=%u\n", c, grid[c], cpuCount(c)); ++mismatches; }
        totalBinned += grid[c];
        for (uint32_t s = 0; s < grid[c]; ++s) {
            uint32_t li = idx[c * MAXP + s];
            if (li == 0) ++l0clusters;
            if (li == 1) sawL1 = true;
            if (li == 2) sawL2 = true;
        }
    }
    std::printf("clusters=%u totalBinned=%u L0-clusters=%u sawL1=%d sawL2=%d mismatches=%u\n",
                numClusters, totalBinned, l0clusters, (int)sawL1, (int)sawL2, mismatches);

    TST_REQUIRE_MSG(mismatches == 0, "GPU binning disagrees with CPU reference");
    TST_REQUIRE_MSG(l0clusters > 0, "in-frustum light L0 was not binned into any cluster");
    TST_REQUIRE_MSG(!sawL1 && !sawL2, "out-of-frustum lights were not culled");

    device.destroy(paramsBuf); device.destroy(lightsBuf); device.destroy(gridBuf); device.destroy(idxBuf);
    std::printf("cluster binning ok\n");
}
