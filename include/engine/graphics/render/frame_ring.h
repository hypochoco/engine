//
//  frame_ring.h
//  engine::graphics / render
//
//  Per-frame-in-flight bump allocator over CpuToGpu buffers, and the fix for two latent
//  renderer hazards (see notes/investigations/2026-07-04-render-framework-plan.md §1):
//
//   1. PER-FRAME HAZARD — a single upload buffer overwritten every frame is read-write raced
//      when framesInFlight > 1. Each frame writes into its OWN arena (indexed by
//      FrameContext::frameIndex()); combined with the Device's frames-in-flight throttle, a
//      frame in flight is never overwritten by the next.
//   2. MULTI-VIEW CLOBBER — reusing one buffer across views in a frame makes every view read
//      the last view's data (commands execute later). Each alloc() returns a distinct
//      {buffer, offset} region, so views/passes never clobber each other.
//
//  Header-only (no new backend TU / CMake reglob). Owns its GPU buffers via the Device.
//

#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <vector>

#include "engine/graphics/rhi/device.h"

namespace engine::render {

class FrameRingAllocator {
public:
    struct Alloc {
        rhi::BufferHandle buffer{};
        uint64_t          offset = 0;
        void*             ptr    = nullptr;   // CPU write pointer into the mapped arena
    };

    FrameRingAllocator() = default;
    FrameRingAllocator(rhi::Device& device, uint32_t framesInFlight,
                       uint64_t initialBytes = 64 * 1024,
                       rhi::BufferUsage usage = rhi::BufferUsage::Uniform | rhi::BufferUsage::Storage)
        : device_(&device), usage_(usage), initialBytes_(std::max<uint64_t>(initialBytes, 256)) {
        arenas_.resize(std::max(framesInFlight, 1u));
    }

    FrameRingAllocator(FrameRingAllocator&&) noexcept = default;
    FrameRingAllocator& operator=(FrameRingAllocator&&) noexcept = default;
    FrameRingAllocator(const FrameRingAllocator&) = delete;
    FrameRingAllocator& operator=(const FrameRingAllocator&) = delete;

    // Select + reset the arena for this frame index. Frees buffers retired the last time this
    // arena was used (safe: framesInFlight frames have since completed on the GPU).
    void beginFrame(uint32_t frameIndex) {
        current_ = frameIndex % static_cast<uint32_t>(arenas_.size());
        Arena& a = arenas_[current_];
        for (auto h : a.retired) device_->destroy(h);
        a.retired.clear();
        a.offset = 0;
        if (!a.buffer.valid()) grow(a, initialBytes_);
    }

    // Bump-allocate `bytes` (aligned) from the current frame's arena and return the region plus
    // a CPU write pointer. Grows if needed WITHOUT invalidating regions already handed out this
    // frame: the full buffer is retired (still alive until this arena is reused) and a bigger one
    // takes over — earlier allocs keep their own {buffer, offset}, only new allocs use the new
    // buffer. So a caller must use the handle returned by THIS alloc for THIS region.
    Alloc alloc(uint64_t bytes, uint64_t align = 256) {
        Arena& a = arenas_[current_];
        uint64_t base = (a.offset + (align - 1)) & ~(align - 1);
        if (base + bytes > a.capacity) {
            uint64_t need = base + bytes;
            uint64_t newCap = a.capacity ? a.capacity : initialBytes_;
            while (newCap < need) newCap *= 2;
            if (a.buffer.valid()) a.retired.push_back(a.buffer);
            grow(a, newCap);
            base = 0;
        }
        a.offset = base + bytes;
        Alloc out;
        out.buffer = a.buffer;
        out.offset = base;
        out.ptr    = static_cast<uint8_t*>(device_->map(a.buffer)) + base;
        return out;
    }

    // Convenience: allocate + copy a trivially-copyable range in one call.
    template <class T>
    Alloc upload(const T* data, size_t count, uint64_t align = 256) {
        Alloc a = alloc(static_cast<uint64_t>(sizeof(T)) * count, align);
        if (a.ptr && count) std::memcpy(a.ptr, data, sizeof(T) * count);
        return a;
    }

    void destroy() {
        if (!device_) return;
        for (auto& a : arenas_) {
            for (auto h : a.retired) device_->destroy(h);
            a.retired.clear();
            if (a.buffer.valid()) device_->destroy(a.buffer);
            a.buffer = {};
            a.capacity = 0;
        }
    }

private:
    struct Arena {
        rhi::BufferHandle              buffer{};
        uint64_t                       capacity = 0;
        uint64_t                       offset   = 0;
        std::vector<rhi::BufferHandle> retired;
    };

    void grow(Arena& a, uint64_t cap) {
        a.buffer = device_->createBuffer(
            { .size = cap, .usage = usage_, .memory = rhi::MemoryMode::CpuToGpu });
        a.capacity = cap;
    }

    rhi::Device*       device_ = nullptr;
    rhi::BufferUsage   usage_  = rhi::BufferUsage::Uniform;
    uint64_t           initialBytes_ = 64 * 1024;
    std::vector<Arena> arenas_;
    uint32_t           current_ = 0;
};

} // namespace engine::render
