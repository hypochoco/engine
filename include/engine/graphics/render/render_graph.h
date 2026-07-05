//
//  render_graph.h
//  engine::graphics / render
//
//  Minimal, explicit, correctness-first render graph (RF3 of the render-framework plan,
//  notes/investigations/2026-07-04-render-framework-plan.md §4). Passes are registered with
//  their attachments and the textures they READ; the graph executes them in registration order
//  and emits resource-state barriers at pass boundaries from those declarations.
//
//  Intentionally NOT included in v1 (deferred optimizations, per the owner-approved scope):
//   - automatic pass reordering (registration order IS execution order — the caller lists
//     producers before consumers, which is the natural way to build the frame),
//   - transient memory ALIASING / pass culling.
//  These are pure optimizations that can be added later without changing this API.
//
//  Barriers: derived from declared reads (→ ShaderRead) and the pass's writes (→ RenderTarget
//  for raster color/depth, StorageRW for compute). On Metal these are near-no-ops (the driver
//  auto-tracks hazards); on Vulkan they will become real image-layout transitions + pipeline
//  barriers. Keeping the declarations explicit here is what makes the Vulkan backend correct by
//  construction. Header-only (no new backend TU / CMake reglob).
//

#pragma once

#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>

#include "engine/graphics/rhi/device.h"
#include "engine/graphics/rhi/command_list.h"

namespace engine::render {

class RenderGraph {
public:
    // A color attachment for a raster pass.
    struct ColorTarget {
        rhi::RenderTargetHandle rt;
        rhi::TextureHandle      tex;          // backing texture (for barrier state tracking)
        float                   clear[4] = {0, 0, 0, 1};
        rhi::LoadOp             load  = rhi::LoadOp::Clear;
        rhi::StoreOp            store = rhi::StoreOp::Store;
    };
    // Optional depth attachment for a raster pass (`used=false` ⇒ no depth).
    struct DepthTarget {
        rhi::RenderTargetHandle rt;
        rhi::TextureHandle      tex;
        float                   clearDepth = 1.0f;
        rhi::LoadOp             load  = rhi::LoadOp::Clear;
        rhi::StoreOp            store = rhi::StoreOp::DontCare;
        bool                    used  = false;
    };

    using RecordFn = std::function<void(rhi::CommandList&)>;

    explicit RenderGraph(rhi::Device& device) : device_(&device) {}

    // A raster pass: the graph binds `color` (+ optional `depth`), sets a full-target viewport,
    // then invokes `record` to issue draws (record must NOT call begin/endRendering). `reads`
    // are textures this pass samples (transitioned to ShaderRead before the pass).
    void addRasterPass(const char* name, ColorTarget color, DepthTarget depth,
                       uint32_t width, uint32_t height,
                       std::vector<rhi::TextureHandle> reads, RecordFn record) {
        Pass p;
        p.name = name; p.kind = Kind::Raster;
        p.color = color; p.depth = depth; p.width = width; p.height = height;
        p.reads = std::move(reads); p.record = std::move(record);
        passes_.push_back(std::move(p));
    }

    // A compute pass: the graph emits barriers for `reads` (→ ShaderRead) and `writes`
    // (→ StorageRW) then invokes `record` (which issues dispatch()es).
    void addComputePass(const char* name,
                        std::vector<rhi::TextureHandle> reads,
                        std::vector<rhi::TextureHandle> writes, RecordFn record) {
        Pass p;
        p.name = name; p.kind = Kind::Compute;
        p.reads = std::move(reads); p.writes = std::move(writes); p.record = std::move(record);
        passes_.push_back(std::move(p));
    }

    // Record all passes in registration order into the frame's command list, emitting barriers
    // at each pass boundary, then submit.
    void execute(rhi::FrameContext& frame) {
        rhi::CommandList cl = device_->commandList(frame);
        std::vector<rhi::ResourceTransition> transitions;

        for (const auto& p : passes_) {
            transitions.clear();
            // reads → ShaderRead
            for (auto t : p.reads) transitionTo(transitions, t, rhi::ResourceState::ShaderRead);
            // writes → target state
            if (p.kind == Kind::Raster) {
                transitionTo(transitions, p.color.tex, rhi::ResourceState::RenderTarget);
                if (p.depth.used) transitionTo(transitions, p.depth.tex, rhi::ResourceState::RenderTarget);
            } else {
                for (auto t : p.writes) transitionTo(transitions, t, rhi::ResourceState::StorageRW);
            }
            if (!transitions.empty()) cl.resourceBarrier(transitions);

            if (p.kind == Kind::Raster) {
                rhi::ColorAttachment ca;
                ca.target = p.color.rt;
                ca.load = p.color.load; ca.store = p.color.store;
                for (int i = 0; i < 4; ++i) ca.clearColor[i] = p.color.clear[i];

                rhi::DepthAttachment da;
                if (p.depth.used) {
                    da.target = p.depth.rt;
                    da.load = p.depth.load; da.store = p.depth.store;
                    da.clearDepth = p.depth.clearDepth;
                }

                rhi::RenderTargetDesc rtd;
                // Depth-only pass (e.g. shadow map): no color attachment.
                const bool hasColor = p.color.rt.valid();
                rtd.color = hasColor ? std::span<const rhi::ColorAttachment>(&ca, 1)
                                     : std::span<const rhi::ColorAttachment>();
                rtd.depth = p.depth.used ? &da : nullptr;
                rtd.width = p.width; rtd.height = p.height;

                cl.beginRendering(rtd);
                cl.setViewport(0, 0, float(p.width), float(p.height));
                if (p.record) p.record(cl);
                cl.endRendering();
            } else {
                cl.beginCompute();
                if (p.record) p.record(cl);
                cl.endCompute();
            }
        }
        device_->submit(frame, cl);
    }

    void reset() { passes_.clear(); state_.clear(); }
    size_t passCount() const { return passes_.size(); }

private:
    enum class Kind { Raster, Compute };
    struct Pass {
        const char*                     name = "";
        Kind                            kind = Kind::Raster;
        ColorTarget                     color{};
        DepthTarget                     depth{};
        uint32_t                        width = 0, height = 0;
        std::vector<rhi::TextureHandle> reads;
        std::vector<rhi::TextureHandle> writes;
        RecordFn                        record;
    };

    void transitionTo(std::vector<rhi::ResourceTransition>& out, rhi::TextureHandle tex,
                      rhi::ResourceState to) {
        if (!tex.valid()) return;
        rhi::ResourceState from = rhi::ResourceState::Undefined;
        auto it = state_.find(tex.index);
        if (it != state_.end()) from = it->second;
        if (from == to) return;   // already in the right state — no barrier needed
        out.push_back({ tex, from, to });
        state_[tex.index] = to;
    }

    rhi::Device*                                  device_ = nullptr;
    std::vector<Pass>                             passes_;
    std::unordered_map<uint32_t, rhi::ResourceState> state_;   // texture index → last known state
};

} // namespace engine::render
