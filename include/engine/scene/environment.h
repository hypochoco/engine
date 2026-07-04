//
//  environment.h
//  engine::scene
//
//  Scene-level, ECS-facing environment settings: the background (clear) color and the world
//  light. These live as World resources (typed singletons) so gameplay/sim code configures
//  them at the ECS level; `applyEnvironment` copies them onto a render::RenderView, keeping the
//  Renderer unaware of the ECS. Both fall back to the RenderView's defaults when unset.
//

#pragma once

#include <glm/glm.hpp>

#include "engine/ecs/world.h"
#include "engine/graphics/render/render_view.h"

namespace engine::scene {

// Background (clear) color for the view.
struct Background {
    glm::vec4 color{0.08f, 0.10f, 0.14f, 1.0f};
};

// The world's directional light + ambient term.
struct SceneLighting {
    render::DirectionalLight light{};
};

// Copy the world's Background / SceneLighting resources (if present) onto `view`.
inline void applyEnvironment(ecs::World& world, render::RenderView& view) {
    if (const auto* bg = world.getResource<Background>()) {
        view.clearColor[0] = bg->color.r;
        view.clearColor[1] = bg->color.g;
        view.clearColor[2] = bg->color.b;
        view.clearColor[3] = bg->color.a;
    }
    if (const auto* sl = world.getResource<SceneLighting>()) {
        view.light = sl->light;
    }
}

} // namespace engine::scene
