#include "scene/Scene.h"
#include "scene/Components.h"
#include "scene/System.h"

namespace leo {

Scene::Scene() = default;
Scene::~Scene() = default;

entt::entity Scene::createEntity() {
    return m_registry.create();
}

void Scene::destroyEntity(entt::entity e) {
    m_registry.destroy(e);
}

entt::entity Scene::addCube(const glm::vec3& pos,
                            const glm::vec3& scale,
                            const glm::vec3& albedo) {
    auto e = m_registry.create();
    m_registry.emplace<Transform>(e, Transform{pos, {1,0,0,0}, scale});
    m_registry.emplace<Renderable>(e, Renderable{m_cubeMesh, m_defaultShader, albedo, 32.0f});
    return e;
}

entt::entity Scene::addPlane(float width, float depth,
                             const glm::vec3& albedo) {
    auto e = m_registry.create();
    m_registry.emplace<Transform>(e, Transform{{0,0,0}, {1,0,0,0}, {width,1.0f,depth}});
    m_registry.emplace<Renderable>(e, Renderable{m_planeMesh, m_defaultShader, albedo, 16.0f});
    return e;
}

entt::entity Scene::addDirectionalLight(const glm::vec3& dir, const glm::vec3& color) {
    auto e = m_registry.create();
    m_registry.emplace<Light>(e, Light{Light::Type::Directional, color, dir, {0,0,0}, 1.0f});
    return e;
}

void Scene::addSystem(std::unique_ptr<System> s) {
    m_systems.push_back(std::move(s));
}

} // namespace leo
