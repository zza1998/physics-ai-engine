#pragma once

#include <entt/entt.hpp>
#include <memory>
#include <vector>
#include <glm/glm.hpp>

namespace leo {

class System;
class Mesh;
class Shader;

class Scene {
public:
    Scene();
    ~Scene();

    entt::registry& registry() { return m_registry; }
    const entt::registry& registry() const { return m_registry; }

    entt::entity createEntity();
    void destroyEntity(entt::entity e);

    // 辅助创建函数: 物体用默认 mesh/shader (由 setDefaultMesh/Shader 注入)
    entt::entity addCube(const glm::vec3& pos,
                         const glm::vec3& scale = {1.0f, 1.0f, 1.0f},
                         const glm::vec3& albedo = {0.8f, 0.8f, 0.8f});
    entt::entity addPlane(float width, float depth,
                          const glm::vec3& albedo = {0.5f, 0.5f, 0.5f});
    entt::entity addDirectionalLight(const glm::vec3& dir, const glm::vec3& color);

    // 注入 M1 demo 用的默认 mesh/shader (立方体/平面共享一个 shader)
    void setCubeMesh(Mesh* m) { m_cubeMesh = m; }
    void setPlaneMesh(Mesh* m) { m_planeMesh = m; }
    void setDefaultShader(Shader* s) { m_defaultShader = s; }

    Mesh*   cubeMesh() const { return m_cubeMesh; }
    Mesh*   planeMesh() const { return m_planeMesh; }
    Shader* defaultShader() const { return m_defaultShader; }

    // System 注册 (Application 也持有一份, 这里方便测试时独立用)
    void addSystem(std::unique_ptr<System> s);
    const std::vector<std::unique_ptr<System>>& systems() const { return m_systems; }

private:
    entt::registry m_registry;
    std::vector<std::unique_ptr<System>> m_systems;

    Mesh*   m_cubeMesh     = nullptr;
    Mesh*   m_planeMesh    = nullptr;
    Shader* m_defaultShader = nullptr;
};

} // namespace leo
