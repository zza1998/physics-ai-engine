#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

namespace leo {

class Mesh;
class Shader;

// Transform: M2 RB-PBD 朝向投影会复用 quaternion rotation
struct Transform {
    glm::vec3 position = {0.0f, 0.0f, 0.0f};
    glm::quat rotation = {1.0f, 0.0f, 0.0f, 0.0f}; // 单位四元数
    glm::vec3 scale    = {1.0f, 1.0f, 1.0f};

    glm::mat4 modelMatrix() const {
        glm::mat4 m = glm::translate(glm::mat4(1.0f), position);
        m *= glm::mat4_cast(rotation);
        m = glm::scale(m, scale);
        return m;
    }
};

// Renderable: mesh/shader 非拥有, 由 Application 持有的静态图元
struct Renderable {
    Mesh*    mesh      = nullptr;
    Shader*  shader    = nullptr;
    glm::vec3 albedo   = {0.8f, 0.8f, 0.8f};
    float    shininess = 32.0f;
};

// Light: M1 只用 Directional
struct Light {
    enum class Type { Directional, Point, Spot };
    Type     type      = Type::Directional;
    glm::vec3 color    = {1.0f, 1.0f, 1.0f};
    glm::vec3 direction = {-0.2f, -1.0f, -0.3f};
    glm::vec3 position = {0.0f, 0.0f, 0.0f}; // 点光用, M1 不用
    float    intensity = 1.0f;
};

// ---- M2 在此追加物理组件 (M1 不定义) ----
// struct VerletPoint  { glm::vec3 x_current, x_prev; float mass; int collision_group; };
// struct StaticBody   { AABB aabb; float friction; };
// struct ColliderRef  { ... };
// struct RagdollRef   { std::vector<entt::entity> points; std::vector<Constraint> constraints; };

} // namespace leo
