#pragma once

#include <memory>
#include <vector>
#include <glm/glm.hpp>
#include "scene/System.h"
#include "physics/PhysicsTypes.h"
#include "physics/Constraint.h"
#include "physics/Collider.h"

namespace leo {

struct PhysicsConfig {
    glm::vec3 gravity{0.0f, -9.81f, 0.0f};
    float damping         = 0.99f;  // 位置阻尼 (每子步)
    int   substeps        = 8;      // 每帧子步数
    int   iterations      = 16;     // 约束求解迭代
    float dt_max          = 1.0f / 30.0f;  // dt 上限, 防卡顿爆炸
    bool  debug_draw_points    = true;
    bool  debug_draw_colliders = true;
    bool  debug_draw_constraints = true;
};

class PhysicsSystem : public System {
public:
    PhysicsSystem(PhysicsConfig cfg = {}) : m_cfg(cfg) {}

    void update(float dt, Scene& scene) override;
    const char* name() const override { return "PhysicsSystem"; }

    // demo 辅助: 创建一个可推动的点云箱子
    // 8 点 + 12 距离约束, 加 Renderable (用 cube mesh) + ColliderRef
    entt::entity spawnBox(Scene& scene, const glm::vec3& pos, const glm::vec3& size,
                          class Mesh* cubeMesh, class Shader* shader,
                          const glm::vec3& albedo = {0.8f, 0.4f, 0.2f});

    // 创建静态平台/墙 (StaticBody + 可选 Renderable)
    entt::entity spawnStaticBox(Scene& scene, const glm::vec3& pos, const glm::vec3& size,
                                class Mesh* cubeMesh, class Shader* shader,
                                const glm::vec3& albedo = {0.5f, 0.5f, 0.5f});

    PhysicsConfig& config() { return m_cfg; }
    const PhysicsConfig& config() const { return m_cfg; }

    // 约束访问 (DebugRenderer 用)
    const std::vector<std::unique_ptr<Constraint>>& constraints() const { return m_constraints; }

private:
    void substep(float dt, Scene& scene);
    void resolveCollisions(entt::registry& reg);
    void solveConstraints(entt::registry& reg, float dt);
    void applyDamping(entt::registry& reg);
    void snapshotPoints(entt::registry& reg);
    bool detectAndRollback(entt::registry& reg);  // true = 检测到 NaN 并回滚

    PhysicsConfig m_cfg;
    std::vector<std::unique_ptr<Constraint>> m_constraints;
    // NaN 回滚: m_lastGood 是上一帧成功结束的状态, 回滚目标
    std::vector<std::pair<entt::entity, glm::vec3>> m_snapshot_curr;  // substep 前 (检测用)
    std::vector<std::pair<entt::entity, glm::vec3>> m_snapshot_prev;
    std::vector<std::pair<entt::entity, glm::vec3>> m_lastGood_curr;  // 上一帧成功状态
    std::vector<std::pair<entt::entity, glm::vec3>> m_lastGood_prev;
    bool m_hasLastGood = false;
};

} // namespace leo
