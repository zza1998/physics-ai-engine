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
    float damping         = 0.99f;  // 位置阻尼 (每子步, Verlet)
    int   substeps        = 8;      // 每帧子步数
    int   iterations      = 16;     // 约束求解迭代
    float dt_max          = 1.0f / 30.0f;  // dt 上限, 防卡顿爆炸
    // RB-PBD 参数
    float angular_damping = 0.9f;   // RB 角速度阻尼 (每子步)
    float linear_damping  = 0.998f; // RB 线速度阻尼 (每子步, 接近 1 防过度衰减重力)
    float rb_push_soften  = 0.3f;   // RB 接触软化
    float contact_slop    = 0.005f; // 接触容差, 穿透 < slop 不处理
    // 休眠: 速度持续低于阈值一段时间后 sleeping, 跳过物理 (被接触唤醒)
    float sleep_lin_threshold  = 0.15f;  // 线速度阈值 (m/s), 放宽让微滑尽快休眠
    float sleep_ang_threshold  = 0.15f;  // 角速度阈值 (rad/s)
    float sleep_time           = 0.3f;   // 持续多久才休眠 (秒) (防微接触反复)
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

    // RB-PBD 接触求解 (public 便于单测 + 未来扩展)
    void solveContactRB_Static(RigidBody& rb, const glm::vec3& cp,
                               const glm::vec3& normal, float pen, float soften);
    void solveContactRB_RB(RigidBody& a, RigidBody& b, const glm::vec3& cp,
                           const glm::vec3& normal, float pen, float soften);
    void updateRigidBodyVelocities(entt::registry& reg, float dt);  // 位置反算速度

private:
    void substep(float dt, Scene& scene);
    void resolveCollisions(entt::registry& reg);
    void solveConstraints(entt::registry& reg, float dt);
    void applyDamping(entt::registry& reg);
    void snapshotPoints(entt::registry& reg);
    bool detectAndRollback(entt::registry& reg);  // true = 检测到 NaN 并回滚 (Verlet)

    // ---- RB-PBD (Step4+) ----
    void resolveRBCollisions(entt::registry& reg);              // RB vs Static + RB vs RB
    void applyContactImpulses(entt::registry& reg);             // 冲量法: 法向归零 + 切向摩擦 (保留重力 v)
    void updateSleep(entt::registry& reg, float dt);            // 休眠/唤醒: 低速持续→sleeping, 被撞→唤醒
    void applyRigidBodyDamping(entt::registry& reg);
    void syncTransforms(entt::registry& reg);                   // RB.x/q -> Transform
    void snapshotRigidBodies(entt::registry& reg);
    bool detectAndRollbackRB(entt::registry& reg);

    // 缓存本子步的接触流形 (积分后/位置求解前捕获).
    // 速度求解时即使位置投影已分离, 仍用此流形算冲量, 避免漏处理导致 v.y 残留.
    struct CachedRBStaticContact {
        entt::entity rb;
        entt::entity sb;
        ContactManifold mf;
    };
    struct CachedRBRBContact {
        entt::entity a;
        entt::entity b;
        ContactManifold mf;
    };
    void collectContacts(entt::registry& reg);
    std::vector<CachedRBStaticContact> m_cachedRBStatic;
    std::vector<CachedRBRBContact> m_cachedRBRB;

    PhysicsConfig m_cfg;
    std::vector<std::unique_ptr<Constraint>> m_constraints;
    // NaN 回滚: m_lastGood 是上一帧成功结束的状态, 回滚目标 (Verlet)
    std::vector<std::pair<entt::entity, glm::vec3>> m_snapshot_curr;  // substep 前 (检测用)
    std::vector<std::pair<entt::entity, glm::vec3>> m_snapshot_prev;
    std::vector<std::pair<entt::entity, glm::vec3>> m_lastGood_curr;  // 上一帧成功状态
    std::vector<std::pair<entt::entity, glm::vec3>> m_lastGood_prev;
    bool m_hasLastGood = false;

    // RB NaN 回滚快照
    struct RBSnapshot { glm::vec3 x; glm::quat q; glm::vec3 v; glm::vec3 omega; };
    std::vector<std::pair<entt::entity, RBSnapshot>> m_snapshotRB;
    std::vector<std::pair<entt::entity, RBSnapshot>> m_lastGoodRB;
    bool m_hasLastGoodRB = false;
};

} // namespace leo
