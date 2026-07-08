#pragma once

#include <array>
#include <vector>
#include <entt/entt.hpp>
#include "physics/Math.h"
#include "physics/PhysicsTypes.h"

namespace leo {

// Collider 抽象接口: 碰撞代码只依赖此接口, 不知内部是点云还是 RB-PBD 刚体
// 阶段1: PointCloudBox (8 点 + 12 强距离约束) — M2 demo 历史, 待删
// 阶段2: RigidBodyBox (RB-PBD, 接口不变 + OBB 专用方法)
class Collider {
public:
    virtual ~Collider() = default;
    virtual AABB getAABB() const = 0;
    // 点 vs 此 collider, 返回接触信息 (normal 指向点应被推移的方向)
    virtual Contact collidePoint(const glm::vec3& p) const = 0;
    // 检查某点 entity 是否属于此 collider (避免自碰撞, M3 ragdoll 同组过滤用)
    virtual bool owns(entt::entity e) const = 0;
};

// 接触流形: OBB-OBB / OBB-AABB 碰撞结果 (多点)
struct ContactManifold {
    bool hit = false;
    glm::vec3 normal{0.0f};      // 最小穿透轴 (单位), 从 other 指向 this (this 应被推开的方向)
    float penetration = 0.0f;
    std::vector<glm::vec3> contact_points;  // 世界空间接触点 (1~4 个)
};

// RB-PBD 刚体盒子: 持有 RigidBody entity, OBB 碰撞
class RigidBodyBox : public Collider {
public:
    RigidBodyBox(entt::registry& reg, entt::entity body)
        : m_reg(reg), m_body(body) {}

    AABB getAABB() const override;
    Contact collidePoint(const glm::vec3& p) const override;  // 点 vs OBB (转局部用 pointVsAABB)
    bool owns(entt::entity e) const override { return false; }  // 刚体不持有点 entity

    // OBB-OBB SAT 碰撞 (15 轴), 返回流形 (normal 从 other 指向 this)
    ContactManifold collideOBB(const RigidBodyBox& other) const;
    // OBB vs 静态 AABB (AABB 视作 q=identity 的 OBB), normal 从 aabb 指向 this
    ContactManifold collideStaticAABB(const AABB& aabb) const;

    const RigidBody& body() const;
    RigidBody& bodyMut();
    entt::entity bodyEntity() const { return m_body; }

private:
    // 内部 SAT 实现: 此 OBB (A) vs 另一 OBB (B 的 center/axes/half)
    // normal 方向: 从 B 指向 A (A 应被推开)
    ContactManifold satOBB(
        const glm::vec3& cA, const glm::mat3& RA, const glm::vec3& hA,
        const glm::vec3& cB, const glm::mat3& RB, const glm::vec3& hB) const;

    entt::registry& m_reg;
    entt::entity m_body;
};

} // namespace leo

