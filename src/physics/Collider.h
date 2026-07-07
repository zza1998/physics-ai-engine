#pragma once

#include <array>
#include <entt/entt.hpp>
#include "physics/Math.h"
#include "physics/PhysicsTypes.h"

namespace leo {

// Collider 抽象接口: 碰撞代码只依赖此接口, 不知内部是点云还是 RB-PBD 刚体
// 阶段1: PointCloudBox (8 点 + 12 强距离约束)
// 阶段2 (M6): RigidBodyBox (RB-PBD, 接口不变)
class Collider {
public:
    virtual ~Collider() = default;
    virtual AABB getAABB() const = 0;
    // 点 vs 此 collider, 返回接触信息 (normal 指向点应被推移的方向)
    virtual Contact collidePoint(const glm::vec3& p) const = 0;
    // 检查某点 entity 是否属于此 collider (避免自碰撞, M3 ragdoll 同组过滤用)
    virtual bool owns(entt::entity e) const = 0;
};

// 硬点云箱子: 8 个 VerletPoint + AABB 从 8 点重算
// collidePoint 用箱体 AABB 做点 vs AABB
class PointCloudBox : public Collider {
public:
    // 8 个点的 entity (非拥有), 由 PhysicsSystem::spawnBox 创建
    PointCloudBox(entt::registry& reg, std::array<entt::entity, 8> points);

    AABB getAABB() const override;
    Contact collidePoint(const glm::vec3& p) const override;

    const std::array<entt::entity, 8>& pointEntities() const { return m_points; }
    // 检查某点是否属于此箱子 (避免自碰撞)
    bool owns(entt::entity e) const override {
        for (auto pe : m_points) if (pe == e) return true;
        return false;
    }

private:
    entt::registry& m_reg;
    std::array<entt::entity, 8> m_points;
};

} // namespace leo
