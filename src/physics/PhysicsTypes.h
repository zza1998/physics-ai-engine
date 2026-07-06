#pragma once

#include <glm/glm.hpp>
#include <memory>
#include "physics/Math.h"

namespace leo {

class Collider; // 前向声明, ColliderRef 持有

// Verlet 点: 位置隐式速度 (x_current - x_prev)
// inv_mass = 0 表示无限质量 (静态/钉住)
struct VerletPoint {
    glm::vec3 x_current{0.0f};
    glm::vec3 x_prev{0.0f};
    float     inv_mass = 1.0f;     // 反质量, 0 = 无限质量
    int       collision_group = 0; // 相邻点同组不互碰 (M3 ragdoll 用)

    bool isStatic() const { return inv_mass == 0.0f; }
};

// 静态刚体: 不动的 AABB (地面/墙/平台), 无限质量
struct StaticBody {
    AABB  aabb;
    float friction = 0.3f;
};

// Collider 引用: 拥有 Collider 多态对象 (硬点云箱 / RB-PBD 刚体)
struct ColliderRef {
    std::unique_ptr<Collider> collider;
};

} // namespace leo
