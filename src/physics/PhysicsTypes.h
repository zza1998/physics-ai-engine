#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
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

// RB-PBD 刚体: 质心 + 惯性张量 + 力矩, 替代点云箱子
// 半隐式 Euler 积分: 显式存 v/omega. inv_mass=0 表示静态 (不积分)
struct RigidBody {
    glm::vec3 x{0.0f};              // 质心位置 (世界)
    glm::quat q{1.0f, 0.0f, 0.0f, 0.0f};  // 朝向 (单位四元数)
    glm::vec3 v{0.0f};              // 线速度 (世界)
    glm::vec3 omega{0.0f};          // 角速度 (世界, rad/s)
    float     inv_mass = 1.0f;      // 反质量, 0 = 静态
    glm::mat3 inv_inertia_local{1.0f};   // 局部反惯性张量 (常量)
    glm::mat3 inv_inertia_world{1.0f};   // 世界反惯性张量 (每帧从 q 重算)
    glm::vec3 half_size{0.5f};      // 盒子半尺寸 (局部)
    float     friction = 0.8f;
    float     restitution = 0.0f;   // 恢复系数 (0=非弹性, demo 默认 0)

    // 速度反算用: substep 内 integrate 后/接触求解前的快照
    glm::vec3 x_prev{0.0f};
    glm::quat q_prev{1.0f, 0.0f, 0.0f, 0.0f};

    // 休眠: 速度持续低于阈值时标记 sleeping, 跳过积分/碰撞 (被接触唤醒)
    bool  sleeping = false;
    float sleepTimer = 0.0f;

    bool isStatic() const { return inv_mass == 0.0f; }
    // 活跃: 非静态且未休眠. 休眠体在接触求解里视作静态 (不被动), 但可被唤醒.
    bool isActive() const { return inv_mass != 0.0f && !sleeping; }
};

// 盒子局部惯性张量: Ixx=(1/12)m(hy²+hz²) 等, inv_I=diag(1/Ixx,1/Iyy,1/Izz)
inline glm::mat3 computeBoxInvInertiaLocal(float mass, const glm::vec3& half_size) {
    float hx = half_size.x, hy = half_size.y, hz = half_size.z;
    float Ixx = (1.0f / 12.0f) * mass * (hy * hy + hz * hz);
    float Iyy = (1.0f / 12.0f) * mass * (hx * hx + hz * hz);
    float Izz = (1.0f / 12.0f) * mass * (hx * hx + hy * hy);
    return glm::mat3(
        1.0f / Ixx, 0.0f,       0.0f,
        0.0f,       1.0f / Iyy, 0.0f,
        0.0f,       0.0f,       1.0f / Izz
    );
}

// Collider 引用: 拥有 Collider 多态对象 (硬点云箱 / RB-PBD 刚体)
struct ColliderRef {
    std::unique_ptr<Collider> collider;
};

} // namespace leo

