#pragma once

#include <entt/entt.hpp>
#include <glm/glm.hpp>

namespace leo {

// 约束基类: PBD 位置投影
// 持有 entity 而非裸指针, 防 entt pool 扩容导致指针失效
class Constraint {
public:
    virtual ~Constraint() = default;
    // 投影: 把违反约束的位置修正回满足约束的位置
    virtual void project(entt::registry& reg, float dt) = 0;
    // 当前违反量 (>=0), 调试用
    virtual float violation(entt::registry& reg) const = 0;
};

// 距离约束: |x_a - x_b| = rest_length
// 按反质量比分配位移 (重的点少动, 轻的点多动)
class DistanceConstraint : public Constraint {
public:
    DistanceConstraint(entt::entity a, entt::entity b, float rest_length, float stiffness = 1.0f)
        : m_a(a), m_b(b), m_rest(rest_length), m_stiffness(stiffness) {}

    void project(entt::registry& reg, float dt) override;
    float violation(entt::registry& reg) const override;

    entt::entity entityA() const { return m_a; }
    entt::entity entityB() const { return m_b; }

private:
    entt::entity m_a;
    entt::entity m_b;
    float m_rest;
    float m_stiffness;  // 0~1, 1.0 = 硬约束
};

// 角度约束: 三点 (a, b, c), 限制 b 处夹角 (a-b 向量与 c-b 向量夹角)
// M2 实现, demo 暂不用, M3 ragdoll 肘膝限位用
class AngleConstraint : public Constraint {
public:
    AngleConstraint(entt::entity a, entt::entity b, entt::entity c,
                    float min_angle, float max_angle, float stiffness = 1.0f)
        : m_a(a), m_b(b), m_c(c), m_min(min_angle), m_max(max_angle), m_stiffness(stiffness) {}

    void project(entt::registry& reg, float dt) override;
    float violation(entt::registry& reg) const override;

private:
    entt::entity m_a, m_b, m_c;
    float m_min, m_max;  // 弧度
    float m_stiffness;
};

} // namespace leo
