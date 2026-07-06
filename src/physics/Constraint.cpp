#include "physics/Constraint.h"
#include "physics/PhysicsTypes.h"
#include <glm/geometric.hpp>
#include <cmath>
#include <algorithm>

namespace leo {

// ---- DistanceConstraint ----
void DistanceConstraint::project(entt::registry& reg, float /*dt*/) {
    if (!reg.valid(m_a) || !reg.valid(m_b)) return;
    auto* pa = reg.try_get<VerletPoint>(m_a);
    auto* pb = reg.try_get<VerletPoint>(m_b);
    if (!pa || !pb) return;

    glm::vec3 delta = pa->x_current - pb->x_current;
    float dist2 = glm::dot(delta, delta);
    if (dist2 < 1e-12f) {
        // 重合: 沿任意轴推一点
        delta = glm::vec3(1.0f, 0.0f, 0.0f);
        dist2 = 1.0f;
    }
    float dist = std::sqrt(dist2);
    float diff = (dist - m_rest) / dist;  // 相对违反量

    // 反质量比: w_a / (w_a + w_b), 静态点 inv_mass=0 不动
    float wa = pa->inv_mass;
    float wb = pb->inv_mass;
    float wsum = wa + wb;
    if (wsum == 0.0f) return;  // 两端都静态, 不解

    float corr = diff * m_stiffness;
    if (!pa->isStatic()) pa->x_current -= delta * (corr * (wa / wsum));
    if (!pb->isStatic()) pb->x_current += delta * (corr * (wb / wsum));
}

float DistanceConstraint::violation(entt::registry& reg) const {
    if (!reg.valid(m_a) || !reg.valid(m_b)) return 0.0f;
    auto* pa = reg.try_get<VerletPoint>(m_a);
    auto* pb = reg.try_get<VerletPoint>(m_b);
    if (!pa || !pb) return 0.0f;
    float dist = glm::distance(pa->x_current, pb->x_current);
    return std::abs(dist - m_rest);
}

// ---- AngleConstraint ----
// 投影策略 (简化): 若当前夹角超出 [min, max], 把 a/c 点绕 b 旋转回限位边界
void AngleConstraint::project(entt::registry& reg, float /*dt*/) {
    if (!reg.valid(m_a) || !reg.valid(m_b) || !reg.valid(m_c)) return;
    auto* pa = reg.try_get<VerletPoint>(m_a);
    auto* pb = reg.try_get<VerletPoint>(m_b);
    auto* pc = reg.try_get<VerletPoint>(m_c);
    if (!pa || !pb || !pc) return;
    // 中点静态也允许解 (只移动 a 和 c, 绕 b 旋转)

    glm::vec3 va = pa->x_current - pb->x_current;
    glm::vec3 vc = pc->x_current - pb->x_current;
    float la = glm::length(va);
    float lc = glm::length(vc);
    if (la < 1e-6f || lc < 1e-6f) return;

    float cosA = glm::dot(va, vc) / (la * lc);
    cosA = std::clamp(cosA, -1.0f, 1.0f);
    float angle = std::acos(cosA);

    if (angle >= m_min && angle <= m_max) return;  // 在限位内, 不解

    float target = (angle < m_min) ? m_min : m_max;
    // 简化: 仅移动 a 和 c (绕 b), 按反质量分配
    // 完整实现需绕旋转轴投影, M3 ragdoll 时细化. M2 给一个稳定近似:
    // 把 a 和 c 在它们构成的平面内旋转到目标角
    float rotate = target - angle;  // 需要旋转的角度
    // 旋转轴 = va × vc, 共线时退化, 用任意垂直轴
    glm::vec3 axis = glm::cross(va, vc);
    if (glm::dot(axis, axis) < 1e-8f) {
        // 共线: 选与 va 不平行的轴做叉积
        glm::vec3 ref = (std::abs(va.y) < 0.9f) ? glm::vec3(0,1,0) : glm::vec3(1,0,0);
        axis = glm::cross(va, ref);
        if (glm::dot(axis, axis) < 1e-12f) return;
    }
    axis = glm::normalize(axis);

    // Rodrigues 旋转, 角度按反质量分摊
    float wa = pa->inv_mass;
    float wc = pc->inv_mass;
    float wsum = wa + wc;
    if (wsum == 0.0f) return;

    auto rotateVec = [](glm::vec3 v, glm::vec3 axis, float ang) {
        float c = std::cos(ang);
        float s = std::sin(ang);
        return v * c + glm::cross(axis, v) * s + axis * glm::dot(axis, v) * (1.0f - c);
    };

    if (!pa->isStatic()) {
        pa->x_current = pb->x_current + rotateVec(va, axis, rotate * (wc / wsum) * m_stiffness);
    }
    if (!pc->isStatic()) {
        pc->x_current = pb->x_current + rotateVec(vc, -axis, rotate * (wa / wsum) * m_stiffness);
    }
}

float AngleConstraint::violation(entt::registry& reg) const {
    if (!reg.valid(m_a) || !reg.valid(m_b) || !reg.valid(m_c)) return 0.0f;
    auto* pa = reg.try_get<VerletPoint>(m_a);
    auto* pb = reg.try_get<VerletPoint>(m_b);
    auto* pc = reg.try_get<VerletPoint>(m_c);
    if (!pa || !pb || !pc) return 0.0f;
    glm::vec3 va = pa->x_current - pb->x_current;
    glm::vec3 vc = pc->x_current - pb->x_current;
    float la = glm::length(va), lc = glm::length(vc);
    if (la < 1e-6f || lc < 1e-6f) return 0.0f;
    float angle = std::acos(std::clamp(glm::dot(va, vc) / (la * lc), -1.0f, 1.0f));
    if (angle < m_min) return m_min - angle;
    if (angle > m_max) return angle - m_max;
    return 0.0f;
}

} // namespace leo
