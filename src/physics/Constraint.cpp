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
// 投影策略: 若当前夹角超出 [min, max], 把 a/c 点在 va-vc 平面内旋转回限位边界
// 关键: 使用"朝对方旋转"的方式 (在平面内插值), 避免叉积轴符号翻转导致振荡
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
    // 需要旋转的角度 (target - angle < 0 = 需减小角度 = 折叠)
    float rotate = target - angle;
    // 限制单步旋转量, 防止大角度修正注入能量 (落地瞬间关节急转→假性速度)
    rotate = std::clamp(rotate, -0.52f, 0.52f);  // ~30°/步

    // 反质量分配: 重的少转, 轻的多转
    float wa = pa->inv_mass;
    float wc = pc->inv_mass;
    float wsum = wa + wc;
    if (wsum == 0.0f) return;

    // 稳健旋转: 把 v1 朝 v2 方向旋转 ang (在 v1-v2 平面内)
    // 用 n1 + perp 基底插值, 不依赖叉积轴方向, 不会振荡
    // perp = normalize(v2_proj - (v2_proj·n1)*n1), 即 v2 在垂直于 v1 方向的分量
    // 正 ang = 朝 v2 转 (减小夹角), 负 ang = 远离 v2 (增大夹角)
    auto rotateTowards = [](glm::vec3 v1, glm::vec3 v2, float ang) {
        glm::vec3 n1 = glm::normalize(v1);
        glm::vec3 n2 = glm::normalize(v2);
        // v2 垂直于 v1 的分量
        glm::vec3 perp = n2 - glm::dot(n2, n1) * n1;
        float pl = glm::length(perp);
        if (pl < 1e-8f) {
            // 共线 (含反向 180°): 选任意垂直轴做旋转方向
            glm::vec3 ref = (std::abs(n1.y) < 0.9f) ? glm::vec3(0,1,0) : glm::vec3(1,0,0);
            perp = ref - glm::dot(ref, n1) * n1;
            pl = glm::length(perp);
            if (pl < 1e-8f) return v1;  // 极端退化, 不转
        }
        perp /= pl;
        // 在 n1-perp 平面内旋转 ang: 正=朝 v2 (减小夹角), 负=远离 (增大夹角)
        float c = std::cos(ang);
        float s = std::sin(ang);
        return (n1 * c + perp * s) * glm::length(v1);  // 保持原长度
    };

    // a 和 c 各分担一半角度变化, 按反质量分配比例
    // rotate<0 (需减小角度): 两者都朝对方转 (正角度)
    // rotate>0 (需增大角度): 两者都远离对方 (负角度)
    float shareA = wc / wsum;  // a 承担的比例 (c 越重 a 转越多)
    float shareC = wa / wsum;
    // a 朝 c 转 -rotate*shareA (rotate<0→正=减小, rotate>0→负=增大)
    float angA = -rotate * shareA * m_stiffness;
    float angC = -rotate * shareC * m_stiffness;

    if (!pa->isStatic()) {
        pa->x_current = pb->x_current + rotateTowards(va, vc, angA);
    }
    if (!pc->isStatic()) {
        pc->x_current = pb->x_current + rotateTowards(vc, va, angC);
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
