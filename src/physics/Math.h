#pragma once

#include <glm/glm.hpp>
#include <algorithm>

namespace leo {

// AABB: 用 lo/hi 避免与 Windows min/max 宏冲突
struct AABB {
    glm::vec3 lo;
    glm::vec3 hi;

    glm::vec3 center() const { return (lo + hi) * 0.5f; }
    glm::vec3 size() const { return hi - lo; }
    bool contains(const glm::vec3& p) const {
        return p.x >= lo.x && p.x <= hi.x &&
               p.y >= lo.y && p.y <= hi.y &&
               p.z >= lo.z && p.z <= hi.z;
    }
};

// 接触信息: 点 vs AABB 碰撞结果
struct Contact {
    glm::vec3 normal;       // 碰撞法线 (从箱体指向点, 即点应被推开的方向)
    float     penetration;  // 穿透深度 (>=0)
    bool      hit;          // 是否碰撞
};

// 点 vs AABB: 若点在 AABB 内, 沿最近面推出
// 返回的 normal 是点应被推移的方向 (从 AABB 指向点)
inline Contact pointVsAABB(const glm::vec3& p, const AABB& aabb) {
    Contact c{glm::vec3(0.0f), 0.0f, false};
    if (!aabb.contains(p)) return c;
    c.hit = true;

    // 找最近的面: 计算点到 6 个面的距离, 取最小
    float dxLo = p.x - aabb.lo.x;
    float dxHi = aabb.hi.x - p.x;
    float dyLo = p.y - aabb.lo.y;
    float dyHi = aabb.hi.y - p.y;
    float dzLo = p.z - aabb.lo.z;
    float dzHi = aabb.hi.z - p.z;

    float minPen = dxLo;
    glm::vec3 n(-1.0f, 0.0f, 0.0f);  // 点在 +X 方向被推出? 不: 点应沿 -X 推到 lo 面
    // 修正: normal 是点应被推移的方向. 若点靠近 lo.x 面, 应往 -X 推 (穿出 lo 面)
    // 但物理上我们要把点推出箱体到最近面外, normal 指向推去方向

    auto consider = [&](float pen, const glm::vec3& dir) {
        if (pen < minPen) { minPen = pen; n = dir; }
    };
    consider(dxLo, glm::vec3(-1.0f, 0.0f, 0.0f));  // 推到 lo.x 面外, 方向 -X
    consider(dxHi, glm::vec3( 1.0f, 0.0f, 0.0f));  // 推到 hi.x 面外, 方向 +X
    consider(dyLo, glm::vec3(0.0f, -1.0f, 0.0f));
    consider(dyHi, glm::vec3(0.0f,  1.0f, 0.0f));
    consider(dzLo, glm::vec3(0.0f, 0.0f, -1.0f));
    consider(dzHi, glm::vec3(0.0f, 0.0f,  1.0f));

    c.normal = n;
    c.penetration = minPen;
    return c;
}

} // namespace leo
