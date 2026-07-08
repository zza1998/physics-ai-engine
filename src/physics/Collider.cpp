#include "physics/Collider.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_access.hpp>  // glm::column
#include <cmath>
#include <limits>

namespace leo {

// ---- RigidBodyBox ----

const RigidBody& RigidBodyBox::body() const {
    return m_reg.get<RigidBody>(m_body);
}
RigidBody& RigidBodyBox::bodyMut() {
    return m_reg.get<RigidBody>(m_body);
}

AABB RigidBodyBox::getAABB() const {
    const RigidBody& rb = body();
    glm::mat3 R(rb.q);
    glm::vec3 lo(std::numeric_limits<float>::max());
    glm::vec3 hi(std::numeric_limits<float>::lowest());
    for (int i = 0; i < 8; ++i) {
        glm::vec3 local(
            (i & 1) ? rb.half_size.x : -rb.half_size.x,
            (i & 2) ? rb.half_size.y : -rb.half_size.y,
            (i & 4) ? rb.half_size.z : -rb.half_size.z);
        glm::vec3 world = rb.x + R * local;
        lo = glm::min(lo, world);
        hi = glm::max(hi, world);
    }
    return AABB{lo, hi};
}

Contact RigidBodyBox::collidePoint(const glm::vec3& p) const {
    // 点 vs OBB: 转到 OBB 局部空间, 用 pointVsAABB
    const RigidBody& rb = body();
    glm::mat3 R(rb.q);
    glm::mat3 Rt = glm::transpose(R);  // 世界->局部
    glm::vec3 local = Rt * (p - rb.x);
    AABB localBox{-rb.half_size, rb.half_size};
    Contact c = pointVsAABB(local, localBox);
    if (c.hit) {
        // 法线转回世界空间
        c.normal = R * c.normal;
    }
    return c;
}

// SAT 15 轴: A/B 各 3 轴 + 9 交叉轴. normal 从 B 指向 A.
ContactManifold RigidBodyBox::satOBB(
    const glm::vec3& cA, const glm::mat3& RA, const glm::vec3& hA,
    const glm::vec3& cB, const glm::mat3& RB, const glm::vec3& hB) const {

    ContactManifold mf;
    glm::vec3 t = cB - cA;  // A 到 B 的向量

    // 收集 15 轴 (RA[0..2]=A 的世界轴, RB[0..2]=B 的世界轴)
    // glm::mat3 列主序: RA[0] 是第 0 列 = A 的局部 X 轴在世界空间
    glm::vec3 axesA[3] = { glm::column(RA, 0), glm::column(RA, 1), glm::column(RA, 2) };
    glm::vec3 axesB[3] = { glm::column(RB, 0), glm::column(RB, 1), glm::column(RB, 2) };

    glm::vec3 allAxes[15];
    int nAxes = 0;
    for (int i = 0; i < 3; ++i) allAxes[nAxes++] = axesA[i];
    for (int i = 0; i < 3; ++i) allAxes[nAxes++] = axesB[i];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            glm::vec3 cr = glm::cross(axesA[i], axesB[j]);
            float len2 = glm::dot(cr, cr);
            if (len2 > 1e-6f) {
                allAxes[nAxes++] = cr / std::sqrt(len2);
            }
            // 退化 (平行) 跳过该轴
        }
    }

    float minPen = std::numeric_limits<float>::max();
    glm::vec3 minAxis(0.0f);
    bool separated = false;

    for (int k = 0; k < nAxes; ++k) {
        glm::vec3 L = allAxes[k];
        // 投影半径: r = sum half[i] * |dot(axis_i, L)|
        float rA = hA.x * std::abs(glm::dot(axesA[0], L))
                 + hA.y * std::abs(glm::dot(axesA[1], L))
                 + hA.z * std::abs(glm::dot(axesA[2], L));
        float rB = hB.x * std::abs(glm::dot(axesB[0], L))
                 + hB.y * std::abs(glm::dot(axesB[1], L))
                 + hB.z * std::abs(glm::dot(axesB[2], L));
        float d = glm::dot(t, L);
        float pen = rA + rB - std::abs(d);
        if (pen < 0.0f) { separated = true; break; }
        if (pen < minPen) {
            minPen = pen;
            // normal 从 B 指向 A: 若 d>0 (B 在 A 的 +L 侧), A 应往 -L 推
            minAxis = (d > 0.0f) ? -L : L;
        }
    }

    if (separated || minPen == std::numeric_limits<float>::max()) {
        return mf;  // hit=false
    }

    mf.hit = true;
    mf.normal = minAxis;
    mf.penetration = minPen;

    // 多接触点流形: 选法线最对齐的面 (A 或 B 的某面), 取该面 4 角点作为候选接触点.
    // 保留落在对方体内 (沿法线投影在对方该方向半径内) 的点, 最多 4 个.
    // 这对面-面接触 (箱子平放地面/堆叠) 产生 4 点, 位置投影时保持稳定不抖;
    // 单点采样会在迭代间跳变接触点 → 角速度激发.
    auto collectFacePoints = [](const glm::vec3& center, const glm::mat3& R, const glm::vec3& h,
                                const glm::vec3& faceNormal /*指向对方, 物体外法线*/) {
        // 找 faceNormal 在物体局部哪根轴最对齐, 确定该面 4 角的局部符号
        glm::vec3 localN = glm::transpose(R) * faceNormal;
        int axis = 0;
        float maxAbs = std::abs(localN.x);
        if (std::abs(localN.y) > maxAbs) { maxAbs = std::abs(localN.y); axis = 1; }
        if (std::abs(localN.z) > maxAbs) { maxAbs = std::abs(localN.z); axis = 2; }
        float sign = (localN[axis] > 0.0f) ? 1.0f : -1.0f;
        std::vector<glm::vec3> pts;
        for (int i = 0; i < 4; ++i) {
            glm::vec3 local(0.0f);
            local[axis] = sign * h[axis];
            // 另两轴遍历 +-h
            int a0 = (axis + 1) % 3;
            int a1 = (axis + 2) % 3;
            local[a0] = (i & 1) ? h[a0] : -h[a0];
            local[a1] = (i & 2) ? h[a1] : -h[a1];
            pts.push_back(center + R * local);
        }
        return pts;
    };

    // normal 从 B 指向 A. A 朝 B 的外法线 = -normal (A 应被推开, 其接触面朝 -normal).
    // B 朝 A 的外法线 = +normal.
    auto ptsA = collectFacePoints(cA, RA, hA, -mf.normal);  // A 的接触面 4 角
    auto ptsB = collectFacePoints(cB, RB, hB,  mf.normal);  // B 的接触面 4 角

    // 保留落在对方体内的点: 对 A 的面点, 检查它在 B 的 OBB 内; 反之亦然.
    auto inOBB = [](const glm::vec3& p, const glm::vec3& c, const glm::mat3& R, const glm::vec3& h) {
        glm::vec3 lp = glm::transpose(R) * (p - c);
        return std::abs(lp.x) <= h.x + 1e-4f &&
               std::abs(lp.y) <= h.y + 1e-4f &&
               std::abs(lp.z) <= h.z + 1e-4f;
    };
    for (const auto& p : ptsA) if (inOBB(p, cB, RB, hB)) mf.contact_points.push_back(p);
    for (const auto& p : ptsB) if (inOBB(p, cA, RA, hA)) mf.contact_points.push_back(p);

    // 兜底: 若 clipping 全落空 (边缘/点接触), 退化为最深顶点单点.
    if (mf.contact_points.empty()) {
        float bestDepth = -std::numeric_limits<float>::max();
        glm::vec3 bestPoint(0.0f);
        auto considerBody = [&](const glm::vec3& center, const glm::mat3& R, const glm::vec3& h,
                                const glm::vec3& dir) {
            for (int i = 0; i < 8; ++i) {
                glm::vec3 local(
                    (i & 1) ? h.x : -h.x,
                    (i & 2) ? h.y : -h.y,
                    (i & 4) ? h.z : -h.z);
                glm::vec3 world = center + R * local;
                float depth = glm::dot(world - center, dir);
                if (depth > bestDepth) {
                    bestDepth = depth;
                    bestPoint = world;
                }
            }
        };
        considerBody(cA, RA, hA, -mf.normal);
        considerBody(cB, RB, hB,  mf.normal);
        mf.contact_points.push_back(bestPoint);
    }
    return mf;
}

ContactManifold RigidBodyBox::collideOBB(const RigidBodyBox& other) const {
    const RigidBody& a = body();
    const RigidBody& b = other.body();
    glm::mat3 RA(a.q), RB(b.q);
    return satOBB(a.x, RA, a.half_size, b.x, RB, b.half_size);
}

ContactManifold RigidBodyBox::collideStaticAABB(const AABB& aabb) const {
    // AABB 视作 q=identity 的 OBB, center=aabb.center(), half=aabb.size()/2
    const RigidBody& a = body();
    glm::mat3 RA(a.q);
    glm::mat3 RIdent(1.0f);
    glm::vec3 cB = aabb.center();
    glm::vec3 hB = aabb.size() * 0.5f;
    ContactManifold mf = satOBB(a.x, RA, a.half_size, cB, RIdent, hB);
    // satOBB 的 normal 是 "从 B 指向 A", 即从 aabb 指向 RB, 符合 collideStaticAABB 约定
    return mf;
}

} // namespace leo

