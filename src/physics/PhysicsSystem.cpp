#include "physics/PhysicsSystem.h"
#include "physics/Verlet.h"
#include "scene/Scene.h"
#include "scene/Components.h"
#include "graphics/Mesh.h"
#include "graphics/Shader.h"
#include "Logger.h"
#include <cmath>
#include <limits>

namespace leo {

void PhysicsSystem::update(float dt, Scene& scene) {
    // dt 上限保护: 卡顿时不让物理爆炸
    if (dt > m_cfg.dt_max) dt = m_cfg.dt_max;
    if (dt <= 0.0f) return;

    auto& reg = scene.registry();
    // 帧开始: 若无 lastGood (首帧), 用当前状态初始化 (前提: 当前无 NaN)
    if (!m_hasLastGood) {
        snapshotPoints(reg);
        m_lastGood_curr = m_snapshot_curr;
        m_lastGood_prev = m_snapshot_prev;
        m_hasLastGood = true;
    }

    float sub_dt = dt / m_cfg.substeps;
    for (int s = 0; s < m_cfg.substeps; ++s) {
        substep(sub_dt, scene);
    }

    // 帧成功结束 (无 NaN): 更新 lastGood
    auto isNaN = [](float v) { return std::isnan(v) || std::isinf(v); };
    bool bad = false;
    auto view = reg.view<VerletPoint>();
    for (auto e : view) {
        VerletPoint& p = view.get<VerletPoint>(e);
        if (isNaN(p.x_current.x) || isNaN(p.x_current.y) || isNaN(p.x_current.z)) { bad = true; break; }
    }
    if (!bad) {
        snapshotPoints(reg);
        m_lastGood_curr = m_snapshot_curr;
        m_lastGood_prev = m_snapshot_prev;
    }
}

void PhysicsSystem::substep(float dt, Scene& scene) {
    auto& reg = scene.registry();

    snapshotPoints(reg);                       // substep 前快照 (检测用)
    integrate(reg, dt, m_cfg.gravity, m_cfg.damping);

    for (int iter = 0; iter < m_cfg.iterations; ++iter) {
        resolveCollisions(reg);                // 先解穿透
        solveConstraints(reg, dt);             // 再解距离/角度约束
    }

    if (detectAndRollback(reg)) {
        LEO_LOG_WARN("Physics NaN detected, rolled back a substep");
    }
}

void PhysicsSystem::resolveCollisions(entt::registry& reg) {
    // 1. VerletPoint vs StaticBody (地面/平台/墙)
    auto points = reg.view<VerletPoint>();
    auto statics = reg.view<StaticBody>();

    for (auto e : points) {
        VerletPoint& p = points.get<VerletPoint>(e);
        if (p.isStatic()) continue;

        for (auto se : statics) {
            const StaticBody& sb = statics.get<StaticBody>(se);
            Contact c = pointVsAABB(p.x_current, sb.aabb);
            if (c.hit) {
                // 沿法线推出穿透
                p.x_current += c.normal * c.penetration;
                // 切向摩擦: 沿接触面衰减速度 (位置差)
                // 简化: 把 x_prev 往 x_current 拉, 减少切向滑动
                glm::vec3 v = p.x_current - p.x_prev;
                // 切向 = v - (v·n)n
                glm::vec3 vt = v - glm::dot(v, c.normal) * c.normal;
                // 摩擦减少切向
                p.x_prev += vt * sb.friction;
            }
        }
    }

    // 2. VerletPoint vs ColliderRef (动态点云箱)
    //    不同 collision_group 的点才检测, 避免 ragdoll 自碰撞
    auto colliders = reg.view<ColliderRef>();
    for (auto e : points) {
        VerletPoint& p = points.get<VerletPoint>(e);
        if (p.isStatic()) continue;

        for (auto ce : colliders) {
            ColliderRef& cr = colliders.get<ColliderRef>(ce);
            if (!cr.collider) continue;
            // 简化: 不检查 collision_group (M2 demo 箱子都是独立组, 互碰 OK)
            Contact c = cr.collider->collidePoint(p.x_current);
            if (c.hit && c.penetration > 0.0001f) {
                p.x_current += c.normal * c.penetration;
            }
        }
    }
}

void PhysicsSystem::solveConstraints(entt::registry& reg, float dt) {
    for (auto& c : m_constraints) {
        c->project(reg, dt);
    }
}

void PhysicsSystem::applyDamping(entt::registry& reg) {
    // 全局阻尼已在 integrate 里通过 damping 参数施加, 此处留空 (预留分层阻尼)
    (void)reg;
}

void PhysicsSystem::snapshotPoints(entt::registry& reg) {
    m_snapshot_curr.clear();
    m_snapshot_prev.clear();
    auto view = reg.view<VerletPoint>();
    for (auto e : view) {
        VerletPoint& p = view.get<VerletPoint>(e);
        m_snapshot_curr.push_back({e, p.x_current});
        m_snapshot_prev.push_back({e, p.x_prev});
    }
}

bool PhysicsSystem::detectAndRollback(entt::registry& reg) {
    auto isNaN = [](float v) { return std::isnan(v) || std::isinf(v); };
    auto checkVec = [&](const glm::vec3& v) {
        return isNaN(v.x) || isNaN(v.y) || isNaN(v.z);
    };

    bool bad = false;
    auto view = reg.view<VerletPoint>();
    for (auto e : view) {
        VerletPoint& p = view.get<VerletPoint>(e);
        if (checkVec(p.x_current) || checkVec(p.x_prev)) { bad = true; break; }
    }

    if (bad && m_hasLastGood) {
        // 回滚到上一帧成功状态 (m_lastGood), 而非本 substep 前的含 NaN 状态
        for (auto& [e, pos] : m_lastGood_curr) {
            if (reg.valid(e) && reg.all_of<VerletPoint>(e)) {
                reg.get<VerletPoint>(e).x_current = pos;
            }
        }
        for (auto& [e, pos] : m_lastGood_prev) {
            if (reg.valid(e) && reg.all_of<VerletPoint>(e)) {
                reg.get<VerletPoint>(e).x_prev = pos;
            }
        }
        return true;
    }
    return false;
}

entt::entity PhysicsSystem::spawnBox(Scene& scene, const glm::vec3& pos, const glm::vec3& size,
                                     Mesh* cubeMesh, Shader* shader, const glm::vec3& albedo) {
    (void)cubeMesh; (void)shader; (void)albedo; // 动态箱子不画 cube mesh, 靠 DebugRenderer 画线框
    auto& reg = scene.registry();
    // 8 个角点 (单位立方体 ±0.5, 缩放 size, 平移 pos)
    glm::vec3 h = size * 0.5f;
    std::array<glm::vec3, 8> corners = {{
        {-h.x,-h.y,-h.z}, { h.x,-h.y,-h.z}, { h.x, h.y,-h.z}, {-h.x, h.y,-h.z},
        {-h.x,-h.y, h.z}, { h.x,-h.y, h.z}, { h.x, h.y, h.z}, {-h.x, h.y, h.z},
    }};

    std::array<entt::entity, 8> pointEntities;
    for (int i = 0; i < 8; ++i) {
        auto pe = reg.create();
        reg.emplace<VerletPoint>(pe, VerletPoint{pos + corners[i], pos + corners[i], 1.0f, 0});
        pointEntities[i] = pe;
    }

    // 12 条棱 (距离约束), stiffness=1.0 硬
    auto addEdge = [&](int i, int j) {
        float len = glm::distance(corners[i], corners[j]);
        m_constraints.push_back(std::make_unique<DistanceConstraint>(
            pointEntities[i], pointEntities[j], len, 1.0f));
    };
    addEdge(0,1); addEdge(1,2); addEdge(2,3); addEdge(3,0);
    addEdge(4,5); addEdge(5,6); addEdge(6,7); addEdge(7,4);
    addEdge(0,4); addEdge(1,5); addEdge(2,6); addEdge(3,7);
    // 6 条面对角线 (抗剪切, 防塌成平面)
    auto addFaceDiag = [&](int i, int j) {
        float len = glm::distance(corners[i], corners[j]);
        m_constraints.push_back(std::make_unique<DistanceConstraint>(
            pointEntities[i], pointEntities[j], len, 1.0f));
    };
    addFaceDiag(0,2); addFaceDiag(1,3);
    addFaceDiag(4,6); addFaceDiag(5,7);
    addFaceDiag(0,5); addFaceDiag(2,7);

    // 箱子主体: 仅 ColliderRef (8 点拥有权在 points entity, collider 引用它们)
    // 可视化靠 DebugRenderer (AABB 线框 + 点 + 约束线)
    auto boxEntity = reg.create();
    reg.emplace<ColliderRef>(boxEntity, ColliderRef{
        std::make_unique<PointCloudBox>(reg, pointEntities)
    });

    return boxEntity;
}

entt::entity PhysicsSystem::spawnStaticBox(Scene& scene, const glm::vec3& pos, const glm::vec3& size,
                                           Mesh* cubeMesh, Shader* shader, const glm::vec3& albedo) {
    auto& reg = scene.registry();
    auto e = reg.create();
    reg.emplace<Transform>(e, Transform{pos, {1,0,0,0}, size});
    reg.emplace<Renderable>(e, Renderable{cubeMesh, shader, albedo, 16.0f});
    // AABB 从 pos/size 推导
    glm::vec3 h = size * 0.5f;
    reg.emplace<StaticBody>(e, StaticBody{AABB{pos - h, pos + h}, 0.5f});
    return e;
}

} // namespace leo
