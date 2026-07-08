#include "physics/PhysicsSystem.h"
#include "physics/Verlet.h"
#include "physics/RBIntegrate.h"
#include "physics/Collider.h"
#include "scene/Scene.h"
#include "scene/Components.h"
#include "graphics/Mesh.h"
#include "graphics/Shader.h"
#include "Logger.h"
#include <cmath>
#include <algorithm>
#include <limits>

namespace leo {

// 碰撞响应配置 (Verlet 点路径用, M3 ragdoll)
static constexpr float kPushSoften   = 0.5f;
static constexpr float kMaxPushStep  = 0.10f;
static constexpr float kFrictionTang = 0.5f;

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
    if (!m_hasLastGoodRB) {
        snapshotRigidBodies(reg);
        m_lastGoodRB = m_snapshotRB;
        m_hasLastGoodRB = true;
    }

    float sub_dt = dt / m_cfg.substeps;
    for (int s = 0; s < m_cfg.substeps; ++s) {
        substep(sub_dt, scene);
    }

    // 帧成功结束 (无 NaN): 更新 lastGood + 同步 Transform 供渲染
    auto isNaN = [](float v) { return std::isnan(v) || std::isinf(v); };
    bool badV = false, badR = false;
    for (auto e : reg.view<VerletPoint>()) {
        VerletPoint& p = reg.get<VerletPoint>(e);
        if (isNaN(p.x_current.x) || isNaN(p.x_current.y) || isNaN(p.x_current.z)) { badV = true; break; }
    }
    if (!badV) {
        snapshotPoints(reg);
        m_lastGood_curr = m_snapshot_curr;
        m_lastGood_prev = m_snapshot_prev;
    }
    auto checkVec = [&](const glm::vec3& v){ return isNaN(v.x)||isNaN(v.y)||isNaN(v.z); };
    for (auto e : reg.view<RigidBody>()) {
        RigidBody& rb = reg.get<RigidBody>(e);
        if (checkVec(rb.x) || checkVec(rb.omega)) { badR = true; break; }
    }
    if (!badR) {
        snapshotRigidBodies(reg);
        m_lastGoodRB = m_snapshotRB;
    }
    updateSleep(reg, dt);          // 休眠: 低速持续→sleeping (被撞时 collectContacts 唤醒)
    syncTransforms(reg);  // RB.x/q -> Transform (每帧末, 供渲染)
}

void PhysicsSystem::substep(float dt, Scene& scene) {
    auto& reg = scene.registry();

    // Verlet 路径 (M3 ragdoll 用, M2 demo 无 Verlet 点时跳过)
    snapshotPoints(reg);
    integrate(reg, dt, m_cfg.gravity, m_cfg.damping);

    // RB 路径: 半隐式 Euler 积分 (存 x_prev/q_prev 供速度反算)
    integrateRigidBodies(reg, dt, m_cfg.gravity);

    // 捕获本子步接触流形 (积分后, 位置求解前).
    // 速度求解用此缓存, 即使位置投影已分离也仍施加冲量, 避免 v.y 残留导致抖动.
    collectContacts(reg);

    // 接触求解迭代
    for (int iter = 0; iter < m_cfg.iterations; ++iter) {
        resolveCollisions(reg);        // Verlet 点 vs AABB (保留)
        resolveRBCollisions(reg);      // RB vs Static + RB vs RB
        solveConstraints(reg, dt);     // Verlet 距离/角度约束 (保留)
    }

    // RB 速度求解: 冲量法 (保留重力 v, 只对接触法向归零 + 切向摩擦)
    // 用 collectContacts 缓存的流形, 不重新检测穿透, 确保已分离接触仍归零法向速度.
    applyContactImpulses(reg);
    applyRigidBodyDamping(reg);

    if (detectAndRollback(reg)) {
        LEO_LOG_WARN("Physics NaN (Verlet) detected, rolled back a substep");
    }
    if (detectAndRollbackRB(reg)) {
        LEO_LOG_WARN("Physics NaN (RigidBody) detected, rolled back a substep");
    }
}

void PhysicsSystem::resolveCollisions(entt::registry& reg) {
    // 1. VerletPoint vs StaticBody (地面/平台/墙) — M3 ragdoll 点用, 单点推
    auto points = reg.view<VerletPoint>();
    auto statics = reg.view<StaticBody>();

    for (auto e : points) {
        VerletPoint& p = points.get<VerletPoint>(e);
        if (p.isStatic()) continue;

        for (auto se : statics) {
            const StaticBody& sb = statics.get<StaticBody>(se);
            Contact c = pointVsAABB(p.x_current, sb.aabb);
            if (!c.hit) continue;
            float push = c.penetration * kPushSoften;
            if (push > kMaxPushStep) push = kMaxPushStep;
            p.x_current += c.normal * push;
            // 切向摩擦: 衰减切向速度 (位置差)
            glm::vec3 v = p.x_current - p.x_prev;
            glm::vec3 vt = v - glm::dot(v, c.normal) * c.normal;
            p.x_prev += vt * sb.friction * kFrictionTang;
        }
    }

    // 2. VerletPoint vs ColliderRef (动态刚体) — ragdoll 点撞 RB 箱, 单点推 (RB 不被动)
    auto colliders = reg.view<ColliderRef>();
    for (auto e : points) {
        VerletPoint& p = points.get<VerletPoint>(e);
        if (p.isStatic()) continue;

        for (auto ce : colliders) {
            ColliderRef& cr = colliders.get<ColliderRef>(ce);
            if (!cr.collider) continue;
            if (cr.collider->owns(e)) continue;  // 自碰撞过滤 (ragdoll 同组)
            Contact c = cr.collider->collidePoint(p.x_current);
            if (!c.hit || c.penetration <= 0.0001f) continue;
            float push = c.penetration * kPushSoften;
            if (push > kMaxPushStep) push = kMaxPushStep;
            p.x_current += c.normal * push;
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
    auto& reg = scene.registry();
    // RB-PBD 刚体箱子: RigidBody + Transform(scale=size) + Renderable(实心 cube) + ColliderRef(RigidBodyBox)
    auto e = reg.create();
    glm::vec3 half = size * 0.5f;
    float mass = 1.0f;
    RigidBody rb;
    rb.x = pos;
    rb.q = glm::quat(1,0,0,0);
    rb.half_size = half;
    rb.inv_mass = 1.0f / mass;
    rb.inv_inertia_local = computeBoxInvInertiaLocal(mass, half);
    rb.inv_inertia_world = rb.inv_inertia_local;
    reg.emplace<RigidBody>(e, rb);
    reg.emplace<Transform>(e, Transform{pos, {1,0,0,0}, size});
    reg.emplace<Renderable>(e, Renderable{cubeMesh, shader, albedo, 32.0f});
    auto& cr = reg.emplace<ColliderRef>(e, ColliderRef{});
    cr.collider = std::make_unique<RigidBodyBox>(reg, e);
    return e;
}

entt::entity PhysicsSystem::spawnStaticBox(Scene& scene, const glm::vec3& pos, const glm::vec3& size,
                                           Mesh* cubeMesh, Shader* shader, const glm::vec3& albedo) {
    auto& reg = scene.registry();
    auto e = reg.create();
    reg.emplace<Transform>(e, Transform{pos, {1,0,0,0}, size});
    reg.emplace<Renderable>(e, Renderable{cubeMesh, shader, albedo, 16.0f});
    // AABB 从 pos/size 推导
    glm::vec3 h = size * 0.5f;
    reg.emplace<StaticBody>(e, StaticBody{AABB{pos - h, pos + h}, 0.8f});
    return e;
}

// ---- RB-PBD 接触求解 (Step4) ----

// 单接触点位置修正: RB vs 静态 (静态不动, 全部位移由 RB 承担)
// cp: 世界接触点, normal: 从静态体指向 RB (RB 应被推开方向), pen: 穿透量
void PhysicsSystem::solveContactRB_Static(RigidBody& rb, const glm::vec3& cp,
                                          const glm::vec3& normal, float pen, float soften) {
    if (rb.isStatic()) return;
    // 每迭代重算世界惯性 (q 可能被前一次修正改过)
    glm::mat3 R(rb.q);
    rb.inv_inertia_world = R * rb.inv_inertia_local * glm::transpose(R);

    glm::vec3 r = cp - rb.x;
    float C = pen * soften;

    // 有效质量: w = inv_mass + dot(r x n, inv_I_world * (r x n))
    glm::vec3 rn = glm::cross(r, normal);
    float w = rb.inv_mass + glm::dot(rn, rb.inv_inertia_world * rn);
    if (w <= 0.0f) return;

    float lambda = C / w;
    glm::vec3 dx = (rb.inv_mass * lambda) * normal;
    glm::vec3 dtheta = rb.inv_inertia_world * rn * lambda;

    rb.x += dx;
    // 旋转: q += 0.5 * quat(0, dtheta) * q
    glm::quat dq(0.0f, dtheta.x, dtheta.y, dtheta.z);
    rb.q = glm::normalize(rb.q + 0.5f * dq * rb.q);
}

// 单接触点位置修正: RB vs RB (双向, 按反质量分配)
// cp: 世界接触点, normal: 从 B 指向 A (A 应被推开方向), pen: 穿透量
void PhysicsSystem::solveContactRB_RB(RigidBody& a, RigidBody& b, const glm::vec3& cp,
                                      const glm::vec3& normal, float pen, float soften) {
    // 每迭代重算世界惯性 (q 可能被前一次修正改过)
    if (!a.isStatic()) {
        glm::mat3 Ra(a.q);
        a.inv_inertia_world = Ra * a.inv_inertia_local * glm::transpose(Ra);
    }
    if (!b.isStatic()) {
        glm::mat3 Rb(b.q);
        b.inv_inertia_world = Rb * b.inv_inertia_local * glm::transpose(Rb);
    }
    glm::vec3 rA = cp - a.x;
    glm::vec3 rB = cp - b.x;
    float C = pen * soften;

    glm::vec3 rnA = glm::cross(rA, normal);
    glm::vec3 rnB = glm::cross(rB, normal);
    // sleeping 体视作静态 (w=0), 不被动, 但对方仍可借它求解
    float wA = a.isActive() ? (a.inv_mass + glm::dot(rnA, a.inv_inertia_world * rnA)) : 0.0f;
    float wB = b.isActive() ? (b.inv_mass + glm::dot(rnB, b.inv_inertia_world * rnB)) : 0.0f;
    float wsum = wA + wB;
    if (wsum <= 0.0f) return;

    float lambda = C / wsum;
    auto applyCorrection = [](RigidBody& rb, const glm::vec3& rn, float lam, const glm::vec3& n) {
        if (!rb.isActive()) return;
        rb.x += (rb.inv_mass * lam) * n;
        glm::vec3 dtheta = rb.inv_inertia_world * rn * lam;
        glm::quat dq(0.0f, dtheta.x, dtheta.y, dtheta.z);
        rb.q = glm::normalize(rb.q + 0.5f * dq * rb.q);
    };
    applyCorrection(a, rnA,  lambda, normal);   // A 沿 +normal 推
    applyCorrection(b, rnB, -lambda, normal);   // B 沿 -normal 推
}

// 速度反算: 从 substep 内 integrate 后位置 (x_prev/q_prev) -> 接触求解后位置 (x/q)
void PhysicsSystem::updateRigidBodyVelocities(entt::registry& reg, float dt) {
    if (dt <= 0.0f) return;
    auto view = reg.view<RigidBody>();
    for (auto e : view) {
        RigidBody& rb = view.get<RigidBody>(e);
        if (rb.isStatic()) { rb.v = {0,0,0}; rb.omega = {0,0,0}; continue; }
        // 线速度
        rb.v = (rb.x - rb.x_prev) / dt;
        // 角速度: 从 q_prev -> q 的旋转量反算
        glm::quat dq = rb.q * glm::inverse(rb.q_prev);
        // 保证 w>=0 (取短弧)
        if (dq.w < 0.0f) dq = -dq;
        float w = std::clamp(dq.w, -1.0f, 1.0f);
        float angle = 2.0f * std::acos(w);
        if (angle > 1e-5f) {
            float s = std::sin(angle * 0.5f);
            glm::vec3 axis(dq.x / s, dq.y / s, dq.z / s);
            rb.omega = (angle / dt) * axis;
        } else {
            rb.omega = {0, 0, 0};
        }
        // 更新 prev (下一子步的基准)
        rb.x_prev = rb.x;
        rb.q_prev = rb.q;
    }
}

void PhysicsSystem::applyRigidBodyDamping(entt::registry& reg) {
    auto view = reg.view<RigidBody>();
    for (auto e : view) {
        RigidBody& rb = view.get<RigidBody>(e);
        if (!rb.isActive()) continue;  // sleeping/static 跳过
        rb.v *= m_cfg.linear_damping;
        rb.omega *= m_cfg.angular_damping;
    }
}

// 休眠: 速度持续低于阈值一段时间 → sleeping (跳过积分/碰撞, 当静态处理).
// 被动态 RB 接触时在 collectContacts 唤醒. 消除堆叠微抖 + 省性能.
void PhysicsSystem::updateSleep(entt::registry& reg, float dt) {
    auto view = reg.view<RigidBody>();
    for (auto e : view) {
        RigidBody& rb = view.get<RigidBody>(e);
        if (rb.isStatic()) continue;
        if (rb.sleeping) continue;  // 已休眠, 等唤醒

        float linV = glm::length(rb.v);
        float angV = glm::length(rb.omega);
        if (linV < m_cfg.sleep_lin_threshold && angV < m_cfg.sleep_ang_threshold) {
            rb.sleepTimer += dt;
            if (rb.sleepTimer >= m_cfg.sleep_time) {
                rb.sleeping = true;
                rb.v = {0.0f, 0.0f, 0.0f};
                rb.omega = {0.0f, 0.0f, 0.0f};
            }
        } else {
            rb.sleepTimer = 0.0f;  // 速度高, 重置计时
        }
    }
}

// 捕获本子步的接触流形 (积分后, 位置求解前).
// 速度求解用此缓存, 避免位置投影分离后漏处理接触冲量 (导致 v.y 残留抖动).
void PhysicsSystem::collectContacts(entt::registry& reg) {
    m_cachedRBStatic.clear();
    m_cachedRBRB.clear();

    auto rbView = reg.view<RigidBody>();
    auto statics = reg.view<StaticBody>();

    // 1. RB vs Static
    for (auto e : rbView) {
        RigidBody& rb = rbView.get<RigidBody>(e);
        if (rb.isStatic()) continue;
        auto* cr = reg.try_get<ColliderRef>(e);
        if (!cr || !cr->collider) continue;
        auto* rbb = dynamic_cast<RigidBodyBox*>(cr->collider.get());
        if (!rbb) continue;

        for (auto se : statics) {
            const StaticBody& sb = statics.get<StaticBody>(se);
            ContactManifold mf = rbb->collideStaticAABB(sb.aabb);
            // 用一个容差: 即使穿透略低于 slop, 仍在接触面附近, 速度求解应归零法向速度.
            // (位置求解用 slop 做投影阈值, 速度求解不应同样阈值跳过)
            if (!mf.hit || mf.penetration < -0.02f) continue;  // -0.02: 允许微小分离也处理
            m_cachedRBStatic.push_back({e, se, mf});
        }
    }

    // 2. RB vs RB (OBB-OBB), AABB broadphase 预筛
    std::vector<entt::entity> dynBodies;
    for (auto e : rbView) {
        if (!rbView.get<RigidBody>(e).isStatic()) dynBodies.push_back(e);
    }
    for (size_t i = 0; i < dynBodies.size(); ++i) {
        auto ea = dynBodies[i];
        auto* cra = reg.try_get<ColliderRef>(ea);
        if (!cra || !cra->collider) continue;
        auto* ba = dynamic_cast<RigidBodyBox*>(cra->collider.get());
        if (!ba) continue;
        AABB aabbA = ba->getAABB();

        for (size_t j = i + 1; j < dynBodies.size(); ++j) {
            auto eb = dynBodies[j];
            auto* crb = reg.try_get<ColliderRef>(eb);
            if (!crb || !crb->collider) continue;
            auto* bb = dynamic_cast<RigidBodyBox*>(crb->collider.get());
            if (!bb) continue;
            AABB aabbB = bb->getAABB();
            if (aabbA.hi.x < aabbB.lo.x || aabbB.hi.x < aabbA.lo.x ||
                aabbA.hi.y < aabbB.lo.y || aabbB.hi.y < aabbA.lo.y ||
                aabbA.hi.z < aabbB.lo.z || aabbB.hi.z < aabbA.lo.z) continue;

            ContactManifold mf = ba->collideOBB(*bb);
            if (!mf.hit || mf.penetration < -0.02f) continue;
            m_cachedRBRB.push_back({ea, eb, mf});
            // 唤醒: sleeping RB 被动态 RB 接触 (且对方在动) → 唤醒
            RigidBody& ra = reg.get<RigidBody>(ea);
            RigidBody& rb = reg.get<RigidBody>(eb);
            if (ra.sleeping && rb.isActive() && glm::length(rb.v) > m_cfg.sleep_lin_threshold) {
                ra.sleeping = false; ra.sleepTimer = 0.0f;
            }
            if (rb.sleeping && ra.isActive() && glm::length(ra.v) > m_cfg.sleep_lin_threshold) {
                rb.sleeping = false; rb.sleepTimer = 0.0f;
            }
        }
    }
}

// 接触冲量法: 保留重力 v, 对每个接触消除法向接近速度 (restitution=0 归零)
// + 切向库仑摩擦 (上限 mu*|jn|). 同时处理 RB-Static 与 RB-RB.
// 关键: 用 collectContacts 缓存的流形 (积分后状态), 不重新检测穿透,
//       确保位置投影已分离的接触仍归零法向速度, 防止 v.y 残留抖动.
void PhysicsSystem::applyContactImpulses(entt::registry& reg) {
    auto rbView = reg.view<RigidBody>();

    // 先把所有 RB 的世界反惯性张量刷新一次 (q 可能被位置求解改过)
    for (auto e : rbView) {
        RigidBody& rb = rbView.get<RigidBody>(e);
        if (rb.isStatic()) continue;
        glm::mat3 R(rb.q);
        rb.inv_inertia_world = R * rb.inv_inertia_local * glm::transpose(R);
    }

    // ---- 1. RB vs Static 冲量 (法向归零 + 摩擦) ----
    for (const auto& c : m_cachedRBStatic) {
        if (!reg.valid(c.rb) || !reg.valid(c.sb)) continue;
        RigidBody& rb = reg.get<RigidBody>(c.rb);
        if (rb.isStatic()) continue;
        const StaticBody& sb = reg.get<StaticBody>(c.sb);

        for (const auto& cp : c.mf.contact_points) {
            glm::vec3 r = cp - rb.x;
            glm::vec3 vp = rb.v + glm::cross(rb.omega, r);
            float vn = glm::dot(vp, c.mf.normal);  // normal 从 static 指向 RB
            if (vn >= 0.0f) continue;  // 已分离方向, 不处理

            glm::vec3 rn = glm::cross(r, c.mf.normal);
            float w = rb.inv_mass + glm::dot(rn, rb.inv_inertia_world * rn);
            if (w <= 0.0f) continue;

            float targetVn = -rb.restitution * vn;  // 0 = 归零
            float dvn = targetVn - vn;
            float jn = dvn / w;
            rb.v += (rb.inv_mass * jn) * c.mf.normal;
            rb.omega += rb.inv_inertia_world * rn * jn;

            // 切向摩擦 (库仑)
            vp = rb.v + glm::cross(rb.omega, r);
            glm::vec3 vt_vec = vp - glm::dot(vp, c.mf.normal) * c.mf.normal;
            float vt_mag = glm::length(vt_vec);
            if (vt_mag > 1e-6f && jn > 0.0f) {
                glm::vec3 t = vt_vec / vt_mag;
                glm::vec3 rt = glm::cross(r, t);
                float wt = rb.inv_mass + glm::dot(rt, rb.inv_inertia_world * rt);
                if (wt > 0.0f) {
                    float jt = -vt_mag / wt;
                    float mu = sb.friction;
                    jt = std::clamp(jt, -mu * jn, mu * jn);
                    rb.v += (rb.inv_mass * jt) * t;
                    rb.omega += rb.inv_inertia_world * rt * jt;
                }
            }
        }
    }

    // ---- 2. RB vs RB 冲量 (双向, 按反质量分配; 法向归零 + 摩擦) ----
    for (const auto& c : m_cachedRBRB) {
        if (!reg.valid(c.a) || !reg.valid(c.b)) continue;
        RigidBody& a = reg.get<RigidBody>(c.a);
        RigidBody& b = reg.get<RigidBody>(c.b);
        // normal 从 b 指向 a (a 应被推开方向)
        const glm::vec3& n = c.mf.normal;

        for (const auto& cp : c.mf.contact_points) {
            glm::vec3 rA = cp - a.x;
            glm::vec3 rB = cp - b.x;
            glm::vec3 vpA = a.v + glm::cross(a.omega, rA);
            glm::vec3 vpB = b.v + glm::cross(b.omega, rB);
            glm::vec3 vrel = vpA - vpB;
            // n 从 b 指向 a (a 应被推开方向). 接近时 vn<0 (与 RB-Static 同号约定).
            float vn = glm::dot(vrel, n);
            if (vn >= 0.0f) continue;  // 已分离方向, 不处理

            glm::vec3 rnA = glm::cross(rA, n);
            glm::vec3 rnB = glm::cross(rB, n);
            // sleeping 体视作静态 (w=0), 不被动, 但对方仍可借它求解
            float wA = a.isActive() ? (a.inv_mass + glm::dot(rnA, a.inv_inertia_world * rnA)) : 0.0f;
            float wB = b.isActive() ? (b.inv_mass + glm::dot(rnB, b.inv_inertia_world * rnB)) : 0.0f;
            float wsum = wA + wB;
            if (wsum <= 0.0f) continue;

            float restitution = 0.5f * (a.restitution + b.restitution);
            float targetVn = -restitution * vn;  // 0 = 归零
            float dvn = targetVn - vn;           // 接近时 >0
            float jn = dvn / wsum;               // 接近时 >0
            // a 沿 +n, b 沿 -n
            if (a.isActive()) {
                a.v += (a.inv_mass * jn) * n;
                a.omega += a.inv_inertia_world * rnA * jn;
            }
            if (b.isActive()) {
                b.v -= (b.inv_mass * jn) * n;
                b.omega -= b.inv_inertia_world * rnB * jn;
            }

            // 切向摩擦 (库仑, 双向)
            vpA = a.v + glm::cross(a.omega, rA);
            vpB = b.v + glm::cross(b.omega, rB);
            vrel = vpA - vpB;
            glm::vec3 vt_vec = vrel - glm::dot(vrel, n) * n;
            float vt_mag = glm::length(vt_vec);
            if (vt_mag > 1e-6f && jn > 0.0f) {
                glm::vec3 t = vt_vec / vt_mag;
                glm::vec3 rtA = glm::cross(rA, t);
                glm::vec3 rtB = glm::cross(rB, t);
                float wtA = a.isActive() ? (a.inv_mass + glm::dot(rtA, a.inv_inertia_world * rtA)) : 0.0f;
                float wtB = b.isActive() ? (b.inv_mass + glm::dot(rtB, b.inv_inertia_world * rtB)) : 0.0f;
                float wtsum = wtA + wtB;
                if (wtsum > 0.0f) {
                    float jt = -vt_mag / wtsum;
                    float mu = 0.5f * (a.friction + b.friction);
                    jt = std::clamp(jt, -mu * jn, mu * jn);
                    if (a.isActive()) {
                        a.v += (a.inv_mass * jt) * t;
                        a.omega += a.inv_inertia_world * rtA * jt;
                    }
                    if (b.isActive()) {
                        b.v -= (b.inv_mass * jt) * t;
                        b.omega -= b.inv_inertia_world * rtB * jt;
                    }
                }
            }
        }
    }
}

// RB 碰撞: RB vs StaticBody(AABB) + RB vs RigidBodyBox(OBB-OBB, AABB broadphase 预筛)
void PhysicsSystem::resolveRBCollisions(entt::registry& reg) {
    float soften = m_cfg.rb_push_soften;

    // 1. RigidBody vs StaticBody (地面/平台)
    auto rbView = reg.view<RigidBody>();
    auto statics = reg.view<StaticBody>();
    for (auto e : rbView) {
        RigidBody& rb = rbView.get<RigidBody>(e);
        if (rb.isStatic()) continue;
        // 用 RigidBodyBox 做碰撞检测 (需要 ColliderRef)
        auto* cr = reg.try_get<ColliderRef>(e);
        if (!cr || !cr->collider) continue;
        auto* rbb = dynamic_cast<RigidBodyBox*>(cr->collider.get());
        if (!rbb) continue;

        for (auto se : statics) {
            const StaticBody& sb = statics.get<StaticBody>(se);
            ContactManifold mf = rbb->collideStaticAABB(sb.aabb);
            if (!mf.hit || mf.penetration < m_cfg.contact_slop) continue;
            for (const auto& cp : mf.contact_points) {
                solveContactRB_Static(rb, cp, mf.normal, mf.penetration, soften);
            }
            // 摩擦由 applyContactImpulses 冲量法统一处理 (位置迭代内摩擦会过度衰减)
            (void)sb;
        }
    }

    // 2. RigidBody vs RigidBody (OBB-OBB), O(N^2) + AABB broadphase 预筛
    // 收集所有动态 RB entity
    std::vector<entt::entity> dynBodies;
    for (auto e : rbView) {
        if (!rbView.get<RigidBody>(e).isStatic()) dynBodies.push_back(e);
    }
    for (size_t i = 0; i < dynBodies.size(); ++i) {
        auto ea = dynBodies[i];
        RigidBody& a = reg.get<RigidBody>(ea);
        auto* cra = reg.try_get<ColliderRef>(ea);
        if (!cra || !cra->collider) continue;
        auto* ba = dynamic_cast<RigidBodyBox*>(cra->collider.get());
        if (!ba) continue;
        AABB aabbA = ba->getAABB();

        for (size_t j = i + 1; j < dynBodies.size(); ++j) {
            auto eb = dynBodies[j];
            auto* crb = reg.try_get<ColliderRef>(eb);
            if (!crb || !crb->collider) continue;
            auto* bb = dynamic_cast<RigidBodyBox*>(crb->collider.get());
            if (!bb) continue;
            // AABB broadphase 预筛
            AABB aabbB = bb->getAABB();
            if (aabbA.hi.x < aabbB.lo.x || aabbB.hi.x < aabbA.lo.x ||
                aabbA.hi.y < aabbB.lo.y || aabbB.hi.y < aabbA.lo.y ||
                aabbA.hi.z < aabbB.lo.z || aabbB.hi.z < aabbA.lo.z) continue;

            RigidBody& b = reg.get<RigidBody>(eb);
            ContactManifold mf = ba->collideOBB(*bb);
            if (!mf.hit || mf.penetration < m_cfg.contact_slop) continue;
            for (const auto& cp : mf.contact_points) {
                solveContactRB_RB(a, b, cp, mf.normal, mf.penetration, soften);
            }
        }
    }
}

void PhysicsSystem::syncTransforms(entt::registry& reg) {
    // RB.x/q -> Transform (供渲染)
    auto view = reg.view<RigidBody, Transform>();
    for (auto e : view) {
        RigidBody& rb = view.get<RigidBody>(e);
        Transform& tf = view.get<Transform>(e);
        tf.position = rb.x;
        tf.rotation = rb.q;
        // scale = 2*half_size (cube mesh 是 +-0.5, scale 后半尺寸 = half_size)
        tf.scale = rb.half_size * 2.0f;
    }
}

void PhysicsSystem::snapshotRigidBodies(entt::registry& reg) {
    m_snapshotRB.clear();
    auto view = reg.view<RigidBody>();
    for (auto e : view) {
        RigidBody& rb = view.get<RigidBody>(e);
        m_snapshotRB.push_back({e, RBSnapshot{rb.x, rb.q, rb.v, rb.omega}});
    }
}

bool PhysicsSystem::detectAndRollbackRB(entt::registry& reg) {
    auto isNaN = [](float v) { return std::isnan(v) || std::isinf(v); };
    auto checkVec = [&](const glm::vec3& v) { return isNaN(v.x) || isNaN(v.y) || isNaN(v.z); };
    auto checkQuat = [&](const glm::quat& q) { return isNaN(q.x) || isNaN(q.y) || isNaN(q.z) || isNaN(q.w); };

    bool bad = false;
    auto view = reg.view<RigidBody>();
    for (auto e : view) {
        RigidBody& rb = view.get<RigidBody>(e);
        if (checkVec(rb.x) || checkVec(rb.v) || checkVec(rb.omega) || checkQuat(rb.q)) { bad = true; break; }
    }
    if (bad && m_hasLastGoodRB) {
        for (auto& [e, snap] : m_lastGoodRB) {
            if (reg.valid(e) && reg.all_of<RigidBody>(e)) {
                RigidBody& rb = reg.get<RigidBody>(e);
                rb.x = snap.x; rb.q = snap.q; rb.v = snap.v; rb.omega = snap.omega;
            }
        }
        return true;
    }
    return false;
}

} // namespace leo
