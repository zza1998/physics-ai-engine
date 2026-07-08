// RB-PBD 刚体单元测试
// Step1 基础: 惯性张量 / 自由落体 / 四元数积分不漂移
#include <gtest/gtest.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <entt/entt.hpp>
#include <cmath>
#include "physics/PhysicsTypes.h"
#include "physics/RBIntegrate.h"

using namespace leo;

// 1. 盒子惯性张量公式
TEST(RigidBody, BoxInertiaTensor) {
    float mass = 12.0f;  // 选 12 让 (1/12)*m 消去
    glm::vec3 half(1.0f, 2.0f, 3.0f);  // hx=1, hy=2, hz=3
    glm::mat3 invI = computeBoxInvInertiaLocal(mass, half);
    // Ixx = (1/12)*12*(2²+3²) = 13 -> inv=1/13
    // Iyy = (1/12)*12*(1²+3²) = 10 -> inv=1/10
    // Izz = (1/12)*12*(1²+2²) = 5  -> inv=1/5
    EXPECT_NEAR(invI[0][0], 1.0f / 13.0f, 1e-5f);  // [col][row], [0][0]=Ixx
    EXPECT_NEAR(invI[1][1], 1.0f / 10.0f, 1e-5f);  // Iyy
    EXPECT_NEAR(invI[2][2], 1.0f / 5.0f,  1e-5f);  // Izz
    // 非对角应为 0
    EXPECT_NEAR(invI[0][1], 0.0f, 1e-6f);
    EXPECT_NEAR(invI[1][2], 0.0f, 1e-6f);
}

// 2. 半隐式 Euler 自由落体
TEST(RigidBody, FreeFall) {
    entt::registry reg;
    auto e = reg.create();
    RigidBody rb;
    rb.x = {0, 10, 0};
    rb.inv_mass = 1.0f;
    rb.half_size = {0.5f, 0.5f, 0.5f};
    rb.inv_inertia_local = computeBoxInvInertiaLocal(1.0f, rb.half_size);
    reg.emplace<RigidBody>(e, rb);

    glm::vec3 g{0, -9.81f, 0};
    float dt = 1.0f / 60.0f;
    for (int i = 0; i < 60; ++i) integrateRigidBodies(reg, dt, g);  // ~1 秒

    RigidBody& r = reg.get<RigidBody>(e);
    // 自由落体 1s: x = 10 - 0.5*g*t² = 10 - 4.905 ~ 5.095
    // 半隐式 Euler 略有误差, 容忍 0.5
    EXPECT_NEAR(r.x.y, 10.0f - 0.5f * 9.81f, 0.5f);
    // 速度 ~ -g*t = -9.81
    EXPECT_NEAR(r.v.y, -9.81f, 0.5f);
}

// 3. 四元数积分不漂移 (无外力矩, 纯旋转)
TEST(RigidBody, QuaternionNoDrift) {
    entt::registry reg;
    auto e = reg.create();
    RigidBody rb;
    rb.x = {0, 0, 0};
    rb.inv_mass = 1.0f;
    rb.omega = {0, 1.0f, 0};  // 绕 Y 轴 1 rad/s
    rb.inv_inertia_local = computeBoxInvInertiaLocal(1.0f, {0.5f, 0.5f, 0.5f});
    reg.emplace<RigidBody>(e, rb);

    glm::vec3 g{0, 0, 0};
    float dt = 0.01f;
    for (int i = 0; i < 100; ++i) integrateRigidBodies(reg, dt, g);  // 1 秒

    RigidBody& r = reg.get<RigidBody>(e);
    // |q| 应保持 ~1 (normalize 每帧执行)
    float qlen = std::sqrt(r.q.x * r.q.x + r.q.y * r.q.y + r.q.z * r.q.z + r.q.w * r.q.w);
    EXPECT_NEAR(qlen, 1.0f, 1e-4f);
    // 绕 Y 轴旋转 1 rad: q.w = cos(0.5), q.y = sin(0.5)
    EXPECT_NEAR(r.q.w, std::cos(0.5f), 0.02f);
    EXPECT_NEAR(r.q.y, std::sin(0.5f), 0.02f);
    // x/z 应近 0 (纯绕 Y)
    EXPECT_NEAR(std::abs(r.q.x), 0.0f, 1e-3f);
    EXPECT_NEAR(std::abs(r.q.z), 0.0f, 1e-3f);
}

// 4. 静态刚体不积分
TEST(RigidBody, StaticDoesNotMove) {
    entt::registry reg;
    auto e = reg.create();
    RigidBody rb;
    rb.x = {0, 5, 0};
    rb.inv_mass = 0.0f;  // 静态
    reg.emplace<RigidBody>(e, rb);

    for (int i = 0; i < 60; ++i) integrateRigidBodies(reg, 1.0f / 60.0f, {0, -9.81f, 0});

    RigidBody& r = reg.get<RigidBody>(e);
    EXPECT_FLOAT_EQ(r.x.y, 5.0f);
    EXPECT_FLOAT_EQ(r.v.y, 0.0f);
}

// ---- OBB-OBB SAT 检测 ----
#include "physics/Collider.h"

// 辅助: 在 registry 里建一个 RigidBody + RigidBodyBox
static std::pair<entt::entity, RigidBodyBox*> makeBox(entt::registry& reg,
                                                       const glm::vec3& pos,
                                                       const glm::vec3& half,
                                                       const glm::quat& q = glm::quat(1,0,0,0)) {
    auto e = reg.create();
    RigidBody rb;
    rb.x = pos;
    rb.q = q;
    rb.half_size = half;
    rb.inv_mass = 1.0f;
    rb.inv_inertia_local = computeBoxInvInertiaLocal(1.0f, half);
    reg.emplace<RigidBody>(e, rb);
    auto& cr = reg.emplace<ColliderRef>(e, ColliderRef{});
    cr.collider = std::make_unique<RigidBodyBox>(reg, e);
    return {e, static_cast<RigidBodyBox*>(cr.collider.get())};
}

// 5. 两个同朝向 OBB 重叠
TEST(OBBSAT, OverlapBoxes) {
    entt::registry reg;
    auto [ea, ba] = makeBox(reg, {0,0,0}, {0.5f,0.5f,0.5f});
    auto [eb, bb] = makeBox(reg, {0.5f,0,0}, {0.5f,0.5f,0.5f});  // 中心距 0.5, 各半径 0.5 -> 重叠 0.5
    ContactManifold mf = ba->collideOBB(*bb);
    EXPECT_TRUE(mf.hit);
    EXPECT_GT(mf.penetration, 0.0f);
    EXPECT_GE(mf.contact_points.size(), 1u);
    // normal 应沿 X (从 B 指向 A, A 在 -X 侧 -> normal=(-1,0,0))
    EXPECT_NEAR(std::abs(mf.normal.x), 1.0f, 1e-4f);
}

// 6. 两个 OBB 分离
TEST(OBBSAT, SeparateBoxes) {
    entt::registry reg;
    auto [ea, ba] = makeBox(reg, {0,0,0}, {0.5f,0.5f,0.5f});
    auto [eb, bb] = makeBox(reg, {5,0,0}, {0.5f,0.5f,0.5f});  // 中心距 5, 远离
    ContactManifold mf = ba->collideOBB(*bb);
    EXPECT_FALSE(mf.hit);
}

// 7. 旋转后碰撞 (一个 OBB 旋转 45 度)
TEST(OBBSAT, RotatedBoxesOverlap) {
    entt::registry reg;
    // A 在原点, B 旋转 45 度绕 Z, 中心稍偏
    auto [ea, ba] = makeBox(reg, {0,0,0}, {1.0f,1.0f,0.5f});
    glm::quat q45 = glm::angleAxis(glm::radians(45.0f), glm::vec3(0,0,1));
    auto [eb, bb] = makeBox(reg, {1.5f,0,0}, {1.0f,1.0f,0.5f}, q45);
    ContactManifold mf = ba->collideOBB(*bb);
    // 旋转后 B 的角点伸到 x~1.5-1.414=0.086, 与 A 的右边 x=1 重叠
    EXPECT_TRUE(mf.hit);
    EXPECT_GT(mf.penetration, 0.0f);
}

// 8. OBB vs 静态 AABB
TEST(OBBSAT, OBBvsStaticAABB) {
    entt::registry reg;
    auto [e, b] = makeBox(reg, {0, 0.4f, 0}, {0.5f,0.5f,0.5f});
    // 地面 AABB: y<=0
    AABB ground{{-10,-1,-10},{10,0,10}};
    ContactManifold mf = b->collideStaticAABB(ground);
    EXPECT_TRUE(mf.hit);
    EXPECT_GT(mf.penetration, 0.0f);
    // normal 从地面指向 RB, 即 +Y 方向
    EXPECT_NEAR(mf.normal.y, 1.0f, 1e-4f);
}

// ---- RB-PBD 接触求解 (Step4) ----
#include "physics/PhysicsSystem.h"

// 9. 偏心接触产生旋转 (力矩)
TEST(RigidBodyContact, EccentricContactProducesRotation) {
    // 箱子质心在 (0, 0.5, 0), 接触点在 (0.5, 0, 0) (右下角, 偏心)
    // normal 从下往上 (+Y). 力臂 r=(0.5,-0.5,0), r×n = (0.5,-0.5,0)×(0,1,0) = (0,0,0.5)
    // 应产生绕 Z 的角速度 (翻转力矩)
    entt::registry reg;
    auto e = reg.create();
    RigidBody rb;
    rb.x = {0, 0.5f, 0};
    rb.half_size = {0.5f, 0.5f, 0.5f};
    rb.inv_mass = 1.0f;
    rb.inv_inertia_local = computeBoxInvInertiaLocal(1.0f, rb.half_size);
    rb.inv_inertia_world = rb.inv_inertia_local;
    reg.emplace<RigidBody>(e, rb);

    PhysicsSystem phys;
    glm::vec3 cp{0.5f, 0.0f, 0.0f};   // 偏心接触点 (右下角)
    glm::vec3 normal{0, 1, 0};        // 向上推
    phys.solveContactRB_Static(reg.get<RigidBody>(e), cp, normal, 0.1f, 1.0f);

    RigidBody& r = reg.get<RigidBody>(e);
    // 位置应上移 (inv_mass*lambda * normal)
    EXPECT_GT(r.x.y, 0.5f) << "偏心接触也应产生平移";
    // q 应偏离 identity (产生旋转) — w 分量 < 1
    EXPECT_LT(r.q.w, 1.0f - 1e-4f) << "偏心接触应产生旋转";
}

// 10. 中心接触不产生旋转
TEST(RigidBodyContact, CenteredContactNoRotation) {
    // 箱子质心在 (0, 0.5, 0), 接触点在 (0, 0, 0) (正下方, 中心)
    // r=(0,-0.5,0), r×n=(0,-0.5,0)×(0,1,0)=(0,0,0) -> 无力矩
    entt::registry reg;
    auto e = reg.create();
    RigidBody rb;
    rb.x = {0, 0.5f, 0};
    rb.half_size = {0.5f, 0.5f, 0.5f};
    rb.inv_mass = 1.0f;
    rb.inv_inertia_local = computeBoxInvInertiaLocal(1.0f, rb.half_size);
    rb.inv_inertia_world = rb.inv_inertia_local;
    reg.emplace<RigidBody>(e, rb);

    PhysicsSystem phys;
    glm::vec3 cp{0, 0, 0};            // 中心接触点
    glm::vec3 normal{0, 1, 0};
    phys.solveContactRB_Static(reg.get<RigidBody>(e), cp, normal, 0.1f, 1.0f);

    RigidBody& r = reg.get<RigidBody>(e);
    EXPECT_GT(r.x.y, 0.5f) << "应上移";
    // q 应保持 identity (无旋转) — w 仍 ~1
    EXPECT_NEAR(r.q.w, 1.0f, 1e-4f) << "中心接触不应产生旋转";
    EXPECT_NEAR(std::abs(r.q.x) + std::abs(r.q.y) + std::abs(r.q.z), 0.0f, 1e-5f);
}


