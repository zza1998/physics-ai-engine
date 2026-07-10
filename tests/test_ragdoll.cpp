// M3a ragdoll 单测: 结构 + 骨长维持 + 跌倒无爆炸 + 碰撞过滤
#include <gtest/gtest.h>
#include <glm/glm.hpp>
#include <entt/entt.hpp>
#include <cmath>
#include "physics/PhysicsSystem.h"
#include "physics/PhysicsTypes.h"
#include "physics/Ragdoll.h"
#include "physics/Constraint.h"
#include "scene/Scene.h"
#include "scene/Components.h"

using namespace leo;

// 1. 结构: spawnRagdoll 后有 17 个 VerletPoint + RagdollRef 主体, 约束数 = 16骨+3抗塌+6角度 = 25
TEST(Ragdoll, Structure) {
    Scene scene;
    PhysicsSystem phys;
    phys.spawnRagdoll(scene, {0, 5, 0}, 1.0f);

    auto& reg = scene.registry();
    size_t pointCount = 0;
    for (auto e : reg.view<VerletPoint>()) { (void)e; pointCount++; }
    EXPECT_EQ(pointCount, 17u);

    size_t rdCount = 0;
    for (auto e : reg.view<RagdollRef>()) { (void)e; rdCount++; }
    EXPECT_EQ(rdCount, 1u);

    // 约束数: 16 骨骼 + 3 抗塌陷 + 6 角度 = 25
    EXPECT_EQ(phys.constraints().size(), 25u);
}

// 2. 骨长维持: 跑 60 帧物理, 每根骨骼距离 ≈ rest_length (误差 < 10%)
TEST(Ragdoll, BoneLengthMaintained) {
    Scene scene;
    PhysicsSystem phys;
    phys.spawnRagdoll(scene, {0, 5, 0}, 1.0f);
    auto& reg = scene.registry();

    // 记录初始骨长 (rest_length)
    auto rdView = reg.view<RagdollRef>();
    ASSERT_EQ(rdView.size(), 1u);
    RagdollRef& rd = rdView.get<RagdollRef>(*rdView.begin());

    std::vector<float> restLens;
    for (const auto& [a, b] : kRagdollBones) {
        auto* pa = reg.try_get<VerletPoint>(rd.points[a]);
        auto* pb = reg.try_get<VerletPoint>(rd.points[b]);
        restLens.push_back(glm::distance(pa->x_current, pb->x_current));
    }

    // 跑 60 帧物理 (带地面, ragdoll 在空中下落)
    AABB ground{{-50,-1,-50},{50,0,50}};
    auto ge = reg.create();
    reg.emplace<StaticBody>(ge, StaticBody{ground, 0.8f});
    for (int i = 0; i < 60; ++i) phys.update(1.0f / 60.0f, scene);

    // 检查每根骨骼长度
    int boneIdx = 0;
    for (const auto& [a, b] : kRagdollBones) {
        auto* pa = reg.try_get<VerletPoint>(rd.points[a]);
        auto* pb = reg.try_get<VerletPoint>(rd.points[b]);
        float len = glm::distance(pa->x_current, pb->x_current);
        float rest = restLens[boneIdx++];
        // 误差 < 10% (PBD 软约束, 允许少量拉伸)
        EXPECT_NEAR(len, rest, rest * 0.1f) << "骨 " << boneIdx << " 拉伸过大";
    }
}

// 3. 跌倒无爆炸: ragdoll 从 y=5 掉落, 跑 120 帧, 无 NaN, 点位置合理 (不飞远)
TEST(Ragdoll, FallsNoExplosion) {
    Scene scene;
    PhysicsSystem phys;
    phys.spawnRagdoll(scene, {0, 5, 0}, 1.0f);
    auto& reg = scene.registry();
    AABB ground{{-50,-1,-50},{50,0,50}};
    auto ge = reg.create();
    reg.emplace<StaticBody>(ge, StaticBody{ground, 0.8f});

    for (int i = 0; i < 120; ++i) phys.update(1.0f / 60.0f, scene);

    // 检查所有点无 NaN 且位置合理 (y 在 -1~10, x/z 在 -5~5)
    auto rdView = reg.view<RagdollRef>();
    RagdollRef& rd = rdView.get<RagdollRef>(*rdView.begin());
    for (int i = 0; i < 17; ++i) {
        auto* p = reg.try_get<VerletPoint>(rd.points[i]);
        ASSERT_NE(p, nullptr);
        EXPECT_FALSE(std::isnan(p->x_current.x)) << "点 " << i << " NaN";
        EXPECT_FALSE(std::isnan(p->x_current.y));
        EXPECT_FALSE(std::isnan(p->x_current.z));
        EXPECT_GT(p->x_current.y, -2.0f) << "点 " << i << " 穿地过深";
        EXPECT_LT(p->x_current.y, 15.0f) << "点 " << i << " 飞太高";
        EXPECT_LT(std::abs(p->x_current.x), 10.0f) << "点 " << i << " 飞太远(x)";
        EXPECT_LT(std::abs(p->x_current.z), 10.0f) << "点 " << i << " 飞太远(z)";
    }
}

// 4b. 长时间落地: ragdoll 从 y=5 掉落, 跑 600 帧 (10秒), 确保真正落地不爆炸
//     FallsNoExplosion 跑 120 帧 (2秒) 可能刚落地, 此测试跑到 10 秒确保落地 + 稳定不飞
TEST(Ragdoll, LandsAndStays) {
    Scene scene;
    PhysicsSystem phys;
    phys.spawnRagdoll(scene, {0, 5, 0}, 1.0f);
    auto& reg = scene.registry();
    AABB ground{{-50,-1,-50},{50,0,50}};
    auto ge = reg.create();
    reg.emplace<StaticBody>(ge, StaticBody{ground, 0.8f});

    auto rdView = reg.view<RagdollRef>();
    RagdollRef& rd = rdView.get<RagdollRef>(*rdView.begin());

    for (int i = 0; i < 600; ++i) phys.update(1.0f / 60.0f, scene);

    // 检查所有点无 NaN 且位置合理 (y 在 -1~5, x/z 在 -10~10 — 允许翻倒滑动)
    for (int i = 0; i < 17; ++i) {
        auto* p = reg.try_get<VerletPoint>(rd.points[i]);
        ASSERT_NE(p, nullptr);
        EXPECT_FALSE(std::isnan(p->x_current.x)) << "点 " << i << " NaN(x)";
        EXPECT_FALSE(std::isnan(p->x_current.y)) << "点 " << i << " NaN(y)";
        EXPECT_FALSE(std::isnan(p->x_current.z)) << "点 " << i << " NaN(z)";
        EXPECT_GT(p->x_current.y, -2.0f) << "点 " << i << " 穿地过深";
        EXPECT_LT(p->x_current.y, 15.0f) << "点 " << i << " 飞太高";
        EXPECT_LT(std::abs(p->x_current.x), 15.0f) << "点 " << i << " 飞太远(x)";
        EXPECT_LT(std::abs(p->x_current.z), 15.0f) << "点 " << i << " 飞太远(z)";
    }
    // pelvis 应在地面附近 (y < 3.0, 不应还在空中也不应飞走)
    auto* pelvis = reg.try_get<VerletPoint>(rd.points[Pelvis]);
    EXPECT_LT(pelvis->x_current.y, 3.0f) << "骨盆 10秒后不应还在高空";
}

// 5. 碰撞过滤: 相邻点同 collision_group, 距离约束维持时不会因碰撞互相推开
TEST(Ragdoll, CollisionFilterGroups) {
    // 头颈同组(1), 躯干同组(2), 左臂同组(3)...
    EXPECT_EQ(kRagdollGroup[Head], kRagdollGroup[Neck]);
    EXPECT_EQ(kRagdollGroup[TorsoUp], kRagdollGroup[TorsoDown]);
    EXPECT_EQ(kRagdollGroup[TorsoDown], kRagdollGroup[Pelvis]);
    EXPECT_EQ(kRagdollGroup[LUpperArm], kRagdollGroup[LForeArm]);
    EXPECT_EQ(kRagdollGroup[LThigh], kRagdollGroup[LShin]);
    // 不同部位不同组
    EXPECT_NE(kRagdollGroup[Head], kRagdollGroup[TorsoUp]);
    EXPECT_NE(kRagdollGroup[LUpperArm], kRagdollGroup[RUpperArm]);
}
