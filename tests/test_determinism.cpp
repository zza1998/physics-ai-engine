// 确定性测试: 相同初始状态 + 相同 dt 序列, 多帧后状态可复现
#include <gtest/gtest.h>
#include <glm/glm.hpp>
#include <vector>
#include <cmath>
#include "physics/PhysicsSystem.h"
#include "physics/PhysicsTypes.h"
#include "scene/Scene.h"
#include "scene/Components.h"

using namespace leo;

// 跑一个固定场景 N 帧, 返回所有 VerletPoint 的最终位置
static std::vector<glm::vec3> runSimulation(int frames) {
    Scene scene;
    PhysicsSystem phys;
    phys.config().gravity = {0, -9.81f, 0};
    phys.config().substeps = 4;
    phys.config().iterations = 8;
    phys.config().damping = 0.99f;

    auto& reg = scene.registry();
    // 几个点 + 一个距离约束
    auto a = reg.create();
    auto b = reg.create();
    reg.emplace<VerletPoint>(a, VerletPoint{{0,10,0}, {0,10,0}, 1.0f, 0});
    reg.emplace<VerletPoint>(b, VerletPoint{{1,10,0}, {1,10,0}, 1.0f, 0});
    // 地面
    auto ground = reg.create();
    reg.emplace<StaticBody>(ground, StaticBody{AABB{{-50,-1,-50},{50,0,50}}, 0.5f});

    // 固定 dt
    float dt = 0.0166667f; // ~60fps
    for (int i = 0; i < frames; ++i) {
        phys.update(dt, scene);
    }

    std::vector<glm::vec3> result;
    auto view = reg.view<VerletPoint>();
    for (auto e : view) result.push_back(view.get<VerletPoint>(e).x_current);
    return result;
}

TEST(Determinism, HundredFrameReplay) {
    auto run1 = runSimulation(100);
    auto run2 = runSimulation(100);

    ASSERT_EQ(run1.size(), run2.size());
    for (size_t i = 0; i < run1.size(); ++i) {
        EXPECT_NEAR(run1[i].x, run2[i].x, 1e-5f) << "点 " << i << " x 不一致";
        EXPECT_NEAR(run1[i].y, run2[i].y, 1e-5f) << "点 " << i << " y 不一致";
        EXPECT_NEAR(run1[i].z, run2[i].z, 1e-5f) << "点 " << i << " z 不一致";
    }
}

TEST(Determinism, StateChangesOverTime) {
    // 健全性: 100 帧后状态应不同于初始 (确认模拟真的在跑)
    auto initial = runSimulation(1);
    auto after = runSimulation(100);
    // 至少有一个点位置变化显著
    bool changed = false;
    for (size_t i = 0; i < initial.size(); ++i) {
        if (glm::distance(initial[i], after[i]) > 0.01f) { changed = true; break; }
    }
    EXPECT_TRUE(changed) << "模拟应产生状态变化";
}
