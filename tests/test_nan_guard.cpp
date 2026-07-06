// NaN 守卫单元测试: 极端输入触发回滚而非传播
#include <gtest/gtest.h>
#include <glm/glm.hpp>
#include <cmath>
#include "physics/PhysicsSystem.h"
#include "physics/PhysicsTypes.h"
#include "scene/Scene.h"
#include "scene/Components.h"

using namespace leo;

TEST(NanGuard, RollsBackOnNaN) {
    // 构造一个点, 注入 NaN, 跑 PhysicsSystem, 应回滚不传播
    Scene scene;
    PhysicsSystem phys;
    phys.config().gravity = {0, -9.81f, 0};

    auto& reg = scene.registry();
    auto e = reg.create();
    glm::vec3 goodPos{0, 5, 0};
    reg.emplace<VerletPoint>(e, VerletPoint{goodPos, goodPos, 1.0f, 0});

    // 跑几帧正常物理 (建立稳定快照)
    for (int i = 0; i < 3; ++i) phys.update(0.016f, scene);
    EXPECT_FALSE(std::isnan(reg.get<VerletPoint>(e).x_current.y));

    // 注入 NaN (模拟极端输入)
    reg.get<VerletPoint>(e).x_current = glm::vec3(NAN, NAN, NAN);

    // 跑一帧, 应检测到 NaN 并回滚 (不传播)
    phys.update(0.016f, scene);

    // 回滚后 x_current 不应再是 NaN
    VerletPoint& p = reg.get<VerletPoint>(e);
    EXPECT_FALSE(std::isnan(p.x_current.x)) << "NaN 应回滚, 不传播";
    EXPECT_FALSE(std::isnan(p.x_current.y));
    EXPECT_FALSE(std::isnan(p.x_current.z));
}

TEST(NanGuard, ClampsHugeDt) {
    // dt 异常大, 应被 clamp, 不爆炸
    Scene scene;
    PhysicsSystem phys;
    auto& reg = scene.registry();
    auto e = reg.create();
    reg.emplace<VerletPoint>(e, VerletPoint{{0,5,0}, {0,5,0}, 1.0f, 0});

    // 喂一个超大 dt (10秒), 应被 clamp 到 dt_max (1/30)
    phys.update(10.0f, scene);

    VerletPoint& p = reg.get<VerletPoint>(e);
    EXPECT_FALSE(std::isnan(p.x_current.y));
    EXPECT_FALSE(std::isinf(p.x_current.y));
    // 不应飞到无穷远
    EXPECT_GT(p.x_current.y, -1000.0f);
    EXPECT_LT(p.x_current.y, 1000.0f);
}
