// Verlet 积分器单元测试: 重力下落 + 能量守恒
#include <gtest/gtest.h>
#include <glm/glm.hpp>
#include <entt/entt.hpp>
#include "physics/Verlet.h"
#include "physics/PhysicsTypes.h"

using namespace leo;

TEST(Verlet, GravityDrop) {
    entt::registry reg;
    auto e = reg.create();
    reg.emplace<VerletPoint>(e, VerletPoint{{0,10,0}, {0,10,0}, 1.0f, 0});

    glm::vec3 g{0, -9.81f, 0};
    float dt = 0.016f;
    // 跑 60 帧 (~1秒), 应该下落
    for (int i = 0; i < 60; ++i) integrate(reg, dt, g, 1.0f);

    VerletPoint& p = reg.get<VerletPoint>(e);
    EXPECT_LT(p.x_current.y, 10.0f) << "应该下落";
    // 自由落体 1s 下落约 4.9m (0.5*g*t²), verlet 近似
    EXPECT_NEAR(p.x_current.y, 10.0f - 0.5f * 9.81f * 1.0f * 1.0f, 0.5f);
}

TEST(Verlet, StaticPointDoesNotMove) {
    entt::registry reg;
    auto e = reg.create();
    reg.emplace<VerletPoint>(e, VerletPoint{{0,5,0}, {0,5,0}, 0.0f, 0}); // inv_mass=0 静态

    glm::vec3 g{0, -9.81f, 0};
    for (int i = 0; i < 60; ++i) integrate(reg, 0.016f, g, 1.0f);

    VerletPoint& p = reg.get<VerletPoint>(e);
    EXPECT_FLOAT_EQ(p.x_current.y, 5.0f) << "静态点不应移动";
}

TEST(Verlet, EnergyConservationNoGravity) {
    // 无重力 + 无阻尼, 点应匀速直线运动
    entt::registry reg;
    auto e = reg.create();
    // 初速度 = 1: x_current=1, x_prev=0 => (x_current-x_prev)=1 每步位移
    reg.emplace<VerletPoint>(e, VerletPoint{{1,0,0}, {0,0,0}, 1.0f, 0});

    glm::vec3 g{0,0,0};
    for (int i = 0; i < 10; ++i) integrate(reg, 0.1f, g, 1.0f);

    VerletPoint& p = reg.get<VerletPoint>(e);
    // verlet: x_{n+1} = 2x_n - x_{n-1}, 每步位移恒定 = 1, 10 步后 = 1 + 10 = 11
    // 动能守恒: 速度 (位置差) 恒定不变
    EXPECT_NEAR(p.x_current.x, 11.0f, 0.01f) << "无外力应匀速, 动能守恒";
    float velocity = (p.x_current.x - p.x_prev.x);
    EXPECT_NEAR(velocity, 1.0f, 1e-4f) << "速度应恒定";
}

TEST(Verlet, DampingReducesVelocity) {
    entt::registry reg;
    auto e = reg.create();
    reg.emplace<VerletPoint>(e, VerletPoint{{1,0,0}, {0,0,0}, 1.0f, 0});

    glm::vec3 g{0,0,0};
    // 无阻尼跑 10 步
    for (int i = 0; i < 10; ++i) integrate(reg, 0.1f, g, 1.0f);
    float noDampPos = reg.get<VerletPoint>(e).x_current.x;

    // 重置, 有阻尼跑 10 步
    reg.get<VerletPoint>(e) = VerletPoint{{1,0,0}, {0,0,0}, 1.0f, 0};
    for (int i = 0; i < 10; ++i) integrate(reg, 0.1f, g, 0.9f);
    float dampedPos = reg.get<VerletPoint>(e).x_current.x;

    EXPECT_LT(dampedPos, noDampPos) << "阻尼应减慢运动 (位移应小于无阻尼)";
    // 速度应递减: 当前步位移 < 第一步位移
    VerletPoint& p = reg.get<VerletPoint>(e);
    float currentStep = p.x_current.x - p.x_prev.x;
    EXPECT_LT(currentStep, 1.0f) << "阻尼后速度应小于初始 1.0";
}
