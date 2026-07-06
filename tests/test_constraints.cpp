// 约束求解器单元测试: 距离约束 + 质量分配 + 角度限位
#include <gtest/gtest.h>
#include <glm/glm.hpp>
#include <entt/entt.hpp>
#include "physics/Constraint.h"
#include "physics/PhysicsTypes.h"
#include <cmath>

using namespace leo;

TEST(DistanceConstraint, ProjectsToRestLength) {
    entt::registry reg;
    auto a = reg.create();
    auto b = reg.create();
    // 两点距离 2.0, rest=1.0, 应被拉近到 1.0
    reg.emplace<VerletPoint>(a, VerletPoint{{0,0,0}, {0,0,0}, 1.0f, 0});
    reg.emplace<VerletPoint>(b, VerletPoint{{2,0,0}, {2,0,0}, 1.0f, 0});

    DistanceConstraint c(a, b, 1.0f, 1.0f);
    // 多次投影收敛
    for (int i = 0; i < 50; ++i) c.project(reg, 0.016f);

    float dist = glm::distance(reg.get<VerletPoint>(a).x_current,
                               reg.get<VerletPoint>(b).x_current);
    EXPECT_NEAR(dist, 1.0f, 1e-4f) << "距离约束应投影到 rest_length";
    EXPECT_NEAR(c.violation(reg), 0.0f, 1e-4f);
}

TEST(DistanceConstraint, MassRatioAllocation) {
    // a 重 (inv_mass 小), b 轻 (inv_mass 大): a 少动, b 多动
    entt::registry reg;
    auto a = reg.create();
    auto b = reg.create();
    reg.emplace<VerletPoint>(a, VerletPoint{{0,0,0}, {0,0,0}, 0.1f, 0});  // 重
    reg.emplace<VerletPoint>(b, VerletPoint{{2,0,0}, {2,0,0}, 1.0f, 0});  // 轻
    glm::vec3 aStart{0,0,0}, bStart{2,0,0};

    DistanceConstraint c(a, b, 1.0f, 1.0f);
    for (int i = 0; i < 50; ++i) c.project(reg, 0.016f);

    glm::vec3 aDisp = reg.get<VerletPoint>(a).x_current - aStart;
    glm::vec3 bDisp = reg.get<VerletPoint>(b).x_current - bStart;
    // a 位移幅度应小于 b (重质量少动)
    EXPECT_LT(glm::length(aDisp), glm::length(bDisp))
        << "重质量点位移应小于轻质量点";
    // 位移比应近似反质量比: |aDisp|/|bDisp| ≈ wa/wb = 0.1/1.0 = 0.1
    float ratio = (glm::length(bDisp) > 1e-6f) ? glm::length(aDisp) / glm::length(bDisp) : 999.f;
    EXPECT_NEAR(ratio, 0.1f, 0.05f) << "位移应按反质量比分配";
}

TEST(DistanceConstraint, StaticPointDoesNotMove) {
    entt::registry reg;
    auto a = reg.create();
    auto b = reg.create();
    reg.emplace<VerletPoint>(a, VerletPoint{{0,0,0}, {0,0,0}, 0.0f, 0});  // 静态
    reg.emplace<VerletPoint>(b, VerletPoint{{2,0,0}, {2,0,0}, 1.0f, 0});

    DistanceConstraint c(a, b, 1.0f, 1.0f);
    for (int i = 0; i < 50; ++i) c.project(reg, 0.016f);

    // a 不动, b 被拉到距 a 为 1.0
    EXPECT_FLOAT_EQ(reg.get<VerletPoint>(a).x_current.x, 0.0f);
    float dist = glm::distance(reg.get<VerletPoint>(a).x_current,
                               reg.get<VerletPoint>(b).x_current);
    EXPECT_NEAR(dist, 1.0f, 1e-4f);
}

TEST(AngleConstraint, EnforcesMaxAngle) {
    // 三点 a-b-c, 初始夹角 180° (共线), 限制最大 90°, 应被弯回
    entt::registry reg;
    auto a = reg.create();
    auto b = reg.create();
    auto c = reg.create();
    reg.emplace<VerletPoint>(a, VerletPoint{{-1,0,0}, {-1,0,0}, 1.0f, 0});
    reg.emplace<VerletPoint>(b, VerletPoint{{0,0,0},  {0,0,0},  0.0f, 0}); // 中点静态
    reg.emplace<VerletPoint>(c, VerletPoint{{1,0,0},  {1,0,0},  1.0f, 0});

    AngleConstraint ac(a, b, c, 0.0f, glm::radians(90.0f), 1.0f);
    for (int i = 0; i < 50; ++i) ac.project(reg, 0.016f);

    float viol = ac.violation(reg);
    EXPECT_LE(viol, 0.05f) << "角度约束应把超出限位的夹角拉回";
}
