// Collider 单元测试: 点 vs AABB 碰撞修正
#include <gtest/gtest.h>
#include <glm/glm.hpp>
#include "physics/Math.h"

using namespace leo;

TEST(PointVsAABB, NoHitWhenOutside) {
    AABB box{{-1,-1,-1}, {1,1,1}};
    Contact c = pointVsAABB({5,0,0}, box);
    EXPECT_FALSE(c.hit);
}

TEST(PointVsAABB, HitWhenInside) {
    AABB box{{-1,-1,-1}, {1,1,1}};
    Contact c = pointVsAABB({0,0,0}, box);  // 正中心
    EXPECT_TRUE(c.hit);
    EXPECT_GT(c.penetration, 0.0f);
}

TEST(PointVsAABB, PushesOutAlongNearestFace) {
    AABB box{{-1,-1,-1}, {1,1,1}};
    // 点靠近 +X 面 (x=0.9), 离 +X 面距离 0.1, 应被往 +X 推
    Contact c = pointVsAABB({0.9f, 0, 0}, box);
    EXPECT_TRUE(c.hit);
    EXPECT_NEAR(c.penetration, 0.1f, 1e-5f);
    EXPECT_NEAR(c.normal.x, 1.0f, 1e-5f);
    EXPECT_NEAR(c.normal.y, 0.0f, 1e-5f);
    EXPECT_NEAR(c.normal.z, 0.0f, 1e-5f);

    // 修正后点应在 box 外
    glm::vec3 pushed = glm::vec3(0.9f, 0, 0) + c.normal * c.penetration;
    EXPECT_GT(pushed.x, box.hi.x - 1e-5f) << "修正后应推出 box";
}

TEST(PointVsAABB, PushesUpWhenNearTopFace) {
    AABB box{{-1,-1,-1}, {1,1,1}};
    // 点靠近 +Y 面 (y=0.8), 应被往 +Y 推 (地面碰撞场景)
    Contact c = pointVsAABB({0, 0.8f, 0}, box);
    EXPECT_TRUE(c.hit);
    EXPECT_NEAR(c.normal.y, 1.0f, 1e-5f);
    EXPECT_NEAR(c.penetration, 0.2f, 1e-5f);
}

TEST(PointVsAABB, CornerPicksAxisWithMinPenetration) {
    AABB box{{0,0,0}, {2,2,2}};
    // 点在 (0.1, 0.1, 0.1) 角落, 三轴距离都是 0.1, 应选其中一个 (实现选 -X)
    Contact c = pointVsAABB({0.1f, 0.1f, 0.1f}, box);
    EXPECT_TRUE(c.hit);
    EXPECT_NEAR(c.penetration, 0.1f, 1e-5f);
    // 法线应是某个轴方向, 长度 1
    EXPECT_NEAR(glm::length(c.normal), 1.0f, 1e-5f);
}
