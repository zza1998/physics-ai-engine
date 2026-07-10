#pragma once

#include <array>
#include <tuple>
#include <utility>
#include <glm/glm.hpp>
#include <entt/entt.hpp>

namespace leo {

// 17 点人形 ragdoll 点索引
enum RagdollPoint : int {
    Head = 0,       // 0 头
    Neck,           // 1 颈
    TorsoUp,        // 2 躯干上
    TorsoDown,      // 3 躯干下
    Pelvis,         // 4 骨盆
    LUpperArm,      // 5 左上臂
    LForeArm,       // 6 左前臂
    LHand,          // 7 左手
    RUpperArm,      // 8 右上臂
    RForeArm,       // 9 右前臂
    RHand,          // 10 右手
    LThigh,         // 11 左大腿
    LShin,          // 12 左小腿
    LFoot,          // 13 左脚
    RThigh,         // 14 右大腿
    RShin,          // 15 右小腿
    RFoot,          // 16 右脚
    Count           // 17
};

// ragdoll 局部坐标 (站立, 头朝 +Y, 单位米, scale 缩放)
// 索引同 RagdollPoint
const glm::vec3 kRagdollLocalPos[17] = {
    {0.00f, 1.70f, 0.00f},  // 头
    {0.00f, 1.50f, 0.00f},  // 颈
    {0.00f, 1.30f, 0.00f},  // 躯干上
    {0.00f, 0.90f, 0.00f},  // 躯干下
    {0.00f, 0.60f, 0.00f},  // 骨盆
    {0.25f, 1.25f, 0.00f},  // 左上臂
    {0.45f, 0.95f, 0.00f},  // 左前臂
    {0.55f, 0.70f, 0.00f},  // 左手
    {-0.25f, 1.25f, 0.00f}, // 右上臂
    {-0.45f, 0.95f, 0.00f}, // 右前臂
    {-0.55f, 0.70f, 0.00f}, // 右手
    {0.15f, 0.35f, 0.00f},  // 左大腿
    {0.15f, -0.10f, 0.00f}, // 左小腿
    {0.15f, -0.40f, 0.10f}, // 左脚
    {-0.15f, 0.35f, 0.00f}, // 右大腿
    {-0.15f, -0.10f, 0.00f},// 右小腿
    {-0.15f, -0.40f, 0.10f} // 右脚
};

// 每点的反质量 (越小越重)
const float kRagdollInvMass[17] = {
    1.0f,  // 头
    1.0f,  // 颈
    0.8f,  // 躯干上
    0.8f,  // 躯干下
    0.5f,  // 骨盆 (最重)
    1.2f,  // 左上臂
    1.5f,  // 左前臂
    2.0f,  // 左手 (最轻)
    1.2f,  // 右上臂
    1.5f,  // 右前臂
    2.0f,  // 右手
    1.2f,  // 左大腿
    1.5f,  // 左小腿
    2.0f,  // 左脚
    1.2f,  // 右大腿
    1.5f,  // 右小腿
    2.0f   // 右脚
};

// 骨骼连接 (距离约束): {a, b}
const std::array<std::pair<int,int>, 16> kRagdollBones = {{
    {Head, Neck},          {Neck, TorsoUp},      {TorsoUp, TorsoDown}, {TorsoDown, Pelvis},
    {TorsoUp, LUpperArm},  {LUpperArm, LForeArm}, {LForeArm, LHand},
    {TorsoUp, RUpperArm},  {RUpperArm, RForeArm}, {RForeArm, RHand},
    {Pelvis, LThigh},      {LThigh, LShin},       {LShin, LFoot},
    {Pelvis, RThigh},      {RThigh, RShin},       {RShin, RFoot}
}};

// 抗塌陷约束 (额外距离约束, 保躯干/肩/髋不塌)
const std::array<std::pair<int,int>, 3> kRagdollBrace = {{
    {TorsoUp, Pelvis},      // 躯干对角
    {LUpperArm, RUpperArm}, // 肩宽
    {LThigh, RThigh}        // 髋宽
}};

// 关节角度约束 (三点 a-b-c, 限制 b 处夹角): {a, b, c, min_deg, max_deg}
const std::array<std::tuple<int,int,int,float,float>, 6> kRagdollJoints = {{
    {Head,      Neck,      TorsoUp,   30.0f, 150.0f},  // 颈
    {TorsoUp,   TorsoDown, Pelvis,    60.0f, 180.0f},  // 躯干
    {TorsoUp,   LUpperArm, LForeArm,  30.0f, 150.0f},  // 左肘
    {TorsoUp,   RUpperArm, RForeArm,  30.0f, 150.0f},  // 右肘
    {Pelvis,    LThigh,    LShin,     30.0f, 150.0f},  // 左膝
    {Pelvis,    RThigh,    RShin,     30.0f, 150.0f}   // 右膝
}};

// collision_group 分配: 相邻骨同组防互碰. 这里简化: 头颈同组1, 躯干同组2, 左臂同组3, 右臂同组4, 左腿同组5, 右腿同组6
const int kRagdollGroup[17] = {
    1, 1,    // 头 颈
    2, 2, 2, // 躯干上 躯干下 骨盆
    3, 3, 3, // 左臂
    4, 4, 4, // 右臂
    5, 5, 5, // 左腿
    6, 6, 6  // 右腿
};

} // namespace leo
