#pragma once

#include "graphics/Shader.h"
#include "graphics/Mesh.h"

namespace leo {

class Scene;
class Camera;

// 物理调试渲染: 点云点 + Collider AABB 线框 + 距离约束连线
// 复用 camera 矩阵, 用纯色 debug shader
class DebugRenderer {
public:
    DebugRenderer();
    ~DebugRenderer();

    bool init(Shader* debugShader);  // shader 非拥有
    void shutdown();

    // flags 由 PhysicsConfig::debug_draw_* 控制 (调用方传入)
    void render(Scene& scene, Camera& camera, float aspect,
                bool drawPoints, bool drawColliders, bool drawConstraints);

private:
    Shader* m_shader = nullptr;  // 非拥有
    Mesh m_pointMesh;   // GL_POINTS, dynamic
    Mesh m_lineMesh;    // GL_LINES, dynamic
};

} // namespace leo
