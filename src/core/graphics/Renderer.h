#pragma once

#include <glm/glm.hpp>
#include "graphics/Shader.h"
#include "graphics/Mesh.h"

namespace leo {

class Scene;
class Camera;
struct Light;

class Renderer {
public:
    Renderer();
    bool init();  // depth test, cull face, clear color

    void setViewport(int x, int y, int w, int h);

    // 遍历场景: 取第一个 Directional Light + 所有 Renderable 实体绘制
    // aspect = 窗口宽/高, 用于透视投影
    void renderScene(Scene& scene, Camera& camera, float aspect);

    // 最小 draw call 单元 (M2 debug 渲染点云/约束线复用)
    // shader 非 const: setXxx 会写 uniform cache
    void draw(Shader& shader, const Mesh& mesh, const glm::mat4& model);

private:
    void applyLight(Shader& shader, const Light& light, const glm::vec3& cameraPos);
};

} // namespace leo
