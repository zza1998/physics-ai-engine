#include "core/Application.h"
#include "core/scene/Scene.h"
#include "physics/PhysicsSystem.h"
#include <glm/glm.hpp>
#include <cstdio>

using namespace leo;

int main() {
    ApplicationConfig cfg;
    cfg.title  = "Leo Physics Engine - M2 (点云物理)";
    cfg.width  = 1280;
    cfg.height = 720;
    Application app(cfg);
    if (!app.init()) {
        std::fprintf(stderr, "Application init failed\n");
        return 1;
    }

    // Application::init 已自动: 地面 + StaticBody + PhysicsSystem + DebugRenderer
    // demo 操作:
    //   F     - 在相机前方生成点云箱子掉落
    //   WASD  - 移动 / Q E - 上下 / 鼠标 - 转视角
    //   1/2/3 - 切换 debug 显示 (点/线框/约束)  [M2 Week3 ImGui 接入后更方便]
    //   ESC   - 退出

    // 预生成几个静态平台让箱子堆叠有层次
    if (auto* phys = app.physics()) {
        Mesh* cube = nullptr; Shader* sh = nullptr;
        // 通过 scene 默认 mesh/shader (addCube 用的) 取出
        Scene& scene = app.scene();
        cube = scene.cubeMesh();
        sh = scene.defaultShader();
        // 两个台阶式平台
        phys->spawnStaticBox(scene, {4, 0.5f, 0}, {3, 1, 3}, cube, sh, {0.3f, 0.3f, 0.32f});
        phys->spawnStaticBox(scene, {7, 1.5f, 0}, {3, 1, 3}, cube, sh, {0.3f, 0.3f, 0.32f});
        // 预生成几个动态箱子 (掉落堆叠, 截图可见物理在动)
        phys->spawnBox(scene, {0, 5, 0}, {0.9f, 0.9f, 0.9f}, cube, sh, {0.9f, 0.4f, 0.2f});
        phys->spawnBox(scene, {0.5f, 7, 0.3f}, {0.9f, 0.9f, 0.9f}, cube, sh, {0.2f, 0.5f, 0.9f});
        phys->spawnBox(scene, {-0.3f, 9, -0.2f}, {0.9f, 0.9f, 0.9f}, cube, sh, {0.9f, 0.9f, 0.2f});
        phys->spawnBox(scene, {0.2f, 11, 0.1f}, {0.9f, 0.9f, 0.9f}, cube, sh, {0.5f, 0.2f, 0.9f});
        // 平台也加 Renderable (spawnStaticBox 已加)
    }

    app.run();
    app.shutdown();
    return 0;
}
