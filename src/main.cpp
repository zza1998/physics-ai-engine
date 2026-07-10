#include "core/Application.h"
#include "core/scene/Scene.h"
#include "physics/PhysicsSystem.h"
#include <glm/glm.hpp>
#include <cstdio>

using namespace leo;

int main() {
    ApplicationConfig cfg;
    cfg.title  = "Leo Physics Engine - M3 (ragdoll)";
    cfg.width  = 1280;
    cfg.height = 720;
    Application app(cfg);
    if (!app.init()) {
        std::fprintf(stderr, "Application init failed\n");
        return 1;
    }

    // Application::init 已自动: 地面 + StaticBody + PhysicsSystem + DebugRenderer
    // demo 操作:
    //   F     - 在相机前方生成 RB 箱子掉落
    //   WASD  - 移动 / Q E - 上下 / 鼠标 - 转视角
    //   G     - 切换 debug 显示 / F1 - 切鼠标 / ESC - 退出

    // 预生成几个静态平台 + 动态箱子 (M2)
    if (auto* phys = app.physics()) {
        Scene& scene = app.scene();
        Mesh* cube = scene.cubeMesh();
        Shader* sh = scene.defaultShader();
        phys->spawnStaticBox(scene, {4, 0.5f, 0}, {3, 1, 3}, cube, sh, {0.3f, 0.3f, 0.32f});
        phys->spawnStaticBox(scene, {7, 1.5f, 0}, {3, 1, 3}, cube, sh, {0.3f, 0.3f, 0.32f});
        phys->spawnBox(scene, {-3, 5, 0}, {0.9f, 0.9f, 0.9f}, cube, sh, {0.9f, 0.4f, 0.2f});
        phys->spawnBox(scene, {-3.5f, 7, 0.3f}, {0.9f, 0.9f, 0.9f}, cube, sh, {0.2f, 0.5f, 0.9f});

        // M3a: 一个 ragdoll 从空中掉落 (骨盆 y=5, 站立姿态)
        phys->spawnRagdoll(scene, {0, 5, 0}, 1.0f);
    }

    app.run();
    app.shutdown();
    return 0;
}
