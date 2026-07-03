#include "core/Application.h"
#include "core/scene/Scene.h"
#include <glm/glm.hpp>

using namespace leo;

int main() {
    ApplicationConfig cfg;
    cfg.title  = "Leo Physics Engine - M1";
    cfg.width  = 1280;
    cfg.height = 720;
    Application app(cfg);
    if (!app.init()) {
        return 1;
    }

    Scene& scene = app.scene();

    // 地面: 20x20 灰色平面, y=0
    scene.addPlane(20.0f, 20.0f, {0.5f, 0.5f, 0.5f});

    // 三个彩色立方体立在平面上 (立方体边长 1, 居中, 所以 y=0.5 贴地)
    scene.addCube({ 0.0f, 0.5f,  0.0f}, {1,1,1}, {0.85f, 0.30f, 0.20f}); // 红橙
    scene.addCube({ 2.0f, 0.5f,  0.0f}, {1,1,1}, {0.20f, 0.60f, 0.90f}); // 蓝
    scene.addCube({-2.0f, 0.5f, -1.0f}, {1,1,1}, {0.95f, 0.90f, 0.20f}); // 黄

    // 方向光
    scene.addDirectionalLight({-0.2f, -1.0f, -0.3f}, {1.0f, 1.0f, 1.0f});

    app.run();
    app.shutdown();
    return 0;
}
