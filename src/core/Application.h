#pragma once

#include <memory>
#include <vector>
#include <string>
#include "core/Window.h"
#include "core/Input.h"
#include "core/Camera.h"
#include "graphics/Renderer.h"
#include "graphics/Shader.h"
#include "graphics/Mesh.h"
#include "scene/Scene.h"
#include "scene/System.h"

namespace leo {

struct ApplicationConfig {
    std::string title = "Leo Physics Engine";
    int width  = 1280;
    int height = 720;
    bool vsync  = true;
};

class Application {
public:
    Application(const ApplicationConfig& config);
    ~Application();

    bool init();     // 创建 Window + 加载 glad + 创建 Renderer/Shader/Mesh/Scene/Camera
    void run();      // 主循环
    void close();    // 设置 m_running = false
    void shutdown(); // 释放资源

    Window&   window()   { return *m_window; }
    Input&    input()    { return *m_input; }
    Camera&   camera()   { return *m_camera; }
    Renderer& renderer() { return *m_renderer; }
    Scene&    scene()    { return *m_scene; }

    // M2 物理系统挂点: app.addSystem(make_unique<PhysicsSystem>())
    void addSystem(std::unique_ptr<System> s);

private:
    void update(float dt);
    void render();

    ApplicationConfig m_config;

    std::unique_ptr<Input>    m_input;
    std::unique_ptr<Window>   m_window;
    std::unique_ptr<Renderer> m_renderer;
    std::unique_ptr<Shader>   m_shader;
    std::unique_ptr<Mesh>     m_cubeMesh;
    std::unique_ptr<Mesh>     m_planeMesh;
    std::unique_ptr<Scene>    m_scene;
    std::unique_ptr<Camera>   m_camera;
    std::vector<std::unique_ptr<System>> m_systems;

    bool  m_running = false;
    float m_lastTime = 0.0f;
};

} // namespace leo
