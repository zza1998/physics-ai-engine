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
#include "graphics/DebugRenderer.h"
#include "ui/ImGuiLayer.h"
#include "scene/Scene.h"
#include "scene/System.h"
#include "physics/PhysicsSystem.h"

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

    bool init();     // 创建 Window + 加载 glad + 创建 Renderer/Shader/Mesh/Scene/Camera + 注册 PhysicsSystem
    void run();      // 主循环
    void close();    // 设置 m_running = false
    void shutdown(); // 释放资源

    Window&   window()   { return *m_window; }
    Input&    input()    { return *m_input; }
    Camera&   camera()   { return *m_camera; }
    Renderer& renderer() { return *m_renderer; }
    Scene&    scene()    { return *m_scene; }
    PhysicsSystem* physics() { return m_physics; }  // 可能为 nullptr

    // M2 物理系统挂点: app.addSystem(make_unique<PhysicsSystem>())
    void addSystem(std::unique_ptr<System> s);

private:
    void update(float dt);
    void render();

    ApplicationConfig m_config;

    std::unique_ptr<Input>    m_input;
    std::unique_ptr<Window>   m_window;
    std::unique_ptr<Renderer> m_renderer;
    std::unique_ptr<Shader>   m_shader;       // blinn_phong
    std::unique_ptr<Shader>   m_debugShader;  // 纯色 debug (M2)
    std::unique_ptr<Mesh>     m_cubeMesh;
    std::unique_ptr<Mesh>     m_planeMesh;
    std::unique_ptr<DebugRenderer> m_debugRenderer;  // M2 点云/线框/约束
    std::unique_ptr<ImGuiLayer>    m_imgui;          // M2 调试面板
    std::unique_ptr<Scene>    m_scene;
    std::unique_ptr<Camera>   m_camera;
    std::vector<std::unique_ptr<System>> m_systems;
    PhysicsSystem* m_physics = nullptr;  // 非拥有, 指向 m_systems 里的 PhysicsSystem

    // demo: 鼠标点击生成箱子计数 + 性能计时
    int   m_boxCount = 0;
    float m_physicsMs = 0.0f;  // 上一帧物理耗时 (ImGui 用, M3)
    float m_fps = 0.0f;

    bool  m_running = false;
    bool  m_cursorLocked = true;  // flycam 锁鼠标, F1 释放点 ImGui
    float m_lastTime = 0.0f;
};

} // namespace leo
