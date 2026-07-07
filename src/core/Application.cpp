#include <glad/gl.h>
#include <GLFW/glfw3.h>
#include "Application.h"
#include "Logger.h"
#include "graphics/MeshPrimitives.h"
#include <string>

namespace leo {

Application::Application(const ApplicationConfig& config) : m_config(config) {}

Application::~Application() {
    shutdown();
}

bool Application::init() {
    // 1. Input 必须先于 Window (Window 持有 Input 引用)
    m_input = std::make_unique<Input>();

    // 2. Window: glfwInit + 创建 4.6 core context + makeContextCurrent
    m_window = std::make_unique<Window>(
        WindowConfig{m_config.title, m_config.width, m_config.height, m_config.vsync, 4, 6},
        *m_input);
    if (!m_window->init()) {
        LEO_LOG_ERROR("Window init failed");
        return false;
    }

    // 3. 加载 glad: 必须在 makeContextCurrent 之后
    if (!gladLoadGL(glfwGetProcAddress)) {
        LEO_LOG_ERROR("gladLoadGL failed - GL loader init error");
        return false;
    }
    LEO_LOG_INFO("OpenGL context ready");

    // 4. Renderer
    m_renderer = std::make_unique<Renderer>();
    m_renderer->init();
    m_renderer->setViewport(0, 0, m_window->width(), m_window->height());

    // 5. Shader + Mesh
    m_shader = std::make_unique<Shader>();
    if (!m_shader->loadFromFile("assets/shaders/blinn_phong.vert",
                                "assets/shaders/blinn_phong.frag")) {
        LEO_LOG_ERROR("Shader load failed");
        return false;
    }
    m_debugShader = std::make_unique<Shader>();
    if (!m_debugShader->loadFromFile("assets/shaders/debug.vert",
                                     "assets/shaders/debug.frag")) {
        LEO_LOG_ERROR("Debug shader load failed");
        return false;
    }
    m_cubeMesh  = std::make_unique<Mesh>(makeCube());
    m_planeMesh = std::make_unique<Mesh>(makePlane(40.0f, 40.0f));

    // 6. Scene (注入默认 mesh/shader 给 addCube/addPlane 用)
    m_scene = std::make_unique<Scene>();
    m_scene->setCubeMesh(m_cubeMesh.get());
    m_scene->setPlaneMesh(m_planeMesh.get());
    m_scene->setDefaultShader(m_shader.get());

    // 6b. 地面平面 + StaticBody (大 AABB, 顶面 y=0)
    auto ground = m_scene->addPlane(40.0f, 40.0f, {0.4f, 0.4f, 0.42f});
    m_scene->registry().emplace<StaticBody>(ground,
        StaticBody{AABB{glm::vec3(-20, -1, -20), glm::vec3(20, 0, 20)}, 0.5f});

    // 6b2. 方向光 (Blinn-Phong 需 uLightColor/uLightDir/uViewPos, 否则 surface 全黑)
    //      方向从右上往下打, 让地面/台阶顶面被照亮.
    m_scene->addDirectionalLight({-0.3f, -1.0f, -0.4f}, {1.0f, 1.0f, 1.0f});

    // 6c. DebugRenderer
    m_debugRenderer = std::make_unique<DebugRenderer>();
    m_debugRenderer->init(m_debugShader.get());

    // 6d. PhysicsSystem 注册
    auto phys = std::make_unique<PhysicsSystem>(PhysicsConfig{});
    m_physics = phys.get();  // 裸指针便于 spawnBox/ImGui 访问
    addSystem(std::move(phys));

    // 6e. ImGui 调试面板
    m_imgui = std::make_unique<ImGuiLayer>();
    if (!m_imgui->init(*m_window)) {
        LEO_LOG_ERROR("ImGui init failed");
        return false;
    }

    // 7. Camera (抬高一点, 俯视地面)
    CameraConfig camCfg;
    camCfg.position = {0, 6, 12};
    camCfg.pitch = -25.0f;
    m_camera = std::make_unique<Camera>(camCfg);

    // 8. 锁定鼠标到窗口 (flycam), 隐藏光标, 移动量作为视角 delta
    //    F1 切换: 锁定时 flycam, 释放时点 ImGui
    glfwSetInputMode(m_window->handle(), GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    m_cursorLocked = true;

    return true;
}

void Application::addSystem(std::unique_ptr<System> s) {
    m_systems.push_back(std::move(s));
}

void Application::run() {
    m_running = true;
    m_lastTime = (float)glfwGetTime();

    while (!m_window->shouldClose() && m_running) {
        float now = (float)glfwGetTime();
        float dt = now - m_lastTime;
        m_lastTime = now;
        m_fps = (dt > 0) ? 1.0f / dt : 0.0f;

        m_window->pollEvents();

        if (m_input->isKeyDown(GLFW_KEY_ESCAPE)) m_running = false;

        // F1 切换鼠标锁定 (flycam 锁 <-> ImGui 释放). edge detect
        static bool f1Held = false;
        bool f1Now = m_input->isKeyDown(GLFW_KEY_F1);
        if (f1Now && !f1Held) {
            m_cursorLocked = !m_cursorLocked;
            glfwSetInputMode(m_window->handle(), GLFW_CURSOR,
                             m_cursorLocked ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);
        }
        f1Held = f1Now;

        // ImGui 捕获键盘时, 跳过 demo 快捷键 (F/G), 避免输入冲突
        bool imguiWantKb = m_imgui && m_imgui->wantCaptureKeyboard();

        // G 键切换 debug 渲染 (点云/线框/约束)
        static bool gHeld = false;
        bool gNow = m_input->isKeyDown(GLFW_KEY_G);
        if (gNow && !gHeld && m_physics && !imguiWantKb) {
            PhysicsConfig& cfg = m_physics->config();
            bool allOn = cfg.debug_draw_points && cfg.debug_draw_colliders && cfg.debug_draw_constraints;
            cfg.debug_draw_points = cfg.debug_draw_colliders = cfg.debug_draw_constraints = !allOn;
        }
        gHeld = gNow;

        // demo: 按 F 键 (edge detect) 在相机前方生成一个点云箱子掉落
        static bool fHeld = false;
        bool fNow = m_input->isKeyDown(GLFW_KEY_F);
        if (fNow && !fHeld && m_physics && !imguiWantKb) {
            glm::vec3 spawnPos = m_camera->position() + m_camera->front() * 5.0f;
            spawnPos.y += 2.0f;
            float s = 0.9f;  // 固定尺寸 (引擎内禁用随机, demo 用固定值)
            m_physics->spawnBox(*m_scene, spawnPos, glm::vec3(s, s, s),
                                m_cubeMesh.get(), m_shader.get());
            m_boxCount++;
            LEO_LOG_INFO("F pressed: spawned box #" + std::to_string(m_boxCount) +
                         " at (" + std::to_string(spawnPos.x) + "," +
                         std::to_string(spawnPos.y) + "," + std::to_string(spawnPos.z) + ")");
        }
        fHeld = fNow;

        update(dt);
        render();

        // ImGui: newFrame -> draw panels -> render (在场景渲染之后, 叠加在最上层)
        if (m_imgui) {
            m_imgui->newFrame();
            if (m_physics) {
                m_imgui->drawPhysicsPanel(m_physics->config(), m_boxCount);
            }
            m_imgui->drawPerfPanel(m_fps, m_physicsMs);
            m_imgui->render();
        }

        m_window->swapBuffers();
        m_input->endFrame();
    }
}

void Application::update(float dt) {
    // ImGui 捕获鼠标或鼠标未锁定时, 跳过 flycam (让用户点 ImGui)
    bool imguiWantMouse = m_imgui && m_imgui->wantCaptureMouse();
    if (m_cursorLocked && !imguiWantMouse) {
        m_camera->update(dt, *m_input);
    }

    // 更新所有 system (PhysicsSystem 在此调用, 注册顺序保证物理先于渲染)
    float physStart = (float)glfwGetTime();
    for (auto& s : m_systems) {
        s->update(dt, *m_scene);
    }
    m_physicsMs = ((float)glfwGetTime() - physStart) * 1000.0f;
}

void Application::render() {
    m_renderer->setViewport(0, 0, m_window->width(), m_window->height());
    m_renderer->renderScene(*m_scene, *m_camera, m_window->aspect());

    // M2 debug 渲染: 点云 + AABB 线框 + 约束线
    if (m_debugRenderer && m_physics) {
        const PhysicsConfig& cfg = m_physics->config();
        m_debugRenderer->render(*m_scene, *m_camera, m_window->aspect(),
                                cfg.debug_draw_points,
                                cfg.debug_draw_colliders,
                                cfg.debug_draw_constraints);
    }
}

void Application::close() {
    m_running = false;
}

void Application::shutdown() {
    // GL 资源需在 context 销毁前释放. 顺序: ImGui/systems/debug -> 资源 -> window
    if (m_imgui) { m_imgui->shutdown(); m_imgui.reset(); }
    m_systems.clear();
    m_physics = nullptr;
    if (m_debugRenderer) { m_debugRenderer->shutdown(); m_debugRenderer.reset(); }
    m_camera.reset();
    m_scene.reset();
    m_planeMesh.reset();
    m_cubeMesh.reset();
    m_debugShader.reset();
    m_shader.reset();
    m_renderer.reset();
    m_window.reset();   // Window 析构里 glfwDestroyWindow + glfwTerminate
    m_input.reset();
}

} // namespace leo
