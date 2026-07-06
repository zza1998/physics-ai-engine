#include <glad/gl.h>
#include <GLFW/glfw3.h>
#include "Application.h"
#include "Logger.h"
#include "graphics/MeshPrimitives.h"

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
    m_cubeMesh  = std::make_unique<Mesh>(makeCube());
    m_planeMesh = std::make_unique<Mesh>(makePlane(20.0f, 20.0f));

    // 6. Scene (注入默认 mesh/shader 给 addCube/addPlane 用)
    m_scene = std::make_unique<Scene>();
    m_scene->setCubeMesh(m_cubeMesh.get());
    m_scene->setPlaneMesh(m_planeMesh.get());
    m_scene->setDefaultShader(m_shader.get());

    // 7. Camera
    m_camera = std::make_unique<Camera>(CameraConfig{});

    // 8. 锁定鼠标到窗口 (flycam), 隐藏光标, 移动量作为视角 delta
    // ESC 退出程序, 无需单独解锁
    glfwSetInputMode(m_window->handle(), GLFW_CURSOR, GLFW_CURSOR_DISABLED);

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

        m_window->pollEvents();

        if (m_input->isKeyDown(GLFW_KEY_ESCAPE)) m_running = false;

        update(dt);
        render();

        m_window->swapBuffers();
        m_input->endFrame();
    }
}

void Application::update(float dt) {
    // 更新摄像机 (flycam)
    m_camera->update(dt, *m_input);

    // 更新所有 system (M2 PhysicsSystem 在此调用, 注册顺序保证物理先于渲染)
    for (auto& s : m_systems) {
        s->update(dt, *m_scene);
    }
}

void Application::render() {
    m_renderer->setViewport(0, 0, m_window->width(), m_window->height());
    m_renderer->renderScene(*m_scene, *m_camera, m_window->aspect());
}

void Application::close() {
    m_running = false;
}

void Application::shutdown() {
    // Mesh/Shader 等 GL 资源在 unique_ptr 析构时释放; 但需 context 仍有效
    // 先释放资源, 再销毁 window
    m_systems.clear();
    m_camera.reset();
    m_scene.reset();
    m_planeMesh.reset();
    m_cubeMesh.reset();
    m_shader.reset();
    m_renderer.reset();
    m_window.reset();   // Window 析构里 glfwDestroyWindow + glfwTerminate
    m_input.reset();
}

} // namespace leo
