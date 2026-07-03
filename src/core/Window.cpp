#include <glad/gl.h>
#include "Window.h"
#include "Logger.h"

namespace leo {

Window::Window(const WindowConfig& cfg, Input& input)
    : m_cfg(cfg), m_input(input), m_width(cfg.width), m_height(cfg.height) {}

Window::~Window() {
    shutdown();
}

bool Window::init() {
    if (!glfwInit()) {
        LEO_LOG_ERROR("glfwInit failed");
        return false;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, m_cfg.gl_major);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, m_cfg.gl_minor);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);

    m_window = glfwCreateWindow(m_cfg.width, m_cfg.height, m_cfg.title.c_str(),
                                nullptr, nullptr);
    if (!m_window) {
        LEO_LOG_ERROR("glfwCreateWindow failed - OpenGL 4.6 core not supported?");
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(m_window);
    glfwSwapInterval(m_cfg.vsync ? 1 : 0);

    // 注册 user pointer + 回调
    glfwSetWindowUserPointer(m_window, this);
    glfwSetKeyCallback(m_window, keyCallback);
    glfwSetMouseButtonCallback(m_window, mouseButtonCallback);
    glfwSetCursorPosCallback(m_window, cursorPosCallback);
    glfwSetScrollCallback(m_window, scrollCallback);
    glfwSetFramebufferSizeCallback(m_window, framebufferSizeCallback);

    // 缓存实际尺寸 (用户可能拖动, 但 init 时取一次)
    glfwGetFramebufferSize(m_window, &m_width, &m_height);
    return true;
}

void Window::shutdown() {
    if (m_window) {
        glfwDestroyWindow(m_window);
        m_window = nullptr;
        glfwTerminate();
    }
}

void Window::swapBuffers() { glfwSwapBuffers(m_window); }
void Window::pollEvents()  { glfwPollEvents(); }
bool Window::shouldClose() const { return glfwWindowShouldClose(m_window); }
void Window::setShouldClose(bool v) { glfwSetWindowShouldClose(m_window, v ? GLFW_TRUE : GLFW_FALSE); }

// ---- 静态 trampoline ----
void Window::keyCallback(GLFWwindow* w, int k, int s, int a, int m) {
    auto* self = static_cast<Window*>(glfwGetWindowUserPointer(w));
    self->m_input.onKey(k, s, a, m);
}
void Window::mouseButtonCallback(GLFWwindow* w, int b, int a, int m) {
    auto* self = static_cast<Window*>(glfwGetWindowUserPointer(w));
    self->m_input.onMouseButton(b, a, m);
}
void Window::cursorPosCallback(GLFWwindow* w, double x, double y) {
    auto* self = static_cast<Window*>(glfwGetWindowUserPointer(w));
    self->m_input.onCursorPos(x, y);
}
void Window::scrollCallback(GLFWwindow* w, double x, double y) {
    auto* self = static_cast<Window*>(glfwGetWindowUserPointer(w));
    self->m_input.onScroll(x, y);
}
void Window::framebufferSizeCallback(GLFWwindow* w, int width, int height) {
    auto* self = static_cast<Window*>(glfwGetWindowUserPointer(w));
    self->m_width = width;
    self->m_height = height;
    glViewport(0, 0, width, height); // glad 已加载, 此回调在 init 后触发
}

} // namespace leo
