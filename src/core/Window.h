#pragma once

#include <string>
// glad 必须在 GLFW 之前: GLFW 的头会触发系统 GL 头, glad2 要求自己先加载
#include <glad/gl.h>
#include <GLFW/glfw3.h>
#include "Input.h"

namespace leo {

struct WindowConfig {
    std::string title = "Leo Physics Engine";
    int width  = 1280;
    int height = 720;
    bool vsync  = true;
    int gl_major = 4;
    int gl_minor = 6;
};

class Window {
public:
    Window(const WindowConfig& cfg, Input& input);
    ~Window();

    bool init();              // glfwInit + hints + createWindow + makeContextCurrent
    void shutdown();

    void swapBuffers();
    void pollEvents();
    bool shouldClose() const;
    void setShouldClose(bool v);

    int width() const { return m_width; }
    int height() const { return m_height; }
    float aspect() const { return (m_height == 0) ? 1.0f : (float)m_width / (float)m_height; }
    GLFWwindow* handle() const { return m_window; }

private:
    // 静态 trampoline -> 实例方法 -> 写入 Input
    static void keyCallback(GLFWwindow* w, int k, int s, int a, int m);
    static void mouseButtonCallback(GLFWwindow* w, int b, int a, int m);
    static void cursorPosCallback(GLFWwindow* w, double x, double y);
    static void scrollCallback(GLFWwindow* w, double x, double y);
    static void framebufferSizeCallback(GLFWwindow* w, int width, int height);

    WindowConfig m_cfg;
    GLFWwindow*  m_window = nullptr;
    Input&       m_input;
    int          m_width;
    int          m_height;
};

} // namespace leo
