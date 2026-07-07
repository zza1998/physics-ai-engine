#pragma once

#include <string>
struct GLFWwindow;

namespace leo {

class Window;
class Camera;
struct PhysicsConfig;

// ImGui 调试面板: 实时调物理参数 + 性能指标 + debug 显示开关
class ImGuiLayer {
public:
    ImGuiLayer();
    ~ImGuiLayer();

    bool init(Window& window);   // ImGui GLFW + OpenGL3 backend init
    void shutdown();

    // 每帧: newFrame -> drawXxx -> render
    void newFrame();
    void render();

    // ImGui 是否捕获鼠标/键盘 (Camera/Input 应跳过)
    bool wantCaptureMouse() const;
    bool wantCaptureKeyboard() const;

    // 调参面板: 直接改 PhysicsConfig 引用
    void drawPhysicsPanel(PhysicsConfig& cfg, int boxCount);
    // 性能面板
    void drawPerfPanel(float fps, float physicsMs);
    // 帮助面板 (操作说明)
    void drawHelpPanel();

private:
    bool m_initialized = false;
};

} // namespace leo
