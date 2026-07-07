#include "ui/ImGuiLayer.h"
#include "Window.h"
#include "physics/PhysicsSystem.h"
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <glad/gl.h>
#include <GLFW/glfw3.h>
#include <cstdio>

namespace leo {

ImGuiLayer::ImGuiLayer() = default;
ImGuiLayer::~ImGuiLayer() { shutdown(); }

bool ImGuiLayer::init(Window& window) {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    (void)io;

    ImGui::StyleColorsDark();
    // 字体稍大 + 窗口圆角
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 4.0f;
    style.FrameRounding = 3.0f;

    if (!ImGui_ImplGlfw_InitForOpenGL(window.handle(), true)) {
        std::fprintf(stderr, "ImGui_ImplGlfw_InitForOpenGL failed\n");
        return false;
    }
    // glsl_version 字符串 (OpenGL 4.6 core)
    if (!ImGui_ImplOpenGL3_Init("#version 460")) {
        std::fprintf(stderr, "ImGui_ImplOpenGL3_Init failed\n");
        return false;
    }
    m_initialized = true;
    return true;
}

void ImGuiLayer::shutdown() {
    if (!m_initialized) return;
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    m_initialized = false;
}

void ImGuiLayer::newFrame() {
    if (!m_initialized) return;
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}

void ImGuiLayer::render() {
    if (!m_initialized) return;
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

bool ImGuiLayer::wantCaptureMouse() const {
    return m_initialized && ImGui::GetIO().WantCaptureMouse;
}
bool ImGuiLayer::wantCaptureKeyboard() const {
    // 只在有文本输入框聚焦时才捕获键盘 (WantTextInput),
    // 而非 WantCaptureKeyboard (后者在窗口聚焦时常驻 true, 会误拦 F/G 快捷键)
    return m_initialized && ImGui::GetIO().WantTextInput;
}

void ImGuiLayer::drawPhysicsPanel(PhysicsConfig& cfg, int boxCount) {
    if (!ImGui::Begin("Physics Debug")) { ImGui::End(); return; }
    ImGui::Text("Boxes: %d", boxCount);
    ImGui::Separator();

    ImGui::Text("Gravity");
    ImGui::SliderFloat3("g", &cfg.gravity[0], -20.0f, 20.0f, "%.2f");
    ImGui::Text("Damping");
    ImGui::SliderFloat("damping", &cfg.damping, 0.90f, 1.00f, "%.4f");
    ImGui::Text("Solver");
    ImGui::SliderInt("substeps", &cfg.substeps, 1, 16);
    ImGui::SliderInt("iterations", &cfg.iterations, 1, 32);
    ImGui::Separator();
    ImGui::Text("Debug Draw");
    ImGui::Checkbox("points (green)", &cfg.debug_draw_points);
    ImGui::Checkbox("AABB wire (cyan)", &cfg.debug_draw_colliders);
    ImGui::Checkbox("constraints (yellow)", &cfg.debug_draw_constraints);
    ImGui::Separator();
    ImGui::TextWrapped("F1 toggle cursor (free/lock). F spawn box. G toggle all debug. ESC quit.");
    ImGui::End();
}

void ImGuiLayer::drawPerfPanel(float fps, float physicsMs) {
    ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Performance", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::End(); return;
    }
    ImGui::Text("FPS: %.0f", fps);
    ImGui::Text("Physics: %.2f ms", physicsMs);
    float physPct = (fps > 0) ? (physicsMs / (1000.0f / fps)) * 100.0f : 0.0f;
    ImGui::Text("Physics share: %.1f%%", physPct);
    ImGui::End();
}

void ImGuiLayer::drawHelpPanel() {
    // 简化: 帮助文本已在 Physics Debug 面板底部显示, 此处留空
}

} // namespace leo
