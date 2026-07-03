#pragma once

#include <array>

namespace leo {

// 键鼠输入状态: current/previous 双数组做 edge detect
class Input {
public:
    bool isKeyDown(int key) const;       // 当前按住
    bool isKeyPressed(int key) const;    // 本帧刚按下 (edge)
    bool isKeyReleased(int key) const;   // 本帧刚释放 (edge)
    bool isMouseButtonDown(int button) const;

    void cursorPos(double& x, double& y) const;
    void mouseDelta(double& dx, double& dy) const;
    float scrollDelta() const;

    // GLFW 回调 (Window 的静态 trampoline 调用)
    void onKey(int key, int scancode, int action, int mods);
    void onMouseButton(int button, int action, int mods);
    void onCursorPos(double x, double y);
    void onScroll(double xoff, double yoff);

    // 每帧末调用: current -> previous, 清 delta/scroll
    void endFrame();

private:
    // action: 0=release 1=press 2=repeat; 存为 bool 按住状态
    std::array<bool, 512> m_keyCurrent{};
    std::array<bool, 512> m_keyPrevious{};
    std::array<bool, 8>   m_mouseCurrent{};
    std::array<bool, 8>   m_mousePrevious{};

    double m_cursorX = 0, m_cursorY = 0;
    double m_lastCursorX = 0, m_lastCursorY = 0;
    double m_deltaX = 0, m_deltaY = 0;
    float  m_scroll = 0;
    bool   m_firstMouse = true;
};

} // namespace leo
