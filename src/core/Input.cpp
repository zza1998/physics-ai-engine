#include "Input.h"

namespace leo {

bool Input::isKeyDown(int key) const {
    if (key < 0 || key >= (int)m_keyCurrent.size()) return false;
    return m_keyCurrent[key];
}

bool Input::isKeyPressed(int key) const {
    if (key < 0 || key >= (int)m_keyCurrent.size()) return false;
    return m_keyCurrent[key] && !m_keyPrevious[key];
}

bool Input::isKeyReleased(int key) const {
    if (key < 0 || key >= (int)m_keyCurrent.size()) return false;
    return !m_keyCurrent[key] && m_keyPrevious[key];
}

bool Input::isMouseButtonDown(int button) const {
    if (button < 0 || button >= (int)m_mouseCurrent.size()) return false;
    return m_mouseCurrent[button];
}

void Input::cursorPos(double& x, double& y) const {
    x = m_cursorX; y = m_cursorY;
}

void Input::mouseDelta(double& dx, double& dy) const {
    dx = m_deltaX; dy = m_deltaY;
}

float Input::scrollDelta() const { return m_scroll; }

void Input::onKey(int key, int /*scancode*/, int action, int /*mods*/) {
    if (key < 0 || key >= (int)m_keyCurrent.size()) return;
    m_keyCurrent[key] = (action != 0); // press/repeat = true, release = false
}

void Input::onMouseButton(int button, int action, int /*mods*/) {
    if (button < 0 || button >= (int)m_mouseCurrent.size()) return;
    m_mouseCurrent[button] = (action != 0);
}

void Input::onCursorPos(double x, double y) {
    if (m_firstMouse) {
        m_lastCursorX = x;
        m_lastCursorY = y;
        m_firstMouse = false;
    }
    m_deltaX = x - m_lastCursorX;
    m_deltaY = m_lastCursorY - y; // y 反转: 鼠标上移 = pitch 增加
    m_lastCursorX = x;
    m_lastCursorY = y;
    m_cursorX = x;
    m_cursorY = y;
}

void Input::onScroll(double /*xoff*/, double yoff) {
    m_scroll += (float)yoff;
}

void Input::endFrame() {
    m_keyPrevious = m_keyCurrent;
    m_mousePrevious = m_mouseCurrent;
    m_deltaX = 0;
    m_deltaY = 0;
    m_scroll = 0;
}

} // namespace leo
