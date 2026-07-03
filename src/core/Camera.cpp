#include "Camera.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/constants.hpp>
#include <GLFW/glfw3.h>

namespace leo {

Camera::Camera(const CameraConfig& cfg) : m_cfg(cfg) {
    updateVectors();
}

void Camera::updateVectors() {
    glm::vec3 front;
    front.x = cos(glm::radians(m_cfg.yaw)) * cos(glm::radians(m_cfg.pitch));
    front.y = sin(glm::radians(m_cfg.pitch));
    front.z = sin(glm::radians(m_cfg.yaw)) * cos(glm::radians(m_cfg.pitch));
    m_front = glm::normalize(front);
    m_right = glm::normalize(glm::cross(m_front, m_cfg.worldUp));
    m_up    = glm::normalize(glm::cross(m_right, m_front));
}

void Camera::update(float dt, const Input& input) {
    // 鼠标 look
    double dx, dy;
    input.mouseDelta(dx, dy);
    m_cfg.yaw   += (float)dx * m_cfg.mouseSensitivity;
    m_cfg.pitch += (float)dy * m_cfg.mouseSensitivity;
    if (m_cfg.pitch >  89.0f) m_cfg.pitch =  89.0f;
    if (m_cfg.pitch < -89.0f) m_cfg.pitch = -89.0f;

    updateVectors();

    // WASD 移动
    float v = m_cfg.moveSpeed * dt;
    if (input.isKeyDown(GLFW_KEY_W)) m_cfg.position += m_front * v;
    if (input.isKeyDown(GLFW_KEY_S)) m_cfg.position -= m_front * v;
    if (input.isKeyDown(GLFW_KEY_D)) m_cfg.position += m_right * v;
    if (input.isKeyDown(GLFW_KEY_A)) m_cfg.position -= m_right * v;
    if (input.isKeyDown(GLFW_KEY_SPACE))       m_cfg.position += m_cfg.worldUp * v;
    if (input.isKeyDown(GLFW_KEY_LEFT_SHIFT))  m_cfg.position -= m_cfg.worldUp * v;
}

glm::mat4 Camera::viewMatrix() const {
    return glm::lookAt(m_cfg.position, m_cfg.position + m_front, m_up);
}

glm::mat4 Camera::projectionMatrix(float aspect) const {
    return glm::perspective(glm::radians(m_cfg.fov), aspect, m_cfg.nearPlane, m_cfg.farPlane);
}

} // namespace leo
