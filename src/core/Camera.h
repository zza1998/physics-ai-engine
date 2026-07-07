#pragma once

#include <glm/glm.hpp>
#include "Input.h"

namespace leo {

struct CameraConfig {
    glm::vec3 position = {0.0f, 2.0f, 5.0f};
    glm::vec3 worldUp  = {0.0f, 1.0f, 0.0f};
    float yaw   = -90.0f;  // -Z 朝前
    float pitch = 0.0f;
    float fov       = 45.0f;
    float nearPlane = 0.1f;
    float farPlane  = 1000.0f;
    float moveSpeed        = 5.0f;
    float mouseSensitivity = 0.1f;
};

class Camera {
public:
    Camera(const CameraConfig& cfg = {});

    // 读鼠标改 yaw/pitch, 读 WASD 改 position
    void update(float dt, const Input& input);

    glm::mat4 viewMatrix() const;
    glm::mat4 projectionMatrix(float aspect) const;
    const glm::vec3& position() const { return m_cfg.position; }
    const glm::vec3& front() const { return m_front; }

private:
    void updateVectors();

    CameraConfig m_cfg;
    glm::vec3 m_front = {0,0,-1};
    glm::vec3 m_right = {1,0,0};
    glm::vec3 m_up    = {0,1,0};
};

} // namespace leo
