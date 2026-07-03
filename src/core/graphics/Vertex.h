#pragma once

#include <glm/glm.hpp>

namespace leo {

// M1 顶点布局: 位置 + 法线 (够 Blinn-Phong)
// M2 可加 texCoord, color 等
struct Vertex {
    glm::vec3 position;
    glm::vec3 normal;
};

} // namespace leo
