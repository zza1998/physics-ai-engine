#include "graphics/MeshPrimitives.h"
#include "graphics/Vertex.h"
#include <vector>

namespace leo {

Mesh makeCube() {
    // 6 面 x 4 顶点 = 24, 每面独立顶点保证法线不插值模糊
    // 顺序: +X -X +Y -Y +Z -Z
    std::vector<Vertex> verts;
    verts.reserve(24);

    // 每面: 4 顶点 (法线 = 面方向), 2 三角形
    auto face = [&](glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 d, glm::vec3 n) {
        verts.push_back({a, n});
        verts.push_back({b, n});
        verts.push_back({c, n});
        verts.push_back({d, n});
    };

    // +X (右)
    face({ 0.5f,-0.5f, 0.5f}, { 0.5f,-0.5f,-0.5f}, { 0.5f, 0.5f,-0.5f}, { 0.5f, 0.5f, 0.5f}, { 1,0,0});
    // -X (左)
    face({-0.5f,-0.5f,-0.5f}, {-0.5f,-0.5f, 0.5f}, {-0.5f, 0.5f, 0.5f}, {-0.5f, 0.5f,-0.5f}, {-1,0,0});
    // +Y (上)
    face({-0.5f, 0.5f, 0.5f}, { 0.5f, 0.5f, 0.5f}, { 0.5f, 0.5f,-0.5f}, {-0.5f, 0.5f,-0.5f}, {0,1,0});
    // -Y (下)
    face({-0.5f,-0.5f,-0.5f}, { 0.5f,-0.5f,-0.5f}, { 0.5f,-0.5f, 0.5f}, {-0.5f,-0.5f, 0.5f}, {0,-1,0});
    // +Z (前)
    face({-0.5f,-0.5f, 0.5f}, { 0.5f,-0.5f, 0.5f}, { 0.5f, 0.5f, 0.5f}, {-0.5f, 0.5f, 0.5f}, {0,0,1});
    // -Z (后)
    face({ 0.5f,-0.5f,-0.5f}, {-0.5f,-0.5f,-0.5f}, {-0.5f, 0.5f,-0.5f}, { 0.5f, 0.5f,-0.5f}, {0,0,-1});

    std::vector<GLuint> indices;
    indices.reserve(36);
    for (GLuint i = 0; i < 24; i += 4) {
        indices.insert(indices.end(), {
            i, i + 1, i + 2,
            i, i + 2, i + 3
        });
    }

    Mesh m;
    m.upload(verts, indices);
    return m;
}

Mesh makePlane(float width, float depth) {
    float hw = width * 0.5f;
    float hd = depth * 0.5f;
    glm::vec3 n = {0.0f, 1.0f, 0.0f};
    std::vector<Vertex> verts = {
        {{-hw, 0,  hd}, n},
        {{ hw, 0,  hd}, n},
        {{ hw, 0, -hd}, n},
        {{-hw, 0, -hd}, n},
    };
    std::vector<GLuint> indices = {
        0, 1, 2,
        0, 2, 3,
    };
    Mesh m;
    m.upload(verts, indices);
    return m;
}

} // namespace leo
