#include "Mesh.h"
#include <cstddef> // offsetof

namespace leo {

Mesh::Mesh() = default;

Mesh::~Mesh() {
    release();
}

Mesh::Mesh(Mesh&& other) noexcept {
    m_vao = other.m_vao;
    m_vbo = other.m_vbo;
    m_ibo = other.m_ibo;
    m_indexCount = other.m_indexCount;
    m_drawMode = other.m_drawMode;
    m_vertCount = other.m_vertCount;
    other.m_vao = 0;
    other.m_vbo = 0;
    other.m_ibo = 0;
    other.m_indexCount = 0;
    other.m_drawMode = GL_TRIANGLES;
    other.m_vertCount = 0;
}

Mesh& Mesh::operator=(Mesh&& other) noexcept {
    if (this != &other) {
        release();
        m_vao = other.m_vao;
        m_vbo = other.m_vbo;
        m_ibo = other.m_ibo;
        m_indexCount = other.m_indexCount;
        m_drawMode = other.m_drawMode;
        m_vertCount = other.m_vertCount;
        other.m_vao = 0;
        other.m_vbo = 0;
        other.m_ibo = 0;
        other.m_indexCount = 0;
        other.m_drawMode = GL_TRIANGLES;
        other.m_vertCount = 0;
    }
    return *this;
}

bool Mesh::upload(const std::vector<Vertex>& verts, const std::vector<GLuint>& indices,
                  GLenum drawMode, bool dynamic) {
    release();

    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &m_vbo);
    glGenBuffers(1, &m_ibo);

    glBindVertexArray(m_vao);

    GLenum usage = dynamic ? GL_DYNAMIC_DRAW : GL_STATIC_DRAW;
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(Vertex), verts.data(), usage);

    if (!indices.empty()) {
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ibo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint), indices.data(), usage);
    }

    // location 0 = position (vec3), location 1 = normal (vec3)
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, position));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));

    glBindVertexArray(0);

    m_indexCount = (GLsizei)indices.size();
    m_vertCount = (GLsizei)verts.size();
    m_drawMode = drawMode;
    return true;
}

void Mesh::updateVertices(const std::vector<Vertex>& verts) {
    // 允许部分更新: 上传的顶点数 <= upload 时预分配的容量 (m_vertCount).
    // DebugRenderer 预分配 1024 点 / 4096 线容量, 但每帧实际点/线数动态变化
    // (例如 4 个箱子 = 32 点 / 96 棱线端点). 之前要求 size == m_vertCount 会静默跳过,
    // 导致 GPU buffer 仍是 init 时的全零数据, draw(overrideCount) 画的全是原点退化几何.
    if (!m_vbo || verts.empty()) return;
    if (verts.size() > (size_t)m_vertCount) return;  // 超过预分配容量, 拒绝
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferSubData(GL_ARRAY_BUFFER, 0, verts.size() * sizeof(Vertex), verts.data());
}

void Mesh::release() {
    if (m_vao) { glDeleteVertexArrays(1, &m_vao); m_vao = 0; }
    if (m_vbo) { glDeleteBuffers(1, &m_vbo); m_vbo = 0; }
    if (m_ibo) { glDeleteBuffers(1, &m_ibo); m_ibo = 0; }
    m_indexCount = 0;
    m_vertCount = 0;
}

void Mesh::bind() const { glBindVertexArray(m_vao); }

void Mesh::draw(GLsizei overrideCount) const {
    if (!m_vao) return;
    glBindVertexArray(m_vao);
    if (m_indexCount > 0) {
        GLsizei n = (overrideCount > 0) ? overrideCount : m_indexCount;
        glDrawElements(m_drawMode, n, GL_UNSIGNED_INT, 0);
    } else if (m_vertCount > 0) {
        GLsizei n = (overrideCount > 0) ? overrideCount : m_vertCount;
        glDrawArrays(m_drawMode, 0, n);
    }
    glBindVertexArray(0);
}

} // namespace leo
