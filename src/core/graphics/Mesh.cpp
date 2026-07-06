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
    other.m_vao = 0;
    other.m_vbo = 0;
    other.m_ibo = 0;
    other.m_indexCount = 0;
}

Mesh& Mesh::operator=(Mesh&& other) noexcept {
    if (this != &other) {
        release();
        m_vao = other.m_vao;
        m_vbo = other.m_vbo;
        m_ibo = other.m_ibo;
        m_indexCount = other.m_indexCount;
        other.m_vao = 0;
        other.m_vbo = 0;
        other.m_ibo = 0;
        other.m_indexCount = 0;
    }
    return *this;
}

bool Mesh::upload(const std::vector<Vertex>& verts, const std::vector<GLuint>& indices) {
    release();

    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &m_vbo);
    glGenBuffers(1, &m_ibo);

    glBindVertexArray(m_vao);

    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(Vertex), verts.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint), indices.data(), GL_STATIC_DRAW);

    // location 0 = position (vec3), location 1 = normal (vec3)
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, position));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));

    glBindVertexArray(0);

    m_indexCount = (GLsizei)indices.size();
    return true;
}

void Mesh::release() {
    if (m_vao) { glDeleteVertexArrays(1, &m_vao); m_vao = 0; }
    if (m_vbo) { glDeleteBuffers(1, &m_vbo); m_vbo = 0; }
    if (m_ibo) { glDeleteBuffers(1, &m_ibo); m_ibo = 0; }
    m_indexCount = 0;
}

void Mesh::bind() const { glBindVertexArray(m_vao); }

void Mesh::draw() const {
    if (!m_vao || m_indexCount == 0) return;
    glBindVertexArray(m_vao);
    glDrawElements(GL_TRIANGLES, m_indexCount, GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}

} // namespace leo
