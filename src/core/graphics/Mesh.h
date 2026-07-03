#pragma once

#include <vector>
#include <glad/gl.h>
#include "graphics/Vertex.h"

namespace leo {

class Mesh {
public:
    Mesh();
    ~Mesh();

    bool upload(const std::vector<Vertex>& verts, const std::vector<GLuint>& indices);
    void release();

    void bind() const;  // glBindVertexArray
    void draw() const;  // glDrawElements

    GLsizei indexCount() const { return m_indexCount; }
    bool valid() const { return m_vao != 0; }

private:
    GLuint  m_vao = 0, m_vbo = 0, m_ibo = 0;
    GLsizei m_indexCount = 0;
};

} // namespace leo
