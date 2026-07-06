#pragma once

#include <vector>
#include <glad/gl.h>
#include "graphics/Vertex.h"

namespace leo {

class Mesh {
public:
    Mesh();
    ~Mesh();

    // Mesh 持有 GL 资源 (VAO/VBO/IBO), 必须禁用拷贝, 实现移动
    // 否则 makeCube() 返回临时对象 -> 拷贝 -> 临时对象析构释放 GL 资源 -> 悬空
    Mesh(const Mesh&) = delete;
    Mesh& operator=(const Mesh&) = delete;
    Mesh(Mesh&& other) noexcept;
    Mesh& operator=(Mesh&& other) noexcept;

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
