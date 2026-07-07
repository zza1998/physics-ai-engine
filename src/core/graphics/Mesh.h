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

    bool upload(const std::vector<Vertex>& verts, const std::vector<GLuint>& indices,
                GLenum drawMode = GL_TRIANGLES, bool dynamic = false);
    void release();

    // 动态更新顶点位置 (debug 点云/线每帧用), 顶点数需与 upload 时一致
    void updateVertices(const std::vector<Vertex>& verts);

    void bind() const;  // glBindVertexArray
    // overrideCount>0 时只画前 overrideCount 个顶点/索引 (debug 动态绘制用)
    void draw(GLsizei overrideCount = 0) const;

    GLsizei indexCount() const { return m_indexCount; }
    bool valid() const { return m_vao != 0; }

private:
    GLuint  m_vao = 0, m_vbo = 0, m_ibo = 0;
    GLsizei m_indexCount = 0;
    GLenum  m_drawMode = GL_TRIANGLES;
    GLsizei m_vertCount = 0;
};

} // namespace leo
