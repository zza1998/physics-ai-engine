#pragma once

#include <string>
#include <unordered_map>
#include <glad/gl.h>
#include <glm/glm.hpp>

namespace leo {

class Shader {
public:
    Shader();
    ~Shader();

    bool loadFromFile(const std::string& vertPath, const std::string& fragPath);
    bool loadFromSource(const std::string& vertSrc, const std::string& fragSrc);

    void use() const;
    GLuint id() const { return m_program; }

    void setMat4(const char* name, const glm::mat4& m);
    void setVec3(const char* name, const glm::vec3& v);
    void setFloat(const char* name, float f);
    void setInt(const char* name, int i);

private:
    GLint uniformLocation(const char* name);
    static bool compile(GLenum type, const std::string& src, GLuint& out);
    static bool link(GLuint vert, GLuint frag, GLuint& outProgram);

    GLuint m_program = 0;
    std::unordered_map<std::string, GLint> m_uniformCache;
};

} // namespace leo
