#include "Shader.h"
#include "Logger.h"
#include <fstream>
#include <sstream>

namespace leo {

Shader::Shader() = default;

Shader::~Shader() {
    if (m_program) glDeleteProgram(m_program);
}

bool Shader::loadFromFile(const std::string& vertPath, const std::string& fragPath) {
    auto readFile = [](const std::string& path, std::string& out) -> bool {
        std::ifstream f(path);
        if (!f) return false;
        std::stringstream ss;
        ss << f.rdbuf();
        out = ss.str();
        return true;
    };
    std::string vertSrc, fragSrc;
    if (!readFile(vertPath, vertSrc)) {
        LEO_LOG_ERROR("Failed to read vertex shader: " + vertPath);
        return false;
    }
    if (!readFile(fragPath, fragSrc)) {
        LEO_LOG_ERROR("Failed to read fragment shader: " + fragPath);
        return false;
    }
    return loadFromSource(vertSrc, fragSrc);
}

bool Shader::loadFromSource(const std::string& vertSrc, const std::string& fragSrc) {
    GLuint vert = 0, frag = 0;
    if (!compile(GL_VERTEX_SHADER, vertSrc, vert))   return false;
    if (!compile(GL_FRAGMENT_SHADER, fragSrc, frag)) { glDeleteShader(vert); return false; }

    if (!link(vert, frag, m_program)) {
        glDeleteShader(vert);
        glDeleteShader(frag);
        return false;
    }
    // 编译链接后可删除 shader 对象
    glDeleteShader(vert);
    glDeleteShader(frag);
    return true;
}

void Shader::use() const { glUseProgram(m_program); }

GLint Shader::uniformLocation(const char* name) {
    auto it = m_uniformCache.find(name);
    if (it != m_uniformCache.end()) return it->second;
    GLint loc = glGetUniformLocation(m_program, name);
    m_uniformCache[name] = loc;
    return loc;
}

void Shader::setMat4(const char* name, const glm::mat4& m) {
    glUniformMatrix4fv(uniformLocation(name), 1, GL_FALSE, &m[0][0]);
}
void Shader::setVec3(const char* name, const glm::vec3& v) {
    glUniform3fv(uniformLocation(name), 1, &v[0]);
}
void Shader::setFloat(const char* name, float f) {
    glUniform1f(uniformLocation(name), f);
}
void Shader::setInt(const char* name, int i) {
    glUniform1i(uniformLocation(name), i);
}

bool Shader::compile(GLenum type, const std::string& src, GLuint& out) {
    out = glCreateShader(type);
    const char* cstr = src.c_str();
    glShaderSource(out, 1, &cstr, nullptr);
    glCompileShader(out);

    GLint ok = 0;
    glGetShaderiv(out, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[1024];
        glGetShaderInfoLog(out, sizeof(log), nullptr, log);
        LEO_LOG_ERROR(std::string("Shader compile failed: ") + log);
        glDeleteShader(out);
        out = 0;
        return false;
    }
    return true;
}

bool Shader::link(GLuint vert, GLuint frag, GLuint& outProgram) {
    outProgram = glCreateProgram();
    glAttachShader(outProgram, vert);
    glAttachShader(outProgram, frag);
    glLinkProgram(outProgram);

    GLint ok = 0;
    glGetProgramiv(outProgram, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[1024];
        glGetProgramInfoLog(outProgram, sizeof(log), nullptr, log);
        LEO_LOG_ERROR(std::string("Program link failed: ") + log);
        glDeleteProgram(outProgram);
        outProgram = 0;
        return false;
    }
    return true;
}

} // namespace leo
