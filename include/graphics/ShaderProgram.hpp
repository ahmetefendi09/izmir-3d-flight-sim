#pragma once
#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glext.h>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include "../core/Types.hpp"

namespace izmir {

class ShaderProgram {
private:
    GLuint m_programId = 0;

    static std::string readFileText(const std::string& path) {
        std::ifstream in(path, std::ios::in | std::ios::binary);
        if (!in.good()) {
            std::cerr << "[SHADER HATA] Dosya okunamadi: " << path << "\n";
            return "";
        }
        std::ostringstream ss;
        ss << in.rdbuf();
        return ss.str();
    }

    static GLuint compileSingleShader(GLenum type, const std::string& src, const std::string& debugName) {
        GLuint sh = glCreateShader(type);
        const char* ptr = src.c_str();
        glShaderSource(sh, 1, &ptr, nullptr);
        glCompileShader(sh);

        GLint ok = 0;
        glGetShaderiv(sh, GL_COMPILE_STATUS, &ok);
        if (!ok) {
            char log[1024];
            glGetShaderInfoLog(sh, sizeof(log), nullptr, log);
            std::cerr << "[SHADER DERLEME HATASI - " << debugName << "]:\n" << log << "\n";
            glDeleteShader(sh);
            return 0;
        }
        return sh;
    }

public:
    ShaderProgram() = default;

    bool loadFromFiles(const std::string& vertPath, const std::string& fragPath) {
        std::string vSrc = readFileText(vertPath);
        std::string fSrc = readFileText(fragPath);
        if (vSrc.empty() || fSrc.empty()) return false;

        GLuint vs = compileSingleShader(GL_VERTEX_SHADER, vSrc, vertPath);
        GLuint fs = compileSingleShader(GL_FRAGMENT_SHADER, fSrc, fragPath);
        if (!vs || !fs) return false;

        m_programId = glCreateProgram();
        glAttachShader(m_programId, vs);
        glAttachShader(m_programId, fs);
        glLinkProgram(m_programId);

        glDeleteShader(vs);
        glDeleteShader(fs);

        GLint ok = 0;
        glGetProgramiv(m_programId, GL_LINK_STATUS, &ok);
        if (!ok) {
            char log[1024];
            glGetProgramInfoLog(m_programId, sizeof(log), nullptr, log);
            std::cerr << "[SHADER LINK HATASI]:\n" << log << "\n";
            glDeleteProgram(m_programId);
            m_programId = 0;
            return false;
        }
        return true;
    }

    void use() const { glUseProgram(m_programId); }
    static void unbind() { glUseProgram(0); }
    GLuint id() const { return m_programId; }

    void setInt(const char* name, int val) const {
        glUniform1i(glGetUniformLocation(m_programId, name), val);
    }
    void setFloat(const char* name, float val) const {
        glUniform1f(glGetUniformLocation(m_programId, name), val);
    }
    void setVec3(const char* name, const Vec3f& v) const {
        glUniform3f(glGetUniformLocation(m_programId, name), v.x, v.y, v.z);
    }
};

} // namespace izmir
