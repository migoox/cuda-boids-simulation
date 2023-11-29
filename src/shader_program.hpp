#ifndef SHADER_PROGRAM_HPP
#define SHADER_PROGRAM_HPP
#include <GL/glew.h>
#include <glm/glm.hpp>
#include <string>
#include <unordered_map>

class ShaderProgram {
public:
    ShaderProgram(const char* vert_path, const char* frag_path);
    ~ShaderProgram();

    bool is_valid() const;

    void bind() const;
    static void unbind();

    void set_uniform_1i(const char* name, int value);
    void set_uniform_2i(const char* name, int v1, int v2);
    void set_uniform_1f(const char* name, float value);
    void set_uniform_2f(const char* name, float f0, float f1);
    void set_uniform_3f(const char* name, float f0, float f1, float f2);
    void set_uniform_3f(const char* name, glm::vec3 vec);
    void set_uniform_4f(const char* name, float f0, float f1, float f2, float f3);
    void set_uniform_4f(const char* name, glm::vec4 vec);
    void set_uniform_mat3f(const char* name, const glm::mat3& matrix);
    void set_uniform_mat4f(const char* name, const glm::mat4& matrix);

private:
    GLint get_uniform_location(const char* name);
    std::string parse_shader(const char* filepath);

    static GLuint compile_shader(GLenum type, const std::string& source);
    static GLuint create_shader_program(const std::string& vert_shader, const std::string& frag_shader);

private:
    GLuint m_id;
    bool m_parsing_failed;
    std::unordered_map<std::string, int> m_uniform_location_cache;
};

#endif