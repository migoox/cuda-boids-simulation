#include <fstream>
#include <sstream>
#include <glm/glm.hpp>
#include "shader_program.hpp"
#include "gl_debug.h"

ShaderProgram::ShaderProgram(const char *vert_path, const char *frag_path)
: m_parsing_failed(false), m_id(0) {
    std::string vert_source = this->parse_shader(vert_path);
    std::string frag_source = this->parse_shader(frag_path);

    if (m_parsing_failed) {
        return;
    }

    m_id = create_shader_program(vert_source, frag_source);
}

ShaderProgram::~ShaderProgram() {
    GLCall( glDeleteProgram(m_id) );
}

void ShaderProgram::bind() const {
    GLCall( glUseProgram(m_id) );

}

void ShaderProgram::unbind() {
    GLCall( glUseProgram(0) );
}

GLint ShaderProgram::get_uniform_location(const char *name) {
    if (m_uniform_location_cache.find(name) != m_uniform_location_cache.end())
        return m_uniform_location_cache[name];

    GLCall( int location = glGetUniformLocation(m_id, name) );
    if (location == -1)
        std::cout << "[OpenGL] Warning: No active uniform variable with name " << name << " found" << std::endl;

    m_uniform_location_cache[name] = location;

    return location;
}

void ShaderProgram::set_uniform_1i(const char* name, int value)
{
    GLCall( glUniform1i(get_uniform_location(name), value) );
}

void ShaderProgram::set_uniform_2i(const char* name, int v1, int v2)
{
    GLCall( glUniform2i(get_uniform_location(name), v1, v2) );
}

void ShaderProgram::set_uniform_1f(const char* name, float value)
{
    GLCall( glUniform1f(get_uniform_location(name), value) );
}

void ShaderProgram::set_uniform_2f(const char* name, float f0, float f1)
{
    GLCall( glUniform2f(get_uniform_location(name), f0, f1) );
}

void ShaderProgram::set_uniform_3f(const char* name, float f0, float f1, float f2)
{
    GLCall( glUniform3f(get_uniform_location(name), f0, f1, f2) );
}

void ShaderProgram::set_uniform_3f(const char* name, glm::vec3 vec)
{
    GLCall( glUniform3f(get_uniform_location(name), vec.x, vec.y, vec.z) );
}

void ShaderProgram::set_uniform_4f(const char* name, float f0, float f1, float f2, float f3)
{
    GLCall( glUniform4f(get_uniform_location(name), f0, f1, f2, f3) );
}

void ShaderProgram::set_uniform_4f(const char* name, glm::vec4 vec)
{
    GLCall( glUniform4f(get_uniform_location(name), vec.x, vec.y, vec.z, vec.w) );
}

void ShaderProgram::set_uniform_mat3f(const char* name, const glm::mat3& matrix)
{
    GLCall(glUniformMatrix3fv(get_uniform_location(name), 1, GL_FALSE, &matrix[0][0]));
}

void ShaderProgram::set_uniform_mat4f(const char* name, const glm::mat4& matrix)
{
    GLCall( glUniformMatrix4fv(get_uniform_location(name), 1, GL_FALSE, &matrix[0][0]) );
}

std::string ShaderProgram::parse_shader(const char *filepath) {
    std::ifstream file(filepath);
    std::stringstream ss;

    if (!file.is_open())
    {
        std::cout << "[Shader Parser]: " << "Error: Cannot load shader: " << filepath << "\n";
        m_parsing_failed = true;
        return "";
    }

    ss << file.rdbuf();
    file.close();

    return std::move(ss.str());
}

GLuint ShaderProgram::compile_shader(GLenum type, const std::string &source) {
    GLCall( GLenum id = glCreateShader(type) );

    const char* src = source.c_str();
    GLCall( glShaderSource(id, 1, &src, nullptr) );
    GLCall( glCompileShader(id) );

    // Error handling
    int result;
    GLCall( glGetShaderiv(id, GL_COMPILE_STATUS, &result) );
    std::cout << "[Shader Compiler]: " << (type == GL_VERTEX_SHADER ? "vertex" : "fragment") << " shader compile status: " << result << std::endl;
    if ( result == GL_FALSE )
    {
        int length;
        GLCall( glGetShaderiv(id, GL_INFO_LOG_LENGTH, &length) );
        char* message = (char*) alloca(length * sizeof(char));
        GLCall( glGetShaderInfoLog(id, length, &length, message) );
        std::cout << "[Shader Compiler]: "
                  << "Failed to compile "
                  << (type == GL_VERTEX_SHADER ? "vertex" : "fragment")
                  << "shader"
                  << std::endl;
        std::cout << message << std::endl;
        GLCall( glDeleteShader(id) );
        std::terminate();
    }

    return id;
}

GLuint ShaderProgram::create_shader_program(const std::string &vert_shader, const std::string &frag_shader) {
    unsigned int program = glCreateProgram();
    unsigned int vs = compile_shader(GL_VERTEX_SHADER, vert_shader);
    unsigned int fs = compile_shader(GL_FRAGMENT_SHADER, frag_shader);

    GLCall( glAttachShader(program, vs) );
    GLCall( glAttachShader(program, fs) );

    GLCall( glLinkProgram(program) );

    GLint program_linked;

    GLCall( glGetProgramiv(program, GL_LINK_STATUS, &program_linked) );
    std::cout << "[Shader Linker]: Program link status: " << program_linked << std::endl;
    if (program_linked != GL_TRUE)
    {
        GLsizei log_length = 0;
        GLchar message[1024];
        GLCall( glGetProgramInfoLog(program, 1024, &log_length, message) );
        std::cout << "[Shader Linker]: Failed to link program" << std::endl;
        std::cout << message << std::endl;
        std::terminate();
    }

    GLCall( glValidateProgram(program) );

    GLCall( glDeleteShader(vs) );
    GLCall( glDeleteShader(fs) );

    return program;
}

bool ShaderProgram::is_valid() const {
    return !m_parsing_failed;
}

