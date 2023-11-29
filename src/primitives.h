#ifndef BOIDS_SIMULATION_PRIMITIVES_H
#define BOIDS_SIMULATION_PRIMITIVES_H
#include <GL/glew.h>
#include "shader_program.hpp"

namespace common {
    class Mesh {
    public:
        Mesh();
        Mesh(float *vertices, size_t vertices_size, u_int32_t *indices, size_t indices_size, u_int32_t indices_count);
        ~Mesh();

        void set(float *vertices, size_t vertices_size, u_int32_t *indices, size_t indices_size, u_int32_t indices_count);
        void load(const char *path);

        void bind() const;
        void unbind() const;

        uint32_t get_count() const { return m_count; };

    private:
        GLuint m_vao, m_vbo, m_ebo;
        uint32_t m_count;
    };

    class Box {
    public:
        Box();

        void draw(const ShaderProgram &program) const;
    private:
        Mesh m_mesh;
    };
}

#endif //BOIDS_SIMULATION_PRIMITIVES_H
