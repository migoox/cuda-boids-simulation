#ifndef BOIDS_SIMULATION_BOID_HPP
#define BOIDS_SIMULATION_BOID_HPP
#include <GL/glew.h>
#include "shader_program.hpp"

class Boid {
public:
    Boid();
    ~Boid();
    void draw(ShaderProgram& shader_program);
private:
    GLuint m_vao, m_vbo, m_ebo;
    uint32_t m_count;
};


#endif //BOIDS_SIMULATION_BOID_HPP
