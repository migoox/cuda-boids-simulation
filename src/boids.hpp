#ifndef BOIDS_SIMULATION_BOIDS_HPP
#define BOIDS_SIMULATION_BOIDS_HPP

// TODO: move to config file
#define BOIDS_COUNT 100

#include <GL/glew.h>
#include "shader_program.hpp"
#include <glm/glm.hpp>

class Boids {
public:
    Boids();
    ~Boids();
    void draw(ShaderProgram& shader_program);

    static glm::vec3 rand_pos(float minX, float maxX, float minY, float maxY, float minZ, float maxZ);
public:
    glm::vec3 positions[BOIDS_COUNT];
    glm::vec3 velocities[BOIDS_COUNT];

private:
    GLuint m_vao, m_vbo, m_ebo;
    uint32_t m_count;
};


#endif //BOIDS_SIMULATION_BOIDS_HPP
