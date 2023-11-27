#include <random>
#include "boids.hpp"
#include "gl_debug.h"

Boids::Boids()
: m_vao(0), m_vbo(0), m_ebo(0), m_count(0) {

    GLCall( glGenVertexArrays(1, &m_vao) );
    GLCall( glGenBuffers(1, &m_vbo) );
    GLCall( glGenBuffers(1, &m_ebo) );

    float vertices[] = {
            0.5f,  0.5f, 0.0f,  // top right
            0.5f, -0.5f, 0.0f,  // bottom right
            -0.5f, -0.5f, 0.0f,  // bottom left
    };

    unsigned int indices[] = {
            0, 1, 2,  // Triangle
    };

    m_count = 3;

    GLCall( glBindVertexArray(m_vao) );

    GLCall( glBindBuffer(GL_ARRAY_BUFFER, m_vbo) );
    GLCall( glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW) );

    GLCall( glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo) );
    GLCall( glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW) );

    GLCall( glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0) );
    GLCall( glEnableVertexAttribArray(0) );

    GLCall( glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0) );
    GLCall( glBindBuffer(GL_ARRAY_BUFFER, 0) );
    GLCall( glBindVertexArray(0) );
}

void Boids::draw(ShaderProgram& shader_program) {
    shader_program.bind();
    GLCall( glBindVertexArray(m_vao) );
    GLCall( glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo) );
    GLCall( glDrawElementsInstanced(GL_TRIANGLES, m_count, GL_UNSIGNED_INT, nullptr, BOIDS_COUNT) );
    //GLCall( glDrawElements(GL_TRIANGLES, m_count, GL_UNSIGNED_INT, nullptr) );
}

Boids::~Boids() {
    GLCall( glDeleteVertexArrays(1, &m_vao) );
    GLCall( glDeleteBuffers(1, &m_vbo) );
    GLCall( glDeleteBuffers(1, &m_ebo) );
}

glm::vec3 Boids::rand_pos(float minX, float maxX, float minY, float maxY, float minZ, float maxZ) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> distX(minX, maxX);
    std::uniform_real_distribution<float> distY(minY, maxY);
    std::uniform_real_distribution<float> distZ(minZ, maxZ);

    float x = distX(gen);
    float y = distY(gen);
    float z = distZ(gen);

    return {x, y, z};
}
