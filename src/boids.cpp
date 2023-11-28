#include <random>
#include "boids.hpp"
#include "gl_debug.h"

boids::BoidsRenderer::BoidsRenderer()
: m_vao(0), m_vbo(0), m_ebo(0), m_count(0) {

    // Initialize OpenGL
    GLCall( glGenVertexArrays(1, &m_vao) );
    GLCall( glGenBuffers(1, &m_vbo) );
    GLCall( glGenBuffers(1, &m_ebo) );

    float vertices[] = {
            -0.3f,  0.f, 0.3f,
            0.3f, 0.f, 0.3f,
            0.f, 0.f, -0.6f,
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

void boids::BoidsRenderer::draw(ShaderProgram& shader_program) const {
    shader_program.bind();
    GLCall( glBindVertexArray(m_vao) );
    GLCall( glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo) );
    GLCall( glDrawElementsInstanced(GL_TRIANGLES, m_count, GL_UNSIGNED_INT, nullptr, BOIDS_COUNT) );
}

boids::BoidsRenderer::~BoidsRenderer() {
    GLCall( glDeleteVertexArrays(1, &m_vao) );
    GLCall( glDeleteBuffers(1, &m_vbo) );
    GLCall( glDeleteBuffers(1, &m_ebo) );
}

glm::vec3 boids::rand_vec(float min_x, float max_x, float min_y, float max_y, float min_z, float max_z) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> distX(min_x, max_x);
    std::uniform_real_distribution<float> distY(min_y, max_y);
    std::uniform_real_distribution<float> distZ(min_z, max_z);

    float x = distX(gen);
    float y = distY(gen);
    float z = distZ(gen);

    return {x, y, z};
}

boids::Boids::Boids() {
    this->reset();
}

void boids::Boids::reset() {
    for (int i = 0; i < BOIDS_COUNT; ++i) {
        this->position[i] = boids::rand_vec(5., -5., 5., -5., 5., -5.);
        this->forward[i] = glm::vec3(0., 0., 1.);
        this->up[i] = glm::vec3(0., 1., 0.);
        this->right[i] = glm::vec3(1., 0., 0.);

        this->velocity[i] = boids::rand_vec(1., -1., 1., -1., 1., -1.);
    }
}

void boids::find_basis_vectors(
        glm::vec3 velocity[BOIDS_COUNT],
        glm::vec3 forward[BOIDS_COUNT],
        glm::vec3 up[BOIDS_COUNT],
        glm::vec3 right[BOIDS_COUNT]
) {

}

void boids::update_shader(
        ShaderProgram &shader_program,
        glm::vec3 position[BOIDS_COUNT],
        glm::vec3 forward[BOIDS_COUNT],
        glm::vec3 up[BOIDS_COUNT],
        glm::vec3 right[BOIDS_COUNT]
) {
    shader_program.bind();
    for (int i = 0; i < BOIDS_COUNT; ++i) {
        shader_program.set_uniform_3f(("u_position[" + std::to_string(i) + "]").c_str(), position[i]);
        shader_program.set_uniform_3f(("u_forward[" + std::to_string(i) + "]").c_str(), forward[i]);
        shader_program.set_uniform_3f(("u_up[" + std::to_string(i) + "]").c_str(), up[i]);
        shader_program.set_uniform_3f(("u_right[" + std::to_string(i) + "]").c_str(), right[i]);
    }
}
