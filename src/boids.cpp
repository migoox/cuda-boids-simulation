#include <random>
#include "boids.hpp"
#include "gl_debug.h"
#include "boids_cpu.hpp"

boids::SimulationParameters::SimulationParameters()
        : distance(5.f),
          separation(1.f),
          alignment(1.f),
          cohesion(1.f),
          aquarium_size(glm::vec3(40.f, 40.f, 40.f)),
          min_speed(1.5f),
          max_speed(4.f),
          noise(0.f),
          boids_count(10000)
{ }

boids::SimulationParameters::SimulationParameters(float distance, float separation, float alignment, float cohesion)
: SimulationParameters() {
    this->distance = distance;
    this->separation = separation;
    this->alignment = alignment;
    this->cohesion = cohesion;
}

boids::BoidsRenderer::BoidsRenderer()
: m_mesh(common::Mesh()) {
    // Let a boid face the direction based on forward vector in lh
    float vertices[] = {
            0.3f,  0.f, -0.3f,
            -0.3f, 0.f, -0.3f,
            0.f, 0.f, 0.6f,
            0.f, 0.3f, -0.3f
    };

    unsigned int indices[] = {
            0, 1, 2,
            0, 3, 2,
            1, 2, 3,
            0, 1, 3
    };

    m_mesh.set(vertices, sizeof(vertices), indices, sizeof(indices), 12);

    GLCall( glGenBuffers(1, &m_pos_ubo_id) );
    GLCall( glGenBuffers(1, &m_orient_ubo_id) );
    GLCall( glBindBuffer(GL_UNIFORM_BUFFER, 0) );
}

void boids::BoidsRenderer::set_ubo(
        glm::vec4 position[SimulationParameters::MAX_BOID_COUNT],
        const BoidsOrientation& orientation
) {
    GLCall( glBindBuffer(GL_UNIFORM_BUFFER, m_pos_ubo_id) );
    GLCall( glBufferData(GL_UNIFORM_BUFFER, SimulationParameters::MAX_BOID_COUNT * 4, position, GL_DYNAMIC_DRAW) );

    GLCall( glBindBuffer(GL_UNIFORM_BUFFER, m_orient_ubo_id) );
    GLCall( glBufferData(GL_UNIFORM_BUFFER, sizeof(boids::BoidsOrientation), &orientation, GL_DYNAMIC_DRAW) );
    GLCall( glBindBuffer(GL_UNIFORM_BUFFER, 0) );
}

void boids::BoidsRenderer::draw(const common::ShaderProgram &shader_program, int count) const {
    shader_program.bind();
    m_mesh.bind();
    GLCall( GLuint pos_block_index = glGetUniformBlockIndex(shader_program.get_id(), "boids_block_position") );
    GLCall( GLuint orient_block_index = glGetUniformBlockIndex(shader_program.get_id(), "boids_block_orientation") );

    GLCall( glUniformBlockBinding(shader_program.get_id(), pos_block_index, 0) );
    GLCall( glUniformBlockBinding(shader_program.get_id(), orient_block_index, 1) );

    GLCall( glBindBufferBase(GL_UNIFORM_BUFFER, 0, m_pos_ubo_id) );
    GLCall( glBindBufferBase(GL_UNIFORM_BUFFER, 1, m_orient_ubo_id) );

    GLCall( glDrawElementsInstanced(GL_TRIANGLES, m_mesh.get_count(), GL_UNSIGNED_INT, nullptr, count) );
}

boids::Boids::Boids(const boids::SimulationParameters &sim_params) {
    this->reset(sim_params);
}

void boids::Boids::reset(const SimulationParameters& sim_params) {
    for (int i = 0; i < SimulationParameters::MAX_BOID_COUNT; ++i) {
       this->position[i] = glm::vec4(boids::rand_vec(
                -sim_params.aquarium_size.x / 2.f,
                sim_params.aquarium_size.x / 2.f,

                -sim_params.aquarium_size.y / 2.f,
                sim_params.aquarium_size.y / 2.f,

                -sim_params.aquarium_size.z / 2.f,
                sim_params.aquarium_size.z / 2.f
        ), 1.f);

        this->orientation.forward[i] = glm::vec4(0.f, 0.f, 1.f, 0.f);
        this->orientation.up[i] = glm::vec4(0.f, 1.f, 0.f, 0.f);
        this->orientation.right[i] = glm::vec4(1.f, 0.f, 0.f, 0.f);

        this->velocity[i] = glm::vec4(0.05f * glm::normalize(boids::rand_vec(1., -1., 1., -1., 1., -1.)), 1.f);
        this->acceleration[i] = glm::vec4(0.f);
    }

    // Update basis vectors (orientation)
    for (BoidId i = 0; i < SimulationParameters::MAX_BOID_COUNT; ++i) {
        orientation.forward[i] = glm::vec4(glm::normalize(velocity[i]), 0.f);
        orientation.right[i] = glm::vec4(glm::normalize(glm::cross(glm::vec3(orientation.up[i]), glm::vec3(orientation.forward[i]))), 0.f);
        orientation.up[i] = glm::vec4(glm::normalize(glm::cross(glm::vec3(orientation.forward[i]) , glm::vec3(orientation.right[i]))), 0.f);
    }
}


glm::vec3 boids::rand_vec(float min_x, float max_x, float min_y, float max_y, float min_z, float max_z) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dist_x(min_x, max_x);
    std::uniform_real_distribution<float> dist_y(min_y, max_y);
    std::uniform_real_distribution<float> dist_z(min_z, max_z);

    float x = dist_x(gen);
    float y = dist_y(gen);
    float z = dist_z(gen);

    return {x, y, z};
}

glm::vec3 boids::rand_unit_vec() {
    return glm::normalize(rand_vec(-1.f, 1.f, -1.f, 1.f, -1.f, 1.f));
}

boids::Obstacles::Obstacles()
: m_radius(), m_pos(), m_box() {
    m_radius.reserve(SimulationParameters::MAX_OBSTACLES_COUNT);
    m_pos.reserve(SimulationParameters::MAX_OBSTACLES_COUNT);
}

void boids::Obstacles::push(glm::vec3 pos, float radius) {
    if (m_radius.size() < SimulationParameters::MAX_OBSTACLES_COUNT) {
        m_radius.push_back(radius);
        m_pos.push_back(pos);
    }
}

void boids::Obstacles::remove(size_t elem) {
    m_radius.erase(m_radius.begin() + elem);
    m_pos.erase(m_pos.begin() + elem);
}

float &boids::Obstacles::radius(size_t elem) {
    return m_radius[elem];
}

glm::vec3 &boids::Obstacles::pos(size_t elem) {
    return m_pos[elem];
}

void boids::Obstacles::draw(common::ShaderProgram &program) {
    program.bind();
    for (int i = 0; i < m_radius.size(); ++i) {
        program.set_uniform_3f(("u_pos[" + std::to_string(i) + "]").c_str(), m_pos[i]);
        program.set_uniform_1f(("u_radius[" + std::to_string(i) + "]").c_str(), m_radius[i]);
    }
    m_box.draw_instanced(program, m_radius.size());
}
