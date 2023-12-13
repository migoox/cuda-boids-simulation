#include <random>
#include "boids.hpp"
#include "gl_debug.h"
#include "boids_cpu.hpp"
#include "cuda_runtime.h"
#include "cuda_gl_interop.h"

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

    m_mesh.bind();
    GLCall( glGenBuffers(1, &m_pos_vbo_id) );
    GLCall( glBindBuffer(GL_ARRAY_BUFFER, m_pos_vbo_id) );
    GLCall( glEnableVertexAttribArray(1) );
    GLCall( glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(glm::vec4), (void*)0) );
    GLCall( glVertexAttribDivisor(1, 1) );

    GLCall( glGenBuffers(1, &m_forward_vbo_id) );
    GLCall( glBindBuffer(GL_ARRAY_BUFFER, m_forward_vbo_id) );
    GLCall( glEnableVertexAttribArray(2) );
    GLCall( glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, sizeof(glm::vec4), (void*)0) );
    GLCall( glVertexAttribDivisor(2, 1) );

    GLCall( glGenBuffers(1, &m_up_vbo_id) );
    GLCall( glBindBuffer(GL_ARRAY_BUFFER, m_up_vbo_id) );
    GLCall( glEnableVertexAttribArray(3) );
    GLCall( glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, sizeof(glm::vec4), (void*)0) );
    GLCall( glVertexAttribDivisor(3, 1) );

    GLCall( glGenBuffers(1, &m_right_vbo_id) );
    GLCall( glBindBuffer(GL_ARRAY_BUFFER, m_right_vbo_id) );
    GLCall( glEnableVertexAttribArray(4) );
    GLCall( glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, sizeof(glm::vec4), (void*)0) );
    GLCall( glVertexAttribDivisor(4, 1) );

    GLCall( glBindBuffer(GL_ARRAY_BUFFER, 0) );
    GLCall( glBindVertexArray(0) );
}

void boids::BoidsRenderer::set_vbos(const SimulationParameters& params, const std::vector<glm::vec4> &position, const boids::BoidsOrientation &orientation) {
    GLCall( glBindBuffer(GL_ARRAY_BUFFER, m_pos_vbo_id) );
    GLCall( glBufferData(GL_ARRAY_BUFFER, params.boids_count * sizeof(glm::vec4), position.data(), GL_DYNAMIC_DRAW));

    GLCall( glBindBuffer(GL_ARRAY_BUFFER, m_forward_vbo_id) );
    GLCall( glBufferData(GL_ARRAY_BUFFER, params.boids_count * sizeof(glm::vec4), orientation.forward.data(), GL_DYNAMIC_DRAW));

    GLCall( glBindBuffer(GL_ARRAY_BUFFER, m_up_vbo_id) );
    GLCall( glBufferData(GL_ARRAY_BUFFER, params.boids_count * sizeof(glm::vec4), orientation.up.data(), GL_DYNAMIC_DRAW));

    GLCall( glBindBuffer(GL_ARRAY_BUFFER, m_right_vbo_id) );
    GLCall( glBufferData(GL_ARRAY_BUFFER, params.boids_count * sizeof(glm::vec4), orientation.right.data(), GL_DYNAMIC_DRAW));
}

void boids::BoidsRenderer::cuda_register_vbos(cudaGraphicsResource** positions, cudaGraphicsResource** forward, cudaGraphicsResource** up, cudaGraphicsResource** right) const {
    GLCall( glBindBuffer(GL_ARRAY_BUFFER, m_pos_vbo_id) );
    cudaGraphicsGLRegisterBuffer(positions, m_pos_vbo_id, cudaGraphicsMapFlagsWriteDiscard);

    GLCall( glBindBuffer(GL_ARRAY_BUFFER, m_forward_vbo_id) );
    cudaGraphicsGLRegisterBuffer(forward, m_forward_vbo_id, cudaGraphicsMapFlagsWriteDiscard);

    GLCall( glBindBuffer(GL_ARRAY_BUFFER, m_up_vbo_id) );
    cudaGraphicsGLRegisterBuffer(up, m_up_vbo_id, cudaGraphicsMapFlagsWriteDiscard);

    GLCall( glBindBuffer(GL_ARRAY_BUFFER, m_right_vbo_id) );
    cudaGraphicsGLRegisterBuffer(right, m_right_vbo_id, cudaGraphicsMapFlagsWriteDiscard);
}

void boids::BoidsRenderer::draw(const common::ShaderProgram &shader_program, int count) const {
    shader_program.bind();
    m_mesh.bind();
    GLCall( glDrawElementsInstanced(GL_TRIANGLES, m_mesh.get_count(), GL_UNSIGNED_INT, nullptr, count) );
}

boids::Boids::Boids(const boids::SimulationParameters &sim_params) {
    this->position.resize(SimulationParameters::MAX_BOID_COUNT);
    this->orientation.forward.resize(SimulationParameters::MAX_BOID_COUNT);
    this->orientation.up.resize(SimulationParameters::MAX_BOID_COUNT);
    this->orientation.right.resize(SimulationParameters::MAX_BOID_COUNT);
    this->velocity.resize(SimulationParameters::MAX_BOID_COUNT);
    this->acceleration.resize(SimulationParameters::MAX_BOID_COUNT);
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
    if (max_x < min_x) {
        std::swap(min_x, max_x);
    }
    if (max_y < min_y) {
        std::swap(min_y, max_y);
    }
    if (max_z < min_z) {
        std::swap(min_z, max_z);
    }

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

const float &boids::Obstacles::radius(size_t elem) const {
    return m_radius[elem];
}

const glm::vec3 &boids::Obstacles::pos(size_t elem) const {
    return m_pos[elem];
}

void boids::Obstacles::clear() {
    m_pos.clear();
    m_radius.clear();
}
