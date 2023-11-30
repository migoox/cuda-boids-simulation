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
          noise(0.f)
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
}

void boids::BoidsRenderer::draw(const common::ShaderProgram& shader_program) const {
    shader_program.bind();
    m_mesh.bind();
    GLCall( glDrawElementsInstanced(GL_TRIANGLES, m_mesh.get_count(), GL_UNSIGNED_INT, nullptr, SimulationParameters::MAX_BOID_COUNT) );
}

boids::Boids::Boids() {
    this->reset();
}

void boids::Boids::reset() {
    for (int i = 0; i < SimulationParameters::MAX_BOID_COUNT; ++i) {
        this->position[i] = boids::rand_vec(5., -5., 5., -5., 5., -5.);
        this->forward[i] = glm::vec3(0.f, 0.f, 1.f);
        this->up[i] = glm::vec3(0.f, 1.f, 0.f);
        this->right[i] = glm::vec3(1.f, 0.f, 0.f);

        this->velocity[i] = 0.05f * glm::normalize(boids::rand_vec(1., -1., 1., -1., 1., -1.));
        this->acceleration[i] = glm::vec3(0.f);
    }
    boids::cpu::update_basis_vectors(this->velocity, this->forward, this->up, this->right);
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



void boids::rand_aquarium_positions(const boids::SimulationParameters &sim_params, glm::vec3 positions[SimulationParameters::MAX_BOID_COUNT]) {
    for (BoidId i = 0; i < SimulationParameters::MAX_BOID_COUNT; ++i) {
        positions[i] = boids::rand_vec(
                -sim_params.aquarium_size.x / 2.f,
                sim_params.aquarium_size.x / 2.f,

                -sim_params.aquarium_size.y / 2.f,
                sim_params.aquarium_size.y / 2.f,

                -sim_params.aquarium_size.z / 2.f,
                sim_params.aquarium_size.z / 2.f
        );
    }
}

glm::vec3 boids::rand_unit_vec() {
    return glm::normalize(rand_vec(-1.f, 1.f, -1.f, 1.f, -1.f, 1.f));
}