#include <random>
#include "boids.hpp"
#include "gl_debug.h"

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
: m_vao(0), m_vbo(0), m_ebo(0), m_count(0) {

    // Initialize OpenGL
    GLCall( glGenVertexArrays(1, &m_vao) );
    GLCall( glGenBuffers(1, &m_vbo) );
    GLCall( glGenBuffers(1, &m_ebo) );

    // Let a boid face the direction based on forward vector in lh
    float vertices[] = {
            0.3f,  0.f, -0.3f,
            -0.3f, 0.f, -0.3f,
            0.f, 0.f, 0.6f,
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
    std::uniform_real_distribution<float> dist_x(min_x, max_x);
    std::uniform_real_distribution<float> dist_y(min_y, max_y);
    std::uniform_real_distribution<float> dist_z(min_z, max_z);

    float x = dist_x(gen);
    float y = dist_y(gen);
    float z = dist_z(gen);

    return {x, y, z};
}

boids::Boids::Boids() {
    this->reset();
}

void boids::Boids::reset() {
    for (int i = 0; i < BOIDS_COUNT; ++i) {
        this->position[i] = boids::rand_vec(5., -5., 5., -5., 5., -5.);
        this->forward[i] = glm::vec3(0.f, 0.f, 1.f);
        this->up[i] = glm::vec3(0.f, 1.f, 0.f);
        this->right[i] = glm::vec3(1.f, 0.f, 0.f);

        this->velocity[i] = 0.05f * glm::normalize(boids::rand_vec(1., -1., 1., -1., 1., -1.));
        this->acceleration[i] = glm::vec3(0.f);
    }
}

void boids::update_basis_vectors(
        glm::vec3 velocity[BOIDS_COUNT],
        glm::vec3 forward[BOIDS_COUNT],
        glm::vec3 up[BOIDS_COUNT],
        glm::vec3 right[BOIDS_COUNT]
) {
    for (int i = 0; i < BOIDS_COUNT; ++i) {
        forward[i] = glm::normalize(velocity[i]);
        right[i] = glm::normalize(glm::cross(up[i], forward[i]));
        up[i] = glm::normalize(glm::cross(forward[i] , right[i]));
    }
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

void boids::update_simulation_naive(
        const SimulationParameters &sim_params,
        glm::vec3 position[BOIDS_COUNT],
        glm::vec3 velocity[BOIDS_COUNT],
        glm::vec3 acceleration[BOIDS_COUNT],
        float dt
) {
    glm::vec3 separation, avg_vel, avg_pos;
    uint32_t neighbors_count;

    for (BoidId i = 0; i < BOIDS_COUNT; ++i) {
        separation = glm::vec3(0.);
        avg_vel = glm::vec3(0.);
        avg_pos = glm::vec3(0.);
        neighbors_count = 0;

        for (BoidId j = 0; j < BOIDS_COUNT; ++j) {
            if (i == j) {
                continue;
            }

            // Skip if boid is not in the field of view
            auto distance = glm::distance(position[i], position[j]);
            if (distance > sim_params.distance) {
                continue;
            }

            separation += glm::normalize(position[i] - position[j]) * sim_params.separation / distance / distance;
            avg_vel += velocity[j];
            avg_pos += position[j];

            ++neighbors_count;
        }

        if (neighbors_count > 0) {
            avg_vel /= float(neighbors_count);
            avg_pos /= float(neighbors_count);
        }

        // Final acceleration of the current boid
        acceleration[i] =
                sim_params.separation * separation +
                sim_params.alignment * (avg_vel - velocity[i]) +
                sim_params.cohesion * (avg_pos - position[i]) +
                sim_params.noise * rand_unit_vec();

        acceleration[i] = 1.f * acceleration[i];

    }

    for (BoidId i = 0; i < BOIDS_COUNT; ++i) {

        velocity[i] += acceleration[i] * dt;
        if (glm::length(velocity[i]) > sim_params.max_speed) {
            velocity[i] = glm::normalize(velocity[i]) * sim_params.max_speed;
        } else if (glm::length(velocity[i]) < sim_params.min_speed){
            velocity[i] = glm::normalize(velocity[i]) * sim_params.min_speed;
        }

        position[i] += velocity[i] * dt;

         if (position[i].x > sim_params.aquarium_size.x / 2.f) {
            position[i].x = -sim_params.aquarium_size.x / 2.f;
        } else if (position[i].x < -sim_params.aquarium_size.x / 2.f) {
            position[i].x = sim_params.aquarium_size.x / 2.f;
        }

        if (position[i].y > sim_params.aquarium_size.y / 2.f) {
            position[i].y = -sim_params.aquarium_size.y / 2.f;
        } else if (position[i].y < -sim_params.aquarium_size.y / 2.f) {
            position[i].y = sim_params.aquarium_size.y / 2.f;
        }

        if (position[i].z > sim_params.aquarium_size.z / 2.f) {
            position[i].z = -sim_params.aquarium_size.z / 2.f;
        } else if (position[i].z < -sim_params.aquarium_size.z / 2.f) {
            position[i].z = sim_params.aquarium_size.z / 2.f;
        }

    }
}

void boids::rand_aquarium_positions(const boids::SimulationParameters &sim_params, glm::vec3 positions[BOIDS_COUNT]) {
    for (BoidId i = 0; i < BOIDS_COUNT; ++i) {
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
