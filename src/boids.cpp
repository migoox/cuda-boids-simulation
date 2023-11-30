#include <random>
#include <iterator>
#include <ranges>
#include <vector>
#include <execution>
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

void boids::BoidsRenderer::draw(const ShaderProgram& shader_program) const {
    shader_program.bind();
    m_mesh.bind();
    GLCall( glDrawElementsInstanced(GL_TRIANGLES, m_mesh.get_count(), GL_UNSIGNED_INT, nullptr, SimulationParameters::MAX_BOID_COUNT) );
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
    for (int i = 0; i < SimulationParameters::MAX_BOID_COUNT; ++i) {
        this->position[i] = boids::rand_vec(5., -5., 5., -5., 5., -5.);
        this->forward[i] = glm::vec3(0.f, 0.f, 1.f);
        this->up[i] = glm::vec3(0.f, 1.f, 0.f);
        this->right[i] = glm::vec3(1.f, 0.f, 0.f);

        this->velocity[i] = 0.05f * glm::normalize(boids::rand_vec(1., -1., 1., -1., 1., -1.));
        this->acceleration[i] = glm::vec3(0.f);
    }
}

void boids::cpu::update_basis_vectors(
        glm::vec3 velocity[SimulationParameters::MAX_BOID_COUNT],
        glm::vec3 forward[SimulationParameters::MAX_BOID_COUNT],
        glm::vec3 up[SimulationParameters::MAX_BOID_COUNT],
        glm::vec3 right[SimulationParameters::MAX_BOID_COUNT]
) {
    for (int i = 0; i < SimulationParameters::MAX_BOID_COUNT; ++i) {
        forward[i] = glm::normalize(velocity[i]);
        right[i] = glm::normalize(glm::cross(up[i], forward[i]));
        up[i] = glm::normalize(glm::cross(forward[i] , right[i]));
    }
}

void boids::cpu::update_shader(
        ShaderProgram &shader_program,
        glm::vec3 position[SimulationParameters::MAX_BOID_COUNT],
        glm::vec3 forward[SimulationParameters::MAX_BOID_COUNT],
        glm::vec3 up[SimulationParameters::MAX_BOID_COUNT],
        glm::vec3 right[SimulationParameters::MAX_BOID_COUNT]
) {
    shader_program.bind();
    for (int i = 0; i < SimulationParameters::MAX_BOID_COUNT; ++i) {
        shader_program.set_uniform_3f(("u_position[" + std::to_string(i) + "]").c_str(), position[i]);
        shader_program.set_uniform_3f(("u_forward[" + std::to_string(i) + "]").c_str(), forward[i]);
        shader_program.set_uniform_3f(("u_up[" + std::to_string(i) + "]").c_str(), up[i]);
        shader_program.set_uniform_3f(("u_right[" + std::to_string(i) + "]").c_str(), right[i]);
    }
}

void boids::cpu::update_simulation_naive(
        const SimulationParameters &sim_params,
        glm::vec3 position[SimulationParameters::MAX_BOID_COUNT],
        glm::vec3 velocity[SimulationParameters::MAX_BOID_COUNT],
        glm::vec3 acceleration[SimulationParameters::MAX_BOID_COUNT],
        float dt
) {
    std::ranges::iota_view indexes((size_t)0, (size_t)SimulationParameters::MAX_BOID_COUNT);
    std::for_each(std::execution::par, indexes.begin(), indexes.end(),
                  [&acceleration,
                          &velocity = std::as_const(velocity),
                          &position = std::as_const(position),
                          &sim_params = std::as_const(sim_params)
                  ](size_t boid_id) {
                      glm::vec3 separation(0.);
                      glm::vec3 avg_vel(0.);
                      glm::vec3 avg_pos(0.);
                      uint32_t neighbors_count = 0;

                      for (BoidId j = 0; j < SimulationParameters::MAX_BOID_COUNT; ++j) {
                          if (boid_id == j) {
                              continue;
                          }

                          // Skip if boid is not in the field of view
                          auto distance = glm::distance(position[boid_id], position[j]);
                          if (distance > sim_params.distance) {
                              continue;
                          }

                          separation += glm::normalize(position[boid_id] - position[j]) * sim_params.separation / distance / distance;
                          avg_vel += velocity[j];
                          avg_pos += position[j];

                          ++neighbors_count;
                      }

                      if (neighbors_count > 0) {
                          avg_vel /= float(neighbors_count);
                          avg_pos /= float(neighbors_count);
                      }

                      // Final acceleration of the current boid
                      acceleration[boid_id] =
                              sim_params.separation * separation +
                              sim_params.alignment * (avg_vel - velocity[boid_id]) +
                              sim_params.cohesion * (avg_pos - position[boid_id]) +
                              sim_params.noise * rand_unit_vec();

                      acceleration[boid_id] = 1.f * acceleration[boid_id];
                  }
    );

    // TODO: Parametrize wall and wall_force values
    float wall = 4.f;
    float wall_acc = 15.f;
    for (BoidId i = 0; i < SimulationParameters::MAX_BOID_COUNT; ++i) {
        if (position[i].x > sim_params.aquarium_size.x / 2.f - wall) {
            auto intensity = std::abs((sim_params.aquarium_size.x / 2.f - wall - position[i].x) / wall);
            acceleration[i] += intensity * glm::vec3(-wall_acc, 0.f, 0.f);
        } else if (position[i].x < -sim_params.aquarium_size.x / 2.f + wall) {
            auto intensity = std::abs((-sim_params.aquarium_size.x / 2.f + wall - position[i].x) / wall);
            acceleration[i] += intensity * glm::vec3(wall_acc, 0.f, 0.f);
        }

        if (position[i].y > sim_params.aquarium_size.y / 2.f - wall) {
            auto intensity = std::abs((sim_params.aquarium_size.y / 2.f - wall - position[i].y) / wall);
            acceleration[i] += intensity * glm::vec3(0.f, -wall_acc, 0.f);
        } else if (position[i].y < -sim_params.aquarium_size.y / 2.f + wall) {
            auto intensity = std::abs((-sim_params.aquarium_size.y / 2.f + wall - position[i].y) / wall);
            acceleration[i] += intensity * glm::vec3(0.f, wall_acc, 0.f);
        }

        if (position[i].z > sim_params.aquarium_size.z / 2.f - wall) {
            auto intensity = std::abs((sim_params.aquarium_size.z / 2.f - wall - position[i].z) / wall);
            acceleration[i] += intensity * glm::vec3(0.f, 0.f, -wall_acc);
        } else if (position[i].z < -sim_params.aquarium_size.z / 2.f + wall) {
            auto intensity = std::abs((-sim_params.aquarium_size.z / 2.f + wall - position[i].z) / wall);
            acceleration[i] += intensity * glm::vec3(0.f, 0.f, wall_acc);
        }

        velocity[i] += acceleration[i] * dt;
        if (glm::length(velocity[i]) > sim_params.max_speed) {
            velocity[i] = glm::normalize(velocity[i]) * sim_params.max_speed;
        } else if (glm::length(velocity[i]) < sim_params.min_speed){
            velocity[i] = glm::normalize(velocity[i]) * sim_params.min_speed;
        }

        position[i] += velocity[i] * dt;
    }
}

void boids::update_simulation_with_grid(
        const SimulationParameters &sim_params,
        const AquariumGrid &aquarium_grid,
        glm::vec3 position[SimulationParameters::MAX_BOID_COUNT],
        glm::vec3 velocity[SimulationParameters::MAX_BOID_COUNT],
        glm::vec3 acceleration[SimulationParameters::MAX_BOID_COUNT],
        float dt
) {
    std::ranges::iota_view indexes((size_t)0, (size_t)SimulationParameters::MAX_BOID_COUNT);
    std::for_each(std::execution::par, indexes.begin(), indexes.end(),
                  [&acceleration,
                          &velocity = std::as_const(velocity),
                          &position = std::as_const(position),
                          &sim_params = std::as_const(sim_params)
                  ](size_t boid_id) {
                      glm::vec3 separation(0.);
                      glm::vec3 avg_vel(0.);
                      glm::vec3 avg_pos(0.);
                      uint32_t neighbors_count = 0;

                      for (BoidId j = 0; j < SimulationParameters::MAX_BOID_COUNT; ++j) {
                          if (boid_id == j) {
                              continue;
                          }

                          // Skip if boid is not in the field of view
                          auto distance = glm::distance(position[boid_id], position[j]);
                          if (distance > sim_params.distance) {
                              continue;
                          }

                          separation += glm::normalize(position[boid_id] - position[j]) * sim_params.separation / distance / distance;
                          avg_vel += velocity[j];
                          avg_pos += position[j];

                          ++neighbors_count;
                      }

                      if (neighbors_count > 0) {
                          avg_vel /= float(neighbors_count);
                          avg_pos /= float(neighbors_count);
                      }

                      // Final acceleration of the current boid
                      acceleration[boid_id] =
                              sim_params.separation * separation +
                              sim_params.alignment * (avg_vel - velocity[boid_id]) +
                              sim_params.cohesion * (avg_pos - position[boid_id]) +
                              sim_params.noise * rand_unit_vec();

                      acceleration[boid_id] = 1.f * acceleration[boid_id];
                  }
    );

    // TODO: Parametrize wall and wall_force values
    float wall = 4.f;
    float wall_acc = 15.f;
    for (BoidId i = 0; i < SimulationParameters::MAX_BOID_COUNT; ++i) {
        if (position[i].x > sim_params.aquarium_size.x / 2.f - wall) {
            auto intensity = std::abs((sim_params.aquarium_size.x / 2.f - wall - position[i].x) / wall);
            acceleration[i] += intensity * glm::vec3(-wall_acc, 0.f, 0.f);
        } else if (position[i].x < -sim_params.aquarium_size.x / 2.f + wall) {
            auto intensity = std::abs((-sim_params.aquarium_size.x / 2.f + wall - position[i].x) / wall);
            acceleration[i] += intensity * glm::vec3(wall_acc, 0.f, 0.f);
        }

        if (position[i].y > sim_params.aquarium_size.y / 2.f - wall) {
            auto intensity = std::abs((sim_params.aquarium_size.y / 2.f - wall - position[i].y) / wall);
            acceleration[i] += intensity * glm::vec3(0.f, -wall_acc, 0.f);
        } else if (position[i].y < -sim_params.aquarium_size.y / 2.f + wall) {
            auto intensity = std::abs((-sim_params.aquarium_size.y / 2.f + wall - position[i].y) / wall);
            acceleration[i] += intensity * glm::vec3(0.f, wall_acc, 0.f);
        }

        if (position[i].z > sim_params.aquarium_size.z / 2.f - wall) {
            auto intensity = std::abs((sim_params.aquarium_size.z / 2.f - wall - position[i].z) / wall);
            acceleration[i] += intensity * glm::vec3(0.f, 0.f, -wall_acc);
        } else if (position[i].z < -sim_params.aquarium_size.z / 2.f + wall) {
            auto intensity = std::abs((-sim_params.aquarium_size.z / 2.f + wall - position[i].z) / wall);
            acceleration[i] += intensity * glm::vec3(0.f, 0.f, wall_acc);
        }

        velocity[i] += acceleration[i] * dt;
        if (glm::length(velocity[i]) > sim_params.max_speed) {
            velocity[i] = glm::normalize(velocity[i]) * sim_params.max_speed;
        } else if (glm::length(velocity[i]) < sim_params.min_speed){
            velocity[i] = glm::normalize(velocity[i]) * sim_params.min_speed;
        }

        position[i] += velocity[i] * dt;
    }
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

boids::CellId boids::AquariumGrid::flatten_coords(
        const boids::SimulationParameters &sim_params,
        boids::CellCoord x,
        boids::CellCoord y,
        boids::CellCoord z
) {
    float cell_size = 2 * sim_params.distance;
    boids::CellCoord grid_size_x = std::ceil(sim_params.aquarium_size.x / cell_size);
    boids::CellCoord grid_size_y = std::ceil(sim_params.aquarium_size.y / cell_size);

    return x + y * grid_size_x + z * grid_size_x * grid_size_y;
}