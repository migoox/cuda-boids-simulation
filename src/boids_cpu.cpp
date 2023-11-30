#include "boids_cpu.hpp"
#include <iterator>
#include <ranges>
#include <vector>
#include <execution>

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
        common::ShaderProgram &shader_program,
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