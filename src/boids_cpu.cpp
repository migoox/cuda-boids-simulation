#include "boids_cpu.hpp"
#include <iterator>
#include <ranges>
#include <vector>
#include <execution>

void boids::cpu::update_simulation_naive(
        const SimulationParameters &sim_params,
        glm::vec4 position[SimulationParameters::MAX_BOID_COUNT],
        glm::vec3 velocity[SimulationParameters::MAX_BOID_COUNT],
        glm::vec3 acceleration[SimulationParameters::MAX_BOID_COUNT],
        boids::BoidsOrientation& orientation,
        float dt
) {
    std::ranges::iota_view indexes((size_t)0, (size_t)SimulationParameters::MAX_BOID_COUNT);
    std::for_each(std::execution::par, indexes.begin(), indexes.end(),
                  [&acceleration,
                          &velocity = std::as_const(velocity),
                          &position = std::as_const(position),
                          &sim_params = std::as_const(sim_params)
                  ](size_t b_id) {
                      glm::vec3 separation(0.);
                      glm::vec3 avg_vel(0.);
                      glm::vec3 avg_pos(0.);
                      uint32_t neighbors_count = 0;

                      for (BoidId other_id = 0; other_id < SimulationParameters::MAX_BOID_COUNT; ++other_id) {
                          if (other_id == b_id) {
                              continue;
                          }

                          auto distance2 = glm::dot(position[b_id] - position[other_id], position[b_id] - position[other_id]);
                          if (distance2 > sim_params.distance * sim_params.distance) {
                              continue;
                          }

                          separation += glm::vec3(glm::normalize(position[b_id] - position[other_id]) / distance2);
                          avg_vel += velocity[other_id];
                          avg_pos += glm::vec3(position[other_id]);

                          ++neighbors_count;
                      }

                      if (neighbors_count > 0) {
                          avg_vel /= float(neighbors_count);
                          avg_pos /= float(neighbors_count);
                      }

                      // Final acceleration of the current boid
                      acceleration[b_id] =
                              sim_params.separation * separation +
                              sim_params.alignment * (avg_vel - velocity[b_id]) +
                              sim_params.cohesion * (avg_pos - glm::vec3(position[b_id])) +
                              sim_params.noise * rand_unit_vec();
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

        position[i] += glm::vec4(velocity[i] * dt, 0.f);
    }

    // Update basis vectors (orientation)
    for (BoidId i = 0; i < SimulationParameters::MAX_BOID_COUNT; ++i) {
        orientation.forward[i] = glm::vec4(glm::normalize(velocity[i]), 0.f);
        orientation.right[i] = glm::vec4(glm::normalize(glm::cross(glm::vec3(orientation.up[i]), glm::vec3(orientation.forward[i]))), 0.f);
        orientation.up[i] = glm::vec4(glm::normalize(glm::cross(glm::vec3(orientation.forward[i]) , glm::vec3(orientation.right[i]))), 0.f);
    }
}