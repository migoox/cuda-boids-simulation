#ifndef BOIDS_SIMULATION_BOIDS_CPU_HPP
#define BOIDS_SIMULATION_BOIDS_CPU_HPP
#include "boids.hpp"

namespace boids::cpu {


    // This function change the direction that the boid is facing basing on it's velocity (d = ||v||)
    void update_basis_vectors(
            glm::vec3 velocity[SimulationParameters::MAX_BOID_COUNT],
            glm::vec3 forward[SimulationParameters::MAX_BOID_COUNT],
            glm::vec3 up[SimulationParameters::MAX_BOID_COUNT],
            glm::vec3 right[SimulationParameters::MAX_BOID_COUNT]
    );

    void update_shader(
            common::ShaderProgram &shader_program,
            glm::vec3 position[SimulationParameters::MAX_BOID_COUNT],
            glm::vec3 forward[SimulationParameters::MAX_BOID_COUNT],
            glm::vec3 up[SimulationParameters::MAX_BOID_COUNT],
            glm::vec3 right[SimulationParameters::MAX_BOID_COUNT]
    );


    void update_simulation_naive(
            const SimulationParameters &sim_params,
            glm::vec3 position[SimulationParameters::MAX_BOID_COUNT],
            glm::vec3 velocity[SimulationParameters::MAX_BOID_COUNT],
            glm::vec3 acceleration[SimulationParameters::MAX_BOID_COUNT],
            float dt
    );
}


#endif //BOIDS_SIMULATION_BOIDS_CPU_HPP
