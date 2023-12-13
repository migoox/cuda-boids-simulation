#ifndef BOIDS_SIMULATION_BOIDS_CPU_HPP
#define BOIDS_SIMULATION_BOIDS_CPU_HPP
#include "boids.hpp"

namespace boids::cpu {
    void update_simulation_naive(
            const SimulationParameters &sim_params,
            const Obstacles& obstacles,
            std::vector<glm::vec4> &position,
            std::vector<glm::vec3> &velocity,
            std::vector<glm::vec3> &acceleration,
            BoidsOrientation &orientation,
            float dt
    );
}


#endif //BOIDS_SIMULATION_BOIDS_CPU_HPP
