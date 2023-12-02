#ifndef BOIDS_SIMULATION_BOIDS_CUDA_HPP
#define BOIDS_SIMULATION_BOIDS_CUDA_HPP
#include "boids_cpu.hpp"

namespace boids::cuda {
    using CellId = uint32_t;
    using CellCoord = uint32_t;

    struct CellCoords {
        CellCoord x, y, z;
    };

    class GPUBoids {
    public:
        GPUBoids() = delete;
        ~GPUBoids();
        explicit GPUBoids(const Boids& boids);

        void update_simulation_with_sort(const SimulationParameters& params, Boids &boids, float dt);
    private:
        glm::vec3 *m_dev_position{};
        glm::vec3 *m_dev_velocity{};
        glm::vec3 *m_dev_acceleration{};

        glm::vec3 *m_dev_forward{};
        glm::vec3 *m_dev_up{};
        glm::vec3 *m_dev_right{};

        // cell_id -> boid_id
        CellId *m_dev_cell_id;
        BoidId *m_dev_boid_id;
        SimulationParameters *m_dev_sim_params;
    };
}


#endif //BOIDS_SIMULATION_BOIDS_CUDA_HPP
