#ifndef BOIDS_SIMULATION_BOIDS_CUDA_H
#define BOIDS_SIMULATION_BOIDS_CUDA_H
#include "boids_cpu.hpp"

namespace boids::cuda {
    using CellId = size_t;
    using CellInfoId = size_t;
    using CellCoord = size_t;

    class GPUBoids {
    public:
        GPUBoids() = delete;
        ~GPUBoids();
        explicit GPUBoids(const Boids& boids);

    private:
        glm::vec3 *m_dev_position{};
        glm::vec3 *m_dev_velocity{};
        glm::vec3 *m_dev_acceleration{};

        glm::vec3 *m_dev_forward{};
        glm::vec3 *m_dev_up{};
        glm::vec3 *m_dev_right{};
    };
}


#endif //BOIDS_SIMULATION_BOIDS_CUDA_H
