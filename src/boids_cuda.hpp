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
        explicit GPUBoids(const Boids& boids, const BoidsRenderer& renderer);
        explicit GPUBoids(const Boids& boids);

        void update_simulation_with_sort(const SimulationParameters& params, Boids &boids, float dt);
        void reset(const SimulationParameters& params);

    private:
        void init_default(const Boids& boids);
        void init_with_gl(const Boids& boids, const BoidsRenderer& renderer);

        void move_boids_data_to_cpu(Boids &boids);

        void swap_buffers();

    private:
        glm::vec4 *m_dev_position_old{};
        glm::vec3 *m_dev_velocity_old{};
        glm::vec4 *m_dev_position{};
        glm::vec3 *m_dev_velocity{};

        BoidsOrientation *m_dev_orient{};

        // cell_id -> boid_id
        CellId *m_dev_cell_id;
        BoidId *m_dev_boid_id;
        SimulationParameters *m_dev_sim_params;

    };
}


#endif //BOIDS_SIMULATION_BOIDS_CUDA_HPP
