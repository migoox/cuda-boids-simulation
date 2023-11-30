#ifndef BOIDS_SIMULATION_BOIDS_CUDA_H
#define BOIDS_SIMULATION_BOIDS_CUDA_H
#include "boids.hpp"
#include "boids_cpu.hpp"

namespace boids::cuda {
    using CellId = size_t;
    using CellInfoId = size_t;
    using CellCoord = size_t;

    __device__ CellId flatten_coords(const SimulationParameters& sim_params, CellCoord x, CellCoord y, CellCoord z);

    // Stores boid's cell id.
    __device__ CellId boid_cell_id[SimulationParameters::MAX_BOID_COUNT];

    // Cell id -> cell info id -> count/start in boid_cell_id

    // Stores cell info id for each cell id. Cell info id allows querying counts and starts arrays.
    __device__ CellInfoId cell_info_id[SimulationParameters::MAX_CELL_COUNT];

    // Stores count of boids inside the boid_cell_id array.
    __device__ size_t count[SimulationParameters::MAX_BOID_COUNT];

    // Stores starting index of all elements in the queried cell.
    __device__ BoidId start[SimulationParameters::MAX_BOID_COUNT];


    class GPUBoids {
    public:
        GPUBoids() = delete;
        explicit GPUBoids(const Boids& boids);

    private:
        Boids* m_boids;
    };
}


#endif //BOIDS_SIMULATION_BOIDS_CUDA_H
