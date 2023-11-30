#include "boids_cuda.hpp"
#include <iostream>

__device__ boids::cuda::CellId
boids::cuda::flatten_coords(const boids::SimulationParameters &sim_params, boids::cuda::CellCoord x,
                                          boids::cuda::CellCoord y, boids::cuda::CellCoord z) {
    float cell_size = 2 * sim_params.distance;
    CellCoord grid_size_x = std::ceil(sim_params.aquarium_size.x / cell_size);
    CellCoord grid_size_y = std::ceil(sim_params.aquarium_size.y / cell_size);

    return x + y * grid_size_x + z * grid_size_x * grid_size_y;
}

boids::cuda::GPUBoids::GPUBoids(const boids::Boids& boids) {
    // Allocate memory on the device using cudaMalloc
//    cudaError_t cudaStatus = cudaMalloc((void**)&deviceMemory, size);
//
//    if (cudaStatus != cudaSuccess) {
//        std::cerr << "[CUDA]: cudaMalloc failed: " << cudaGetErrorString(cudaStatus) << std::endl;
//        return 1;
//    }
}
