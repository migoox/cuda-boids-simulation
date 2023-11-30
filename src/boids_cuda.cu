#include "boids_cuda.h"
#include "cuda_runtime.h"

#include <iostream>

// Stores boid's cell id.
__device__ boids::cuda::CellId boid_cell_id[boids::SimulationParameters::MAX_BOID_COUNT];

// Cell id -> cell info id -> count/start in boid_cell_id

// Stores cell info id for each cell id. Cell info id allows querying counts and starts arrays.
__device__ boids::cuda::CellInfoId cell_info_id[boids::SimulationParameters::MAX_CELL_COUNT];

// Stores count of boids inside the boid_cell_id array.
__device__ size_t count[boids::SimulationParameters::MAX_BOID_COUNT];

// Stores starting index of all elements in the queried cell.
__device__ boids::BoidId start[boids::SimulationParameters::MAX_BOID_COUNT];

__device__ boids::cuda::CellId flatten_coords(const boids::SimulationParameters& sim_params, boids::cuda::CellCoord x, boids::cuda::CellCoord y, boids::cuda::CellCoord z) {
    float cell_size = 2 * sim_params.distance;
    boids::cuda::CellCoord grid_size_x = std::ceil(sim_params.aquarium_size.x / cell_size);
    boids::cuda::CellCoord grid_size_y = std::ceil(sim_params.aquarium_size.y / cell_size);

    return x + y * grid_size_x + z * grid_size_x * grid_size_y;
}

void check_cuda_error(const cudaError_t &cuda_status, const char *msg) {
    if (cuda_status != cudaSuccess) {
        std::cerr << msg << cudaGetErrorString(cuda_status) << std::endl;
        std::terminate();
    }
};

boids::cuda::GPUBoids::GPUBoids(const boids::Boids& boids) {
    // Allocate memory on the device using cudaMalloc
    size_t array_size = SimulationParameters::MAX_BOID_COUNT * sizeof(glm::vec3);

    cudaError_t cuda_status;
    cuda_status = cudaMalloc((void**)&m_dev_position, array_size);
    check_cuda_error(cuda_status, "[CUDA]: cudaMalloc failed: ");
    cuda_status = cudaMalloc((void**)&m_dev_velocity, array_size);
    check_cuda_error(cuda_status, "[CUDA]: cudaMalloc failed: ");
    cuda_status = cudaMalloc((void**)&m_dev_acceleration, array_size);
    check_cuda_error(cuda_status, "[CUDA]: cudaMalloc failed: ");
    cuda_status = cudaMalloc((void**)&m_dev_forward, array_size);
    check_cuda_error(cuda_status, "[CUDA]: cudaMalloc failed: ");
    cuda_status = cudaMalloc((void**)&m_dev_up, array_size);
    check_cuda_error(cuda_status, "[CUDA]: cudaMalloc failed: ");
    cuda_status = cudaMalloc((void**)&m_dev_right, array_size);
    check_cuda_error(cuda_status, "[CUDA]: cudaMalloc failed: ");

    cuda_status = cudaMemcpy(m_dev_position, boids.position, array_size, cudaMemcpyHostToDevice);
    check_cuda_error(cuda_status, "[CUDA]: cudaMemcpy failed: ");
    cuda_status = cudaMemcpy(m_dev_velocity, boids.velocity, array_size, cudaMemcpyHostToDevice);
    check_cuda_error(cuda_status, "[CUDA]: cudaMemcpy failed: ");
    cuda_status = cudaMemcpy(m_dev_acceleration, boids.acceleration, array_size, cudaMemcpyHostToDevice);
    check_cuda_error(cuda_status, "[CUDA]: cudaMemcpy failed: ");
    cuda_status = cudaMemcpy(m_dev_forward, boids.forward, array_size, cudaMemcpyHostToDevice);
    check_cuda_error(cuda_status, "[CUDA]: cudaMemcpy failed: ");
    cuda_status = cudaMemcpy(m_dev_up, boids.up, array_size, cudaMemcpyHostToDevice);
    check_cuda_error(cuda_status, "[CUDA]: cudaMemcpy failed: ");
    cuda_status = cudaMemcpy(m_dev_right, boids.right, array_size, cudaMemcpyHostToDevice);
    check_cuda_error(cuda_status, "[CUDA]: cudaMemcpy failed: ");
}

boids::cuda::GPUBoids::~GPUBoids() {
    cudaFree(m_dev_position);
    cudaFree(m_dev_velocity);
    cudaFree(m_dev_forward);
    cudaFree(m_dev_forward);
    cudaFree(m_dev_up);
    cudaFree(m_dev_right);
}
