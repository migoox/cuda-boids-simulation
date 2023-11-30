#ifndef BOIDS_SIMULATION_BOIDS_HPP
#define BOIDS_SIMULATION_BOIDS_HPP

#include <GL/glew.h>
#include "shader_program.hpp"
#include <glm/glm.hpp>
#include "primitives.h"

namespace boids {
    using BoidId = size_t;

    class SimulationParameters {
    public:
        SimulationParameters();
        SimulationParameters(float distance, float separation, float alignment, float cohesion);

    public:
        constexpr static const size_t MAX_BOID_COUNT = 500;

        constexpr static const size_t MAX_AQUARIUM_SIZE_X = 100;
        constexpr static const size_t MAX_AQUARIUM_SIZE_Y = 100;
        constexpr static const size_t MAX_AQUARIUM_SIZE_Z = 100;

        constexpr static const float MIN_DISTANCE = 0.5f;

        // This formula works as long as MIN_DISTANCE = 0.5f
        constexpr static const size_t MAX_CELL_COUNT = MAX_AQUARIUM_SIZE_X * MAX_AQUARIUM_SIZE_Y * MAX_AQUARIUM_SIZE_Z;

    public:
        float distance;
        float separation;
        float alignment;
        float cohesion;

        float max_speed;
        float min_speed;

        float noise;

        glm::vec3 aquarium_size;
    };

    class BoidsRenderer {
    public:
        // Initializes boids data
        BoidsRenderer();

        void draw(const ShaderProgram &shader_program) const;

    private:
        common::Mesh m_mesh;

    };

    class Boids {
    public:
        Boids();

        // Sets random position and default orientation
        void reset();

    public:
        // Boid's position and velocity
        glm::vec3 position[SimulationParameters::MAX_BOID_COUNT]{};
        glm::vec3 velocity[SimulationParameters::MAX_BOID_COUNT]{};
        glm::vec3 acceleration[SimulationParameters::MAX_BOID_COUNT]{};

        // Boid's basis vectors (assuming left-handed)
        glm::vec3 forward[SimulationParameters::MAX_BOID_COUNT]{}; // z axis direction
        glm::vec3 up[SimulationParameters::MAX_BOID_COUNT]{};      // y axis direction
        glm::vec3 right[SimulationParameters::MAX_BOID_COUNT]{};   // x axis direction
    };

    using CellId = size_t;
    using CellInfoId = size_t;
    using CellCoord = size_t;

    class AquariumGrid {
    public:
        static boids::CellId flatten_coords(const SimulationParameters& sim_params, CellCoord x, CellCoord y, CellCoord z);

    public:
        // Stores boid's cell id.
        CellId boid_cell_id[SimulationParameters::MAX_BOID_COUNT];

        // Cell id -> cell info id -> count/start in boid_cell_id

        // Stores cell info id for each cell id. Cell info id allows querying counts and starts arrays.
        CellInfoId cell_info_id[SimulationParameters::MAX_CELL_COUNT];

        // Stores count of boids inside the boid_cell_id array.
        size_t count[SimulationParameters::MAX_BOID_COUNT];

        // Stores starting index of all elements in the queried cell.
        BoidId start[SimulationParameters::MAX_BOID_COUNT];
    };


    namespace cpu {
        // This function change the direction that the boid is facing basing on it's velocity (d = ||v||)
        void update_basis_vectors(
                glm::vec3 velocity[SimulationParameters::MAX_BOID_COUNT],
                glm::vec3 forward[SimulationParameters::MAX_BOID_COUNT],
                glm::vec3 up[SimulationParameters::MAX_BOID_COUNT],
                glm::vec3 right[SimulationParameters::MAX_BOID_COUNT]
        );

        void update_shader(
                ShaderProgram &shader_program,
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

    void update_simulation_with_grid(
            const SimulationParameters &sim_params,
            const AquariumGrid &aquarium_grid,
            glm::vec3 *position, glm::vec3 *velocity, glm::vec3 *acceleration, float dt
    );


    void rand_aquarium_positions(const SimulationParameters &sim_params, glm::vec3 positions[SimulationParameters::MAX_BOID_COUNT]);
    glm::vec3 rand_vec(float min_x, float max_x, float min_y, float max_y, float min_z, float max_z);
    glm::vec3 rand_unit_vec();
}


#endif //BOIDS_SIMULATION_BOIDS_HPP
