#ifndef BOIDS_SIMULATION_BOIDS_HPP
#define BOIDS_SIMULATION_BOIDS_HPP

// TODO: move to config file
#define BOIDS_COUNT 500

#include <GL/glew.h>
#include "shader_program.hpp"
#include <glm/glm.hpp>

namespace boids {
    using BoidId = uint32_t;

    class SimulationParameters {
    public:
        SimulationParameters();
        SimulationParameters(float distance, float separation, float alignment, float cohesion);

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

        ~BoidsRenderer();

        void draw(ShaderProgram &shader_program) const;

    private:
        GLuint m_vao, m_vbo, m_ebo;
        uint32_t m_count;
    };

    class Boids {
    public:
        Boids();

        // Sets random position and default orientation
        void reset();

    public:
        // Boid's position and velocity
        glm::vec3 position[BOIDS_COUNT]{};
        glm::vec3 velocity[BOIDS_COUNT]{};
        glm::vec3 acceleration[BOIDS_COUNT]{};

        // Boid's basis vectors (assuming left-handed)
        glm::vec3 forward[BOIDS_COUNT]{}; // z axis direction
        glm::vec3 up[BOIDS_COUNT]{};      // y axis direction
        glm::vec3 right[BOIDS_COUNT]{};   // x axis direction
    };

    void rand_aquarium_positions(const SimulationParameters &sim_params, glm::vec3 positions[BOIDS_COUNT]);

    // This function change the direction that the boid is facing basing on it's velocity (d = ||v||)
    void update_basis_vectors(
            glm::vec3 velocity[BOIDS_COUNT],
            glm::vec3 forward[BOIDS_COUNT],
            glm::vec3 up[BOIDS_COUNT],
            glm::vec3 right[BOIDS_COUNT]
    );

    void update_shader(
            ShaderProgram &shader_program,
            glm::vec3 position[BOIDS_COUNT],
            glm::vec3 forward[BOIDS_COUNT],
            glm::vec3 up[BOIDS_COUNT],
            glm::vec3 right[BOIDS_COUNT]
    );


    void update_simulation_naive(
            const SimulationParameters &sim_params,
            glm::vec3 position[BOIDS_COUNT],
            glm::vec3 velocity[BOIDS_COUNT],
            glm::vec3 acceleration[BOIDS_COUNT],
            float dt
    );

    glm::vec3 rand_vec(float min_x, float max_x, float min_y, float max_y, float min_z, float max_z);
    glm::vec3 rand_unit_vec();
}


#endif //BOIDS_SIMULATION_BOIDS_HPP
