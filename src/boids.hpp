#ifndef BOIDS_SIMULATION_BOIDS_HPP
#define BOIDS_SIMULATION_BOIDS_HPP

#include <GL/glew.h>
#include "shader_program.hpp"
#include <glm/glm.hpp>
#include <vector>
#include "primitives.h"
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>


namespace boids {
    using BoidId = uint32_t;

    class SimulationParameters {
    public:
        SimulationParameters();
        SimulationParameters(float distance, float separation, float alignment, float cohesion);

    public:
        constexpr static const size_t MAX_BOID_COUNT = 50000;

        constexpr static const float MAX_AQUARIUM_SIZE_X = 300.f;
        constexpr static const float MAX_AQUARIUM_SIZE_Y = 300.f;
        constexpr static const float MAX_AQUARIUM_SIZE_Z = 300.f;

        constexpr static const float MIN_DISTANCE = 1.f;

        constexpr static const float MIN_SPEED = 0.5f;
        constexpr static const float MAX_SPEED = 5.f;

        // This formula works as long as MIN_DISTANCE = 1.f
        constexpr static const size_t MAX_CELL_COUNT = MAX_AQUARIUM_SIZE_X * MAX_AQUARIUM_SIZE_Y * MAX_AQUARIUM_SIZE_Z;

        constexpr static const size_t MAX_OBSTACLES_COUNT = 8;
        constexpr static const float MAX_OBSTACLE_RADIUS = 10.f;
        constexpr static const float MIN_OBSTACLE_RADIUS = 1.f;

    public:
        int boids_count;

        float distance;
        float separation;
        float alignment;
        float cohesion;

        float max_speed;
        float min_speed;

        float noise;

        glm::vec3 aquarium_size;
    };

    struct BoidsOrientation {
        std::vector<glm::vec4> forward; // z axis direction
        std::vector<glm::vec4> up;      // y axis direction
        std::vector<glm::vec4> right;   // x axis direction
    };

    class Boids {
    public:
        Boids() = delete;
        Boids(const SimulationParameters& sim_params);

        // Sets random position and default orientation
        void reset(const SimulationParameters& sim_params);

    public:
        // Boid's simulation properties
        std::vector<glm::vec3> velocity;
        std::vector<glm::vec3> acceleration;

        // Boid orientation
        std::vector<glm::vec4> position;
        // Boid's basis vectors (assuming left-handed)
        BoidsOrientation orientation;
    };

    class BoidsRenderer {
    public:
        // Initializes boids data
        BoidsRenderer();

        void draw(const common::ShaderProgram &shader_program, int count) const;
        void set_vbos(const SimulationParameters &params, const std::vector<glm::vec4> &position, const BoidsOrientation &orientation);
        void cuda_register_vbos(cudaGraphicsResource** positions, cudaGraphicsResource** forward, cudaGraphicsResource** up, cudaGraphicsResource** right) const;
        
    private:
        common::Mesh m_mesh;

        GLuint m_pos_vbo_id, m_forward_vbo_id, m_up_vbo_id, m_right_vbo_id;
    };

    class Obstacles {
    public:
        Obstacles();
        void push(glm::vec3 pos, float radius);
        void remove(size_t elem);

        const float* get_radius_array() const { return m_radius.data(); }
        const glm::vec3* get_pos_array() const { return m_pos.data(); }

        float &radius(size_t elem);
        glm::vec3 &pos(size_t elem);

        size_t count() const { return m_radius.size(); }

        void draw(common::ShaderProgram& program);
    private:
        std::vector<float> m_radius;
        std::vector<glm::vec3> m_pos;

        common::Box m_box;
    };

    glm::vec3 rand_vec(float min_x, float max_x, float min_y, float max_y, float min_z, float max_z);
    glm::vec3 rand_unit_vec();
}


#endif //BOIDS_SIMULATION_BOIDS_HPP
