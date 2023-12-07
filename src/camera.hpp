#ifndef BOIDS_SIMULATION_CAMERA_HPP
#define BOIDS_SIMULATION_CAMERA_HPP
#include <glm/glm.hpp>

namespace common {
    // Camera interface
    class ICamera {
    public:
        virtual const glm::mat4& get_proj() const = 0;
        virtual const glm::mat4& get_view() const = 0;
        virtual void set_screen_size(float screen_width, float screen_height) = 0;
    };

    class OrbitingCamera : public ICamera {
    public:
        OrbitingCamera() = delete;

        OrbitingCamera(glm::vec3 center, float screen_width, float screen_height);

        const glm::mat4& get_proj() const override;
        const glm::mat4& get_view() const override;
        void set_screen_size(float screen_width, float screen_height) override;


        void update_radius(float delta);

        // Angles should be in radians
        void update_polar_angle(float delta);

        // Angles should be in radians
        void update_azimuthal_angle(float delta);


    private:
        void update_view_matrix();

    private:
        glm::mat4 m_proj_mat{};
        glm::mat4 m_view_mat{};

        glm::vec3 m_center{};

        float m_radius{};
        float m_polar_angle{};
        float m_azimuthal_angle{};
    };
}

#endif //BOIDS_SIMULATION_CAMERA_HPP
