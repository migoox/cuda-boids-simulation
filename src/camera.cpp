#include "camera.hpp"
#include <glm/gtx/transform.hpp>
#include <algorithm>

common::OrbitingCamera::OrbitingCamera(glm::vec3 center, float screen_width, float screen_height)
: m_center(center), m_radius(30.), m_polar_angle(glm::radians(30.)), m_azimuthal_angle(30.) {
    m_proj_mat = glm::perspectiveLH(
            0.33f * glm::pi<float>(),
            screen_width / screen_height,
            0.1f,
            1800.0f
    );
    update_view_matrix();
}

const glm::mat4 &common::OrbitingCamera::get_proj() const {
    return m_proj_mat;
}

const glm::mat4 &common::OrbitingCamera::get_view() const {
    return m_view_mat;
}

void common::OrbitingCamera::set_screen_size(float screen_width, float screen_height) {
    m_proj_mat = glm::perspectiveLH(
            0.33f * glm::pi<float>(),
            screen_width / screen_height,
            0.1f,
            1800.0f
    );
}

void common::OrbitingCamera::update_view_matrix() {
    // Convert camera position described in spherical coordinates to the cartesian coordinates
    auto eye = glm::vec3(
            m_radius * std::sin(m_polar_angle) * std::cos(m_azimuthal_angle) + m_center.x,
            m_radius * std::cos(m_polar_angle) + m_center.y,
            m_radius * std::sin(m_polar_angle) * std::sin(m_azimuthal_angle) + m_center.z
    );

    if (m_polar_angle <= 1e-4) {
        // Fix look at matrix (default up vector is invalid in this case)
        m_view_mat = glm::lookAtLH(
                eye,
                m_center,
                glm::vec3(
                        glm::rotate(
                                glm::mat4(1.0f),
                                glm::pi<float>() / 2.0f - m_azimuthal_angle,
                                glm::vec3(0.0f, 1.0f, 0.0f)
                        )
                        * glm::vec4(0.0, 0.0, -1.0, 1.0)
                )
        );
        return;
    }

    m_view_mat = glm::lookAtLH(
            eye,
            m_center,
            glm::vec3(0.0, 1.0, 0.0)
    );


}

void common::OrbitingCamera::update_radius(float delta) {
    if (m_radius + delta >= 0.5f) {
        m_radius += delta;
        update_view_matrix();
    }
}

void common::OrbitingCamera::update_polar_angle(float delta) {
    m_polar_angle = std::clamp(m_polar_angle + delta, 0.0f, 0.8f * glm::pi<float>());
    update_view_matrix();
}

void common::OrbitingCamera::update_azimuthal_angle(float delta) {
    m_azimuthal_angle += delta;
    update_view_matrix();
}






