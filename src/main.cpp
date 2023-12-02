#define GLM_FORCE_CUDA

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "vendor/imgui/backend/imgui_impl_glfw.h"
#include "vendor/imgui/backend/imgui_impl_opengl3.h"
#include "vendor/imgui/imgui.h"

#include "shader_program.hpp"
#include "camera.hpp"
#include "gl_debug.h"
#include "primitives.h"

#include "boids.hpp"
#include "boids_cpu.hpp"
#include "boids_cuda.hpp"

#include <iostream>
#include <chrono>
#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void process_input(GLFWwindow *window);
bool process_camera_input(GLFWwindow *window, common::OrbitingCamera& camera);

const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

int main() {
    // GLFW: initialize and configure
    glfwInit();

    // Decide GL+GLSL versions
#if defined(IMGUI_IMPL_OPENGL_ES2)
    // GL ES 2.0 + GLSL 100
    const char* glsl_version = "#version 100";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
#elif defined(__APPLE__)
    // GL 3.2 + GLSL 150
    const char* glsl_version = "#version 150";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // Required on Mac
#else
    // GL 3.0 + GLSL 130
    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // 3.2+ only
    //glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+ only
#endif

    // Glfw window creation
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "[GLFW Init]: Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    // Initialize glew
    GLenum err = glewInit();
    if (GLEW_OK != err)
    {
        /* Problem: glewInit failed, something is seriously wrong. */
        std::cerr << "[GLEW Init]: " << glewGetErrorString(err) << std::endl;
        glfwTerminate();
        return -1;
    }

    // Initialize imgui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsLight();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // -------------------------------------------------------------------------

    common::ShaderProgram boids_sp("../res/boid.vert", "../res/boid.frag");
    common::ShaderProgram basic_sp("../res/basic.vert", "../res/basic.frag");

    boids::SimulationParameters sim_params;
    boids::BoidsRenderer boids_renderer;

    boids::Boids boids;
    boids::rand_aquarium_positions(sim_params, boids.position);
    boids::cpu::update_shader(boids_sp, boids.position, boids.forward, boids.up, boids.right);

    boids::cuda::GPUBoids gpu_boids = boids::cuda::GPUBoids(boids);

    common::OrbitingCamera camera(glm::vec3(0.), SCR_WIDTH, SCR_HEIGHT);
    boids_sp.bind();
    boids_sp.set_uniform_mat4f("u_projection_view", camera.get_proj() * camera.get_view());
    basic_sp.bind();
    basic_sp.set_uniform_mat4f("u_projection_view", camera.get_proj() * camera.get_view());

    common::Box aquarium;
    basic_sp.set_uniform_mat4f("u_model", glm::scale(sim_params.aquarium_size));

    // Uncomment this call to draw in wireframe polygons.
    GLCall( glPolygonMode(GL_FRONT_AND_BACK, GL_LINE) );

    std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point previous_time = current_time;

    GLCall( glEnable(GL_DEPTH_TEST) );
    GLCall( glEnable(GL_BLEND) );
    GLCall( glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA) );
    //GLCall( glEnable(GL_CULL_FACE) );
    GLCall( glCullFace(GL_FRONT) );
    GLCall( glFrontFace(GL_CCW) );

    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();
        process_input(window);
        if (process_camera_input(window, camera)) {
            boids_sp.bind();
            boids_sp.set_uniform_mat4f("u_projection_view", camera.get_proj() * camera.get_view());
            basic_sp.bind();
            basic_sp.set_uniform_mat4f("u_projection_view", camera.get_proj() * camera.get_view());
        }

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        {
            ImGui::Begin("Simulation parameters");
            ImGui::Text("Simulation average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);

            ImGui::SliderFloat("View radius", &sim_params.distance, boids::SimulationParameters::MIN_DISTANCE, 100.0f);
            ImGui::SliderFloat("Separation", &sim_params.separation, 0.0f, 5.0f);
            ImGui::SliderFloat("Alignment", &sim_params.alignment, 0.0f, 5.0f);
            ImGui::SliderFloat("Cohesion", &sim_params.cohesion, 0.0f, 5.0f);
            ImGui::SliderFloat("Min speed", &sim_params.min_speed, 0.0f, sim_params.max_speed);
            ImGui::SliderFloat("Max speed", &sim_params.max_speed, sim_params.min_speed, boids::SimulationParameters::MAX_SPEED);
            ImGui::SliderFloat("Noise", &sim_params.noise, 0.0f, 5.0f);

            ImGui::End();
        }

        ImGui::Render();

        // Calculate delta time
        current_time = std::chrono::steady_clock::now();
        std::chrono::duration<float> delta_time = std::chrono::duration_cast<std::chrono::duration<float>>(current_time - previous_time);
        previous_time = current_time;

        // Get the delta time in seconds
        float dt_as_seconds = delta_time.count();
        gpu_boids.update_simulation_with_sort(sim_params, boids, dt_as_seconds);
        boids::cpu::update_shader(boids_sp, boids.position, boids.forward, boids.up, boids.right);

//        boids::cpu::update_simulation_naive(sim_params, boids.position, boids.velocity, boids.acceleration, dt_as_seconds);
//        boids::cpu::update_basis_vectors(boids.velocity, boids.forward, boids.up, boids.right);
//        boids::cpu::update_shader(boids_sp, boids.position, boids.forward, boids.up, boids.right);

        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        boids_renderer.draw(boids_sp);
        aquarium.draw(basic_sp);

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // GLFW: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        glfwSwapBuffers(window);
    }

    // GLFW: terminate, clearing all previously allocated GLFW resources.
    glfwTerminate();
    return 0;
}

bool process_camera_input(GLFWwindow *window, common::OrbitingCamera& camera) {
    bool result = false;
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
        camera.update_azimuthal_angle(-glm::radians(2.f));
        result = true;
    }

    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
        camera.update_azimuthal_angle(glm::radians(2.f));
        result = true;
    }

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
        camera.update_polar_angle(-glm::radians(2.f));
        result = true;
    }

    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
        camera.update_polar_angle(glm::radians(2.f));
        result = true;
    }

    if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) {
        camera.update_radius(-0.8f);
        result = true;
    }

    if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) {
        camera.update_radius(0.8f);
        result = true;
    }

    return result;
}
// Process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
void process_input(GLFWwindow *window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
}

// GLFW: whenever the window size changed (by OS or user resize) this callback function executes
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // Make sure the viewport matches the new window dimensions; note that width and
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}

