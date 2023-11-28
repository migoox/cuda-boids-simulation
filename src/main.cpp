#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "vendor/imgui/backend/imgui_impl_glfw.h"
#include "vendor/imgui/backend/imgui_impl_opengl3.h"
#include "vendor/imgui/imgui.h"

#include "shader_program.hpp"
#include "boids.hpp"
#include "camera.hpp"
#include "gl_debug.h"

#include <iostream>
#include <chrono>
#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow *window);
bool process_camera_input(GLFWwindow *window, common::OrbitingCamera& camera);

const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

int main()
{
    // glfw: initialize and configure
    // ------------------------------
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

    ShaderProgram shader_program("../res/boid.vert", "../res/boid.frag");

    boids::SimulationParameters sim_params;
    boids::BoidsRenderer boids_renderer;
    boids::Boids boids;

    common::OrbitingCamera camera(glm::vec3(0.), SCR_WIDTH, SCR_HEIGHT);

    boids::rand_aquarium_positions(sim_params, boids.position);

    boids::update_basis_vectors(boids.velocity, boids.forward, boids.up, boids.right);
    boids::update_shader(shader_program, boids.position, boids.forward, boids.up, boids.right);

    glm::mat4 proj = glm::perspectiveLH(
            float(glm::radians(90.)),
            float(SCR_WIDTH) / float(SCR_HEIGHT),
            .1f,
            100.f
    );

    glm::mat4 view = glm::lookAtLH(
            glm::vec3(0., 0., -5.),
            glm::vec3(0., 0., 0.),
            glm::vec3(0., 1., 0.)
    );

    shader_program.bind();
    shader_program.set_uniform_mat4f("u_projection_view", camera.get_proj() * camera.get_view());

    // uncomment this call to draw in wireframe polygons.
    GLCall( glPolygonMode(GL_FRONT_AND_BACK, GL_LINE) );

    std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point previous_time = current_time;

    // render loop
    // -----------
    GLCall( glEnable(GL_DEPTH_TEST) );
    while (!glfwWindowShouldClose(window))
    {
        // input
        // -----
        glfwPollEvents();
        processInput(window);
        if (process_camera_input(window, camera)) {
            shader_program.bind();
            shader_program.set_uniform_mat4f("u_projection_view", camera.get_proj() * camera.get_view());
        }

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // 2. Show a simple window that we create ourselves. We use a Begin/End pair to create a named window.
        {
            ImGui::Begin("Simulation parameters");
            ImGui::Text("Simulation average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);

            ImGui::SliderFloat("View radius", &sim_params.distance, 0.0f, 100.0f);
            ImGui::SliderFloat("Separation", &sim_params.separation, 0.0f, 1.0f);
            ImGui::SliderFloat("Alignment", &sim_params.alignment, 0.0f, 1.0f);
            ImGui::SliderFloat("Cohesion", &sim_params.cohesion, 0.0f, 1.0f);

            ImGui::End();
        }

        ImGui::Render();

        // Calculate delta time
        current_time = std::chrono::steady_clock::now();
        std::chrono::duration<float> delta_time = std::chrono::duration_cast<std::chrono::duration<float>>(current_time - previous_time);
        previous_time = current_time;

        // Get the delta time in seconds
        float dt_as_seconds = delta_time.count();

        // boids::update_simulation(boids.position, boids.velocity, glm::vec3(0.0, 0.0, 1.0), dt_as_seconds);
//        boids::update_simulation_naive(sim_params, boids.position, boids.velocity, boids.acceleration, dt_as_seconds);
//        boids::update_basis_vectors(boids.velocity, boids.forward, boids.up, boids.right);
//        boids::update_shader(shader_program, boids.position, boids.forward, boids.up, boids.right);

        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Draw triangle
        boids_renderer.draw(shader_program);

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        // -------------------------------------------------------------------------------
        glfwSwapBuffers(window);
    }

    // glfw: terminate, clearing all previously allocated GLFW resources.
    // ------------------------------------------------------------------
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
        camera.update_radius(-0.2f);
        result = true;
    }

    if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) {
        camera.update_radius(0.2f);
        result = true;
    }

    return result;
}
// Process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
void processInput(GLFWwindow *window)
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

