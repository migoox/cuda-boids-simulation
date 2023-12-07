#define GLM_FORCE_CUDA
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <filesystem>
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
#include <glm/gtx/transform.hpp>

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void process_input(GLFWwindow *window);
bool process_camera_input(GLFWwindow *window, common::OrbitingCamera& camera, float dt);
bool list_view_getter(void* data, int index, const char** output);

const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

static unsigned int curr_src_width = SCR_WIDTH;
static unsigned int curr_src_height = SCR_HEIGHT;

static bool app_resized = false;

enum Solution {
    CPUNaive,
    GPUCUDANaive,
    GPUCUDASortVar1,
    GPUCUDASortVar2
};

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

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    const GLubyte* vendor = glGetString(GL_VENDOR);
    const GLubyte* renderer = glGetString(GL_RENDERER);
    if (vendor && renderer) {
        std::cout << "[GL]: Vendor: " << vendor << std::endl;
        std::cout << "[GL]: Renderer: " << renderer << std::endl;
    }

    // -------------------------------------------------------------------------

    std::string executable_dir = std::filesystem::path(__FILE__).parent_path().string();
    common::ShaderProgram boids_sp(executable_dir + "/../res/boids.vert", executable_dir + "/../res/boids.frag");
    common::ShaderProgram basic_sp(executable_dir + "/../res/basic.vert", executable_dir + "/../res/basic.frag");
    common::ShaderProgram obstacles_sp(executable_dir + "/../res/obstacles.vert",executable_dir +  "/../res/basic.frag");

    Solution curr_solution = Solution::GPUCUDASortVar2;

    boids::SimulationParameters sim_params(4.5f, 0.85f, 2.f, 1.4f);

    // Default settings
    boids::SimulationParameters new_sim_params;
    sim_params.aquarium_size.x = 90.f;
    sim_params.aquarium_size.y = 90.f;
    sim_params.aquarium_size.z = 90.f;
    sim_params.boids_count = 10000;
    new_sim_params = sim_params;

    boids::Obstacles obstacles;

    boids::BoidsRenderer boids_renderer;
    boids::Boids boids(sim_params);
    boids_renderer.set_vbos(sim_params, boids.position, boids.orientation);

    boids::cuda_gpu::GPUBoids gpu_boids = boids::cuda_gpu::GPUBoids(boids, boids_renderer);

    common::OrbitingCamera camera(glm::vec3(0.), SCR_WIDTH, SCR_HEIGHT);
    boids_sp.bind();
    boids_sp.set_uniform_mat4f("u_projection_view", camera.get_proj() * camera.get_view());
    obstacles_sp.bind();
    obstacles_sp.set_uniform_mat4f("u_projection_view", camera.get_proj() * camera.get_view());
    basic_sp.bind();
    basic_sp.set_uniform_mat4f("u_projection_view", camera.get_proj() * camera.get_view());

    common::Box aquarium;
    basic_sp.bind();
    basic_sp.set_uniform_mat4f("u_model", glm::scale(sim_params.aquarium_size));

    std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point previous_time = current_time;

    GLCall( glEnable(GL_DEPTH_TEST) );
    GLCall( glEnable(GL_BLEND) );
    GLCall( glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA) );
    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();
        process_input(window);

        if (app_resized) {
            camera.set_screen_size(static_cast<float>(curr_src_width), static_cast<float>(curr_src_height));
        }

        if (process_camera_input(window, camera, dt)) {
            boids_sp.bind();
            boids_sp.set_uniform_mat4f("u_projection_view", camera.get_proj() * camera.get_view());
            basic_sp.bind();
            basic_sp.set_uniform_mat4f("u_projection_view", camera.get_proj() * camera.get_view());
            obstacles_sp.bind();
            obstacles_sp.set_uniform_mat4f("u_projection_view", camera.get_proj() * camera.get_view());
        }

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        {
            static const char* items[] = { "CPU: Naive", "GPU CUDA: Naive", "GPU CUDA: Sort Var1", "GPU CUDA: Sort Var2"};
            ImGui::Begin("Simulation");

            // Display floating text
            ImGui::SetNextWindowPos(ImVec2(0, 0)); // Set position for the text
            ImGui::Begin("Floating Text", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings);

            // Display floating text
            ImGui::Text("%.1f FPS", io.Framerate);
            ImGui::Text("Solution: %s", items[curr_solution]);
            ImGui::Text("Boids count: %d", sim_params.boids_count);
            ImGui::Text("Aquarium size: (%.2f, %.2f, %.2f)", sim_params.aquarium_size.x, sim_params.aquarium_size.y, sim_params.aquarium_size.z);

            ImGui::End();

            if (ImGui::CollapsingHeader("New", ImGuiTreeNodeFlags_DefaultOpen)) {
                static Solution curr_item = curr_solution;

                if (ImGui::Button("Start")) {
                    curr_solution = curr_item;
                    sim_params.aquarium_size = new_sim_params.aquarium_size;
                    sim_params.boids_count = new_sim_params.boids_count;

                    basic_sp.bind();
                    basic_sp.set_uniform_mat4f("u_model", glm::scale(sim_params.aquarium_size));

                    if (curr_item == Solution::CPUNaive) {
                        boids.reset(sim_params);
                    } else {
                        gpu_boids.reset(sim_params);
                    }
                }

                ImGui::Combo("Solution", reinterpret_cast<int *>(&curr_item), items, IM_ARRAYSIZE(items));

                ImGui::InputInt("Boids count", &new_sim_params.boids_count, 0, 1000, ImGuiInputTextFlags_CharsDecimal);
                new_sim_params.boids_count = (new_sim_params.boids_count < 0) ? 0 : new_sim_params.boids_count;
                new_sim_params.boids_count = (new_sim_params.boids_count > boids::SimulationParameters::MAX_BOID_COUNT) ? boids::SimulationParameters::MAX_BOID_COUNT : new_sim_params.boids_count;

                ImGui::SliderFloat("Aquarium size X", &new_sim_params.aquarium_size.x, 10.f, boids::SimulationParameters::MAX_AQUARIUM_SIZE_X);
                ImGui::SliderFloat("Aquarium size Y", &new_sim_params.aquarium_size.y, 10.f, boids::SimulationParameters::MAX_AQUARIUM_SIZE_Y);
                ImGui::SliderFloat("Aquarium size Z", &new_sim_params.aquarium_size.z, 10.f, boids::SimulationParameters::MAX_AQUARIUM_SIZE_Z);
            }

            if (ImGui::CollapsingHeader("Parameters", ImGuiTreeNodeFlags_DefaultOpen)) {
                ImGui::SliderFloat("View radius", &sim_params.distance, boids::SimulationParameters::MIN_DISTANCE, 100.0f);
                ImGui::SliderFloat("Separation", &sim_params.separation, 0.0f, 5.0f);
                ImGui::SliderFloat("Alignment", &sim_params.alignment, 0.0f, 5.0f);
                ImGui::SliderFloat("Cohesion", &sim_params.cohesion, 0.0f, 5.0f);
                ImGui::SliderFloat("Min speed", &sim_params.min_speed, 0.0f, sim_params.max_speed);
                ImGui::SliderFloat("Max speed", &sim_params.max_speed, sim_params.min_speed, boids::SimulationParameters::MAX_SPEED);
                ImGui::SliderFloat("Noise", &sim_params.noise, 0.0f, 5.0f);
            }

            if (ImGui::CollapsingHeader("Obstacles", ImGuiTreeNodeFlags_DefaultOpen)) {
                static int selected_list_item = -1; // Index of the selected item (-1 means no item is selected)

                if (ImGui::Button("Add")) {
                    obstacles.push(glm::vec3(0.f, 0.f, 0.f), 5.f);
                }
                ImGui::SameLine();
                if (ImGui::Button("Remove")) {
                    if (selected_list_item >= 0 && obstacles.count() > 0) {
                        obstacles.remove(selected_list_item);
                    }
                }

                if (obstacles.count() > 0) {
                    ImGui::ListBox(
                            "##List of obstacles",
                            &selected_list_item,
                            list_view_getter,
                            (void*)obstacles.get_pos_array(),
                            obstacles.count()
                    );
                }

                if (selected_list_item >= 0 && obstacles.count() > 0) {
                    glm::vec3 pos = obstacles.pos(selected_list_item);
                    ImGui::SliderFloat("X", &pos.x, -sim_params.aquarium_size.x / 2.f, sim_params.aquarium_size.x / 2.f);
                    ImGui::SliderFloat("Y", &pos.y, -sim_params.aquarium_size.y / 2.f, sim_params.aquarium_size.y / 2.f);
                    ImGui::SliderFloat("Z", &pos.z, -sim_params.aquarium_size.z / 2.f, sim_params.aquarium_size.z / 2.f);
                    obstacles.pos(selected_list_item) = pos;

                    float radius = obstacles.radius(selected_list_item);
                    ImGui::SliderFloat("Radius", &radius, boids::SimulationParameters::MIN_OBSTACLE_RADIUS, boids::SimulationParameters::MAX_OBSTACLE_RADIUS);
                    obstacles.radius(selected_list_item) = radius;
                }
            }

            ImGui::End();
        }

        ImGui::Render();

        // Calculate delta time
        current_time = std::chrono::steady_clock::now();
        std::chrono::duration<float> delta_time = std::chrono::duration_cast<std::chrono::duration<float>>(current_time - previous_time);
        previous_time = current_time;

        // Get the delta time in seconds
        float dt_as_seconds = delta_time.count();
        if (curr_solution == Solution::CPUNaive) {
            boids::cpu::update_simulation_naive(sim_params, boids.position, boids.velocity, boids.acceleration, boids.orientation, dt_as_seconds);
        } else if (curr_solution == Solution::GPUCUDASortVar1) {
            gpu_boids.update_simulation_with_sort(sim_params, obstacles, boids, dt_as_seconds, 0);
        } else if (curr_solution == Solution::GPUCUDASortVar2) {
            gpu_boids.update_simulation_with_sort(sim_params, obstacles, boids, dt_as_seconds, 1);
        } else {
            gpu_boids.update_simulation_naive(sim_params, obstacles, boids, dt_as_seconds);
        }


        boids_renderer.set_vbos(sim_params, boids.position, boids.orientation);

        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        GLCall( glPolygonMode(GL_FRONT_AND_BACK, GL_FILL) );
        obstacles.draw(obstacles_sp);

        GLCall( glPolygonMode(GL_FRONT_AND_BACK, GL_LINE) );
        boids_renderer.draw(boids_sp, sim_params.boids_count);
        aquarium.draw(basic_sp);

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // GLFW: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        glfwSwapBuffers(window);
    }

    // GLFW: terminate, clearing all previously allocated GLFW resources.
    glfwTerminate();
    return 0;
}

bool process_camera_input(GLFWwindow *window, common::OrbitingCamera& camera, float dt) {
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
    app_resized = true;
    curr_src_width = width;
    curr_src_height = height;
}

bool list_view_getter(void* data, int index, const char** output) {
    static std::string curr_name = "Obstacle";
    curr_name = "Obstacle " + std::to_string(index);
    *output = curr_name.c_str();
    return true;
}
