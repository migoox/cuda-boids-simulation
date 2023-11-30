cmake_minimum_required(VERSION 3.26)

# Allow cuda and cpp languages
project(boids_simulation LANGUAGES CXX CUDA)

# Set cpp standard
set(CMAKE_CUDA_STANDARD 20)
set(CMAKE_CXX_STANDARD 20)

# Find patterns with these patterns and add them to sources
file(GLOB SOURCES
        "src/*.cpp"
        "src/*.hpp"
        "src/*.h"
        "src/*.cu"
)

add_subdirectory(src/vendor)

# Set include and lib directories
include_directories(dependencies/include)

# Add lib directory
if(WIN64)
    set(LIB_DIR ${CMAKE_SOURCE_DIR}/dependencies/lib/win)
    file(GLOB EXTERNAL_LIBS "${LIB_DIR}/*.dll")
else()
    set(LIB_DIR ${CMAKE_SOURCE_DIR}/dependencies/lib/linux)
    file(GLOB EXTERNAL_LIBS "${LIB_DIR}/*.so")
endif()

# Add ImGui source files to your project

# Create an executable from the source files
add_executable(boids_simulation ${SOURCES})

# Link the executable to the external libraries
target_link_libraries(boids_simulation
        ImGui
        ${EXTERNAL_LIBS}
)

# Generate separate object files for each CUDA source file
set_target_properties(boids_simulation PROPERTIES
        CUDA_SEPARABLE_COMPILATION ON)