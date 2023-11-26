# Include ImGui source files
set(IMGUI_DIR imgui)
file(GLOB IMGUI_SOURCES
        "imgui/*.h"
        "imgui/*.cpp"
        "imgui/backend/*.h"
        "imgui/backend/*.cpp"
)

# Add the source files to your executable or library
add_library(ImGui ${IMGUI_SOURCES})
target_include_directories(ImGui PUBLIC ${IMGUI_DIR})

# Include GLFW headers
target_include_directories(ImGui PUBLIC ../../dependencies/include/)