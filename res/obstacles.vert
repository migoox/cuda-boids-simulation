#version 330 core
#define MAX_OBSTACLES_COUNT 8
layout (location = 0) in vec3 a_pos;

uniform vec3 u_pos[MAX_OBSTACLES_COUNT];
uniform float u_radius[MAX_OBSTACLES_COUNT];

uniform mat4 u_projection_view;

void main() {
    gl_Position = u_projection_view * mat4(
    vec4(2*u_radius[gl_InstanceID], 0.f, 0.f, 0.f),
    vec4(0.f, 2*u_radius[gl_InstanceID], 0.f, 0.f),
    vec4(0.f, 0.f, 2*u_radius[gl_InstanceID], 0.f),
    vec4(vec3(2*u_radius[gl_InstanceID]) * u_pos[gl_InstanceID], 1.f)) * vec4(a_pos, 1.f);

//    gl_Position = vec4(a_pos, 1.f);
}
