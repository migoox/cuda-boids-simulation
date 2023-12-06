#version 330 core
#define MAX_OBSTACLES_COUNT 8
#define SQRT2 1.41421356237f
layout (location = 0) in vec3 a_pos;

uniform vec3 u_pos[MAX_OBSTACLES_COUNT];
uniform float u_radius[MAX_OBSTACLES_COUNT];

uniform mat4 u_projection_view;

void main() {
    gl_Position = u_projection_view * mat4(
    vec4(SQRT2*u_radius[gl_InstanceID], 0.f, 0.f, 0.f),
    vec4(0.f, SQRT2*u_radius[gl_InstanceID], 0.f, 0.f),
    vec4(0.f, 0.f, SQRT2*u_radius[gl_InstanceID], 0.f),
    vec4(u_pos[gl_InstanceID], 1.f)) * vec4(a_pos, 1.f);
}
