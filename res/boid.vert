#version 330 core
// TODO: inject boids count during parsing
#define BOIDS_COUNT 100

layout (location = 0) in vec3 aPos;

uniform vec3 u_position[BOIDS_COUNT];
uniform vec3 u_forward[BOIDS_COUNT];
uniform vec3 u_up[BOIDS_COUNT];
uniform vec3 u_right[BOIDS_COUNT];

uniform mat4 u_projection_view;

void main()
{
    mat4 rotation_matrix = mat4(vec4(u_right[gl_InstanceID], 0.), vec4(u_up[gl_InstanceID], 0.), vec4(u_forward[gl_InstanceID], 0.), vec4(0., 0., 0., 1.));
    vec4 pos = vec4(aPos + u_position[gl_InstanceID], 1.);
    pos = u_projection_view * rotation_matrix * pos;
    gl_Position = pos;
}