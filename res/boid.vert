#version 330 core
// TODO: inject boids count during parsing
#define BOIDS_COUNT 500

layout (location = 0) in vec3 a_pos;

uniform vec3 u_position[BOIDS_COUNT];
uniform vec3 u_forward[BOIDS_COUNT];
uniform vec3 u_up[BOIDS_COUNT];
uniform vec3 u_right[BOIDS_COUNT];

uniform mat4 u_projection_view;

void main()
{
    mat4 model_matrix = mat4(vec4(u_right[gl_InstanceID], 0.), vec4(u_up[gl_InstanceID], 0.), vec4(u_forward[gl_InstanceID], 0.), vec4(u_position[gl_InstanceID], 1.));
//    mat4 model_matrix = mat4(vec4(1., .0, .0, 0.), vec4(0., 1., 0., 0.), vec4(0., 0., 1., 0.), vec4(u_position[gl_InstanceID], 1.));

    vec4 pos = vec4(a_pos, 1.);
    gl_Position = u_projection_view * model_matrix * pos;
}