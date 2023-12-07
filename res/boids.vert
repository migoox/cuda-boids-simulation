#version 330 core

layout (location = 0) in vec3 a_pos;
layout (location = 1) in vec4 a_boid_pos;
layout (location = 2) in vec4 a_boid_forward;
layout (location = 3) in vec4 a_boid_up;
layout (location = 4) in vec4 a_boid_right;

uniform mat4 u_projection_view;

void main()
{
    mat4 model_matrix = mat4(vec4(a_boid_right.xyz, 0.), vec4(a_boid_up.xyz, 0.), vec4(a_boid_forward.xyz, 0.), a_boid_pos);
    vec4 pos = vec4(a_pos, 1.);

    gl_Position = u_projection_view * model_matrix * pos;
}