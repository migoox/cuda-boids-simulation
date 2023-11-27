#version 330 core
layout (location = 0) in vec3 aPos;

uniform vec3 u_positions[100];
uniform mat4 u_projection;
uniform mat4 u_view;

void main()
{
    vec4 pos = vec4(aPos + u_positions[gl_InstanceID], 1.);
    pos = u_projection * u_view * pos;
    gl_Position = pos;
}