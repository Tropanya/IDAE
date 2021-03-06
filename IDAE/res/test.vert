#version 330 core

layout (location = 0) in vec3 position;
layout (location = 1) in vec3 normal;

uniform mat4 pr_matrix;
uniform mat4 vw_matrix;
uniform mat4 ml_matrix;

out vec3 FragPos;
out vec3 Normal;

void main()
{
   gl_Position = pr_matrix * vw_matrix * ml_matrix * vec4(position, 1.0f);
   FragPos = vec3(ml_matrix * vec4(position, 1.0f));
   Normal = vec3(ml_matrix * vec4(normal, 1.0f));
   //Normal = normal;
}