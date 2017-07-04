#version 330 core

layout (location = 0) in vec3 position;
layout (location = 1) in vec3 color;

uniform float xOffset;

uniform mat4 pr_matrix;
uniform mat4 vw_matrix = mat4(1.0f);
uniform mat4 ml_matrix = mat4(1.0f);

out vec3 vertexColor;
out vec4 pos;

void main()
{
   gl_Position = pr_matrix * vw_matrix * ml_matrix * vec4(position.x + xOffset, position.y, position.z, 1.0f);
   vertexColor = color;
   pos = vec4(position, 1.0f);
}