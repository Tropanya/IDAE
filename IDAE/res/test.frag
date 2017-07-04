#version 330 core

in vec3 vertexColor;
in vec4 pos;

uniform vec2 light_pos;

out vec4 color;

void main()
{
	float itensity = 0.5f / length(pos.xy - light_pos);

	color = vec4(vertexColor, 1.0f) * itensity;
}