#version 330 core

uniform vec3 objectColor;
uniform vec3 lightColor;
uniform vec3 lightPos;

in vec3 FragPos;
in vec3 Normal;

out vec4 color;

void main()
{
	float ambientStrength = 0.1;
    vec3 ambient = ambientStrength * lightColor;

	vec3 norm = normalize(Normal);
	vec3 lightDir = normalize(lightPos - FragPos);

	float diff = max(dot(norm, lightDir), 0.0f);
	vec3 diffuse = diff * lightColor;

    vec3 result = (ambient + diffuse) * objectColor;

	color = vec4(result, 1.0f);
}