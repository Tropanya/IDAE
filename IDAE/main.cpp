#include "src/Display.h"
#include "src/math/IDAEMath.h"
#include "src/graphics/ShaderProgram.h"

using namespace idaem;
using namespace idaeg;
using namespace std;

//const char *vertexShaderSource = "#version 330 core\n"
//"layout (location = 0) in vec3 aPos;\n"
//"void main()\n"
//"{\n"
//"   gl_Position = vec4(aPos.x, aPos.y, aPos.z, 1.0);\n"
//"}\0";
//const char *fragmentShaderSource = "#version 330 core\n"
//"out vec4 FragColor;\n"
//"void main()\n"
//"{\n"
//"   FragColor = vec4(1.0f, 0.5f, 0.2f, 1.0f);\n"
//"}\n\0";
//
//const char *fragmentShaderSource2 = "#version 330 core\n"
//"out vec4 FragColor;\n"
//"void main()\n"
//"{\n"
//"   FragColor = vec4(1.0f, 1.0f, 0.0f, 1.0f);\n"
//"}\n\0";

/*
TODO for engine:
graphics,
core,
input,
cross platform,
logic,
res manager,
math,
physics,
audio,
network
*/

/*
TODO for graphics:
shaders,
buffers,
camera,
textures,
2Drender,
3Drender,
lights,
shadows,
font
*/

int main()
{
	CDisplay* window = new CDisplay("Engine test", 800, 600);

	/*vec3 v1(-0.5, -0.5, 0.5);
	vec3 v2( 0.5, -0.5, 0.5);
	vec3 v3(-0.5,  0.5, 0.5);
	vec3 v4( 0.5,  0.5, 0.5);

	vec3 z(0.0f);

	z = Normalize(v1);
	cout << z << endl;

	vec3 n1 = Normal(v1, v2, v3);
	vec3 n2 = Normal(v2, v1, v3);
	vec3 n3 = Normal(v3, v1, v2);
	vec3 n4 = Normal(v4, v2, v3);

	cout << n1 << ", " << n2 << ", " << n3 << ", " << n4 << endl;*/

	/*int vertexShader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
	glCompileShader(vertexShader);

	int success;
	char infoLog[512];
	glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
	if (!success)
	{
		glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
		std::cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" << infoLog << std::endl;
	}

	int fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
	glCompileShader(fragmentShader);

	glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
	if (!success)
	{
		glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
		std::cout << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n" << infoLog << std::endl;
	}

	int fragmentShader2 = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(fragmentShader2, 1, &fragmentShaderSource2, NULL);
	glCompileShader(fragmentShader2);

	glGetShaderiv(fragmentShader2, GL_COMPILE_STATUS, &success);
	if (!success)
	{
		glGetShaderInfoLog(fragmentShader2, 512, NULL, infoLog);
		std::cout << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n" << infoLog << std::endl;
	}

	int shaderProgram = glCreateProgram();
	glAttachShader(shaderProgram, vertexShader);
	glAttachShader(shaderProgram, fragmentShader);
	glLinkProgram(shaderProgram);

	glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
	if (!success) {
		glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
		std::cout << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n" << infoLog << std::endl;
	}

	int shaderProgram2 = glCreateProgram();
	glAttachShader(shaderProgram2, vertexShader);
	glAttachShader(shaderProgram2, fragmentShader2);
	glLinkProgram(shaderProgram2);

	glGetProgramiv(shaderProgram2, GL_LINK_STATUS, &success);
	if (!success) {
		glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
		std::cout << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n" << infoLog << std::endl;
	}
	glDeleteShader(vertexShader);
	glDeleteShader(fragmentShader);
	glDeleteShader(fragmentShader2);*/

	CShaderProgram* program = new CShaderProgram("res/test.vert", "res/test.frag");

	CQuaternion q(AngleAxis(90.0f, CVector3(0.0f, 0.0f, 1.0f)));

	float a = Angle(q);

	float b(ToDegrees(a));

	float vertices1[] = {
		 0.5f, -0.5f, 0.0f, 1.0f, 0.0f, 0.0f,
		-0.5f, -0.5f, 0.0f, 0.0f, 1.0f, 0.0f,
		-0.5f,  0.5f, 0.0f, 1.0f, 1.0f, 0.0f,
		 0.5f,  0.5f, 0.0f, 0.0f, 0.0f, 1.0f
	};

	unsigned int indices[] = {
		0, 1, 3,
		1, 2, 3
	};
	unsigned int VBO1, VAO1, EBO;
	//unsigned int VBO2, VAO2/*, EBO*/;

	glGenVertexArrays(1, &VAO1);
	glGenBuffers(1, &VBO1);
	glGenBuffers(1, &EBO);

	glBindVertexArray(VAO1);

	glBindBuffer(GL_ARRAY_BUFFER, VBO1);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices1), vertices1, GL_STATIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);

	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3*sizeof(float)));
	glEnableVertexAttribArray(1);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	mat4 ortho = idaem::Ortho(-2.0f, 2.0f, -2.0f, 2.0f);
	mat4 persp = idaem::Perspective(45.0f, 4 / 3, -1.0f, 1.0f);
	vec2 light = vec2(0.0f, 0.0f);

	program->Enable();
	program->SetUniformMat4fv("pr_matrix", Rotate(ortho, 45.0f, vec3(0.0f, 0.0f, 1.0f)));
	program->SetUniform2f("light_pos", light);

	//glGenVertexArrays(1, &VAO2);
	//glGenBuffers(1, &VBO2);

	//glBindVertexArray(VAO2);

	/*glBindBuffer(GL_ARRAY_BUFFER, VBO2);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices2), vertices2, GL_STATIC_DRAW);*/

	//glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	//glEnableVertexAttribArray(0);

	//glBindBuffer(GL_ARRAY_BUFFER, 0);
	//glBindVertexArray(0);

	while (window->Closed() == window->IsKeyPressed(GLFW_KEY_ESCAPE))
	{
		glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

		window->Clear();

		GLfloat timeValue = (float)glfwGetTime();
		GLfloat moved = (idaem::Sin(timeValue) / 2);

		cout << moved << endl;

		program->SetUniform1f("xOffset", moved);

		//program->SetUniform4f("ourColor", vec4(0.0f, greenValue, 0.0f, 1.0f));
		
		//program->Enable();
		//program->SetUniformMat4fv("pr_matrix", ortho);
		//program->SetUniformMat4fv("ml_matrix", idaem::Translate(mat4(1.0f), vec3(4, 3, 0)));
		glBindVertexArray(VAO1);
		glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
		
		//program->Enable();
		//glBindVertexArray(VAO2);
		//glDrawArrays(GL_TRIANGLES, 0, 3);
		//glBindVertexArray(0);

		window->Update();
	}

	glDeleteVertexArrays(1, &VAO1);
	glDeleteBuffers(1, &VBO1);
	//glDeleteVertexArrays(1, &VAO2);
	//glDeleteBuffers(1, &VBO2);
	//glDeleteBuffers(1, &EBO);
	program->Disable();

	delete window;

	return 0;
}