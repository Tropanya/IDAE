#include "src/Display.h"
#include "src/math/Math.h"
#include "src/graphics/ShaderProgram.h"
#include "src/graphics/buffers/Buffer.h"
#include "src/graphics/buffers/IndexBuffer.h"
#include "src/graphics/buffers/VertexArray.h"

using namespace idaem;
using namespace idaeg;
using namespace std;

#define DRAW_ELEMENT 0

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
++ buffers,
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
	glViewport(0, 0, 800, 600);
	glEnable(GL_DEPTH_TEST);

	CShaderProgram* program = new CShaderProgram("res/test.vert", "res/test.frag");
	CShaderProgram* lampProgram = new CShaderProgram("res/lamp.vert", "res/lamp.frag");

	//mat4 model(1.0f);
	mat4 view;
	mat4 model;
	mat4 projection;
	mat4 ortho;

	//int b = 0;

	/*while (b < 10000000)
	{
		GLfloat angle = 20.0f * b + 5.0f;
		model = model * ToMat4(Rotate(ToQuat(model), (GLfloat)glfwGetTime() * angle, vec3(1.0f, 0.3f, 0.5f)));
		b++;
	}

	while (b < 10000000)
	{
		GLfloat angle = 20.0f * b + 5.0f;
		model = Rotate(model, (GLfloat)glfwGetTime() * angle, vec3(1.0f, 0.3f, 0.5f));
		b++;
	}*/

#if DRAW_ELEMENT
	float vertices[] = {
		 0.5f,  0.5f, 0.0f,
		 0.5f, -0.5f, 0.0f,
		-0.5f, -0.5f, 0.0f,
		-0.5f,  0.5f, 0.0f
	};

	unsigned int indices[] = {
		0, 1, 3,
		1, 2, 3
	};

	CVertexArray VAO;
	CBuffer* VBO = new CBuffer(vertices, 12, 3);
	CIndexBuffer IBO(indices, 6);

	VAO.AddBuffer(VBO, 0);
#else
	GLfloat vertices[] = {
		-0.5f, -0.5f, -0.5f,
		 0.5f, -0.5f, -0.5f,
		 0.5f,  0.5f, -0.5f,
		 0.5f,  0.5f, -0.5f,
		-0.5f,  0.5f, -0.5f,
		-0.5f, -0.5f, -0.5f,

		-0.5f, -0.5f,  0.5f,
		 0.5f, -0.5f,  0.5f,
		 0.5f,  0.5f,  0.5f,
		 0.5f,  0.5f,  0.5f,
		-0.5f,  0.5f,  0.5f,
		-0.5f, -0.5f,  0.5f,

		-0.5f,  0.5f,  0.5f,
		-0.5f,  0.5f, -0.5f,
		-0.5f, -0.5f, -0.5f,
		-0.5f, -0.5f, -0.5f,
		-0.5f, -0.5f,  0.5f,
		-0.5f,  0.5f,  0.5f,

		 0.5f,  0.5f,  0.5f,
		 0.5f,  0.5f, -0.5f,
		 0.5f, -0.5f, -0.5f,
		 0.5f, -0.5f, -0.5f,
		 0.5f, -0.5f,  0.5f,
		 0.5f,  0.5f,  0.5f,

		-0.5f, -0.5f, -0.5f,
		 0.5f, -0.5f, -0.5f,
		 0.5f, -0.5f,  0.5f,
		 0.5f, -0.5f,  0.5f,
		-0.5f, -0.5f,  0.5f,
		-0.5f, -0.5f, -0.5f,

		-0.5f,  0.5f, -0.5f,
		 0.5f,  0.5f, -0.5f,
		 0.5f,  0.5f,  0.5f,
		 0.5f,  0.5f,  0.5f,
		-0.5f,  0.5f,  0.5f,
		-0.5f,  0.5f, -0.5f
	};

	GLfloat normals[] = {
		0.0f,  0.0f, -1.0f,
		0.0f,  0.0f, -1.0f,
		0.0f,  0.0f, -1.0f,
		0.0f,  0.0f, -1.0f,
		0.0f,  0.0f, -1.0f,
		0.0f,  0.0f, -1.0f,

		0.0f,  0.0f,  1.0f,
		0.0f,  0.0f,  1.0f,
		0.0f,  0.0f,  1.0f,
		0.0f,  0.0f,  1.0f,
		0.0f,  0.0f,  1.0f,
		0.0f,  0.0f,  1.0f,

		1.0f,  0.0f,  0.0f,
		1.0f,  0.0f,  0.0f,
		1.0f,  0.0f,  0.0f,
		1.0f,  0.0f,  0.0f,
		1.0f,  0.0f,  0.0f,
		1.0f,  0.0f,  0.0f,

		1.0f,  0.0f,  0.0f,
		1.0f,  0.0f,  0.0f,
		1.0f,  0.0f,  0.0f,
		1.0f,  0.0f,  0.0f,
		1.0f,  0.0f,  0.0f,
		1.0f,  0.0f,  0.0f,

		0.0f, -1.0f,  0.0f,
		0.0f, -1.0f,  0.0f,
		0.0f, -1.0f,  0.0f,
		0.0f, -1.0f,  0.0f,
		0.0f, -1.0f,  0.0f,
		0.0f, -1.0f,  0.0f,

		0.0f,  1.0f,  0.0f,
		0.0f,  1.0f,  0.0f,
		0.0f,  1.0f,  0.0f,
		0.0f,  1.0f,  0.0f,
		0.0f,  1.0f,  0.0f,
		0.0f,  1.0f,  0.0f
	};

	CVertexArray VAO;
	CBuffer* VBO = new CBuffer(vertices, 108, 3);
	CBuffer* normal = new CBuffer(normals, 108, 3);

	CVertexArray lampVAO;

	VAO.AddBuffer(VBO, 0);
	VAO.AddBuffer(normal, 1);
	lampVAO.AddBuffer(VBO, 0);
#endif // DRAW_ELEMENT

	vec3 cubePositions[] = {
		vec3(0.0f,  0.0f,  0.0f),
		vec3(2.0f,  5.0f, -15.0f),
		vec3(-1.5f, -2.2f, -2.5f),
		vec3(-3.8f, -2.0f, -12.3f),
		vec3(2.4f, -0.4f, -3.5f),
		vec3(-1.7f,  3.0f, -7.5f),
		vec3(1.3f, -2.0f, -2.5f),
		vec3(1.5f,  2.0f, -2.5f),
		vec3(1.5f,  0.2f, -1.5f),
		vec3(-1.3f,  1.0f, -1.5f)
	};

	projection = Perspective(45.0f, 800.0f / 600.0f, 0.1f, 100.0f);
	ortho = Ortho(-1.0f, 1.0f, -1.0f, 1.0f);
	float zOffset = 0;
	
	while (window->Closed() == window->IsKeyPressed(GLFW_KEY_ESCAPE))
	{
		//glClearColor(0.2f, 0.3f, 0.3f, 1.0f);

		window->Clear();
#if DRAW_ELEMENT
		program->SetUniformMat4fv("ml_matrix", Rotate(model, 45.0f, vec3(0.0f, 0.0f, 1.0f)));
		program->SetUniformMat4fv("vw_matrix", view);
		program->SetUniformMat4fv("pr_matrix", ortho);

		VAO.Bind();
		IBO.Bind();
		glDrawElements(GL_TRIANGLES, IBO.GetCount(), GL_UNSIGNED_INT, 0);
		IBO.Unbind();
		VAO.Unbind();
#else
		zOffset = Sin((GLfloat)glfwGetTime()) * 2.0f;

		//vec3 lightPos(1.0f, 0.0f, zOffset);
		vec3 lightPos(1.0f, 1.0f, 1.0f);
		mat4 lampModel;
		lampModel = Translate(lampModel, lightPos);
		lampModel = Scale(lampModel, vec3(0.2f));

		program->Enable();
		program->SetUniform3f("objectColor", vec3(1.0f, 0.5f, 0.2f));
		program->SetUniform3f("lightColor", vec3(1.0f, 1.0f, 1.0f));
		program->SetUniform3f("lightPos", lightPos);
		//program->SetUniformMat4fv("ml_matrix", Rotate(model, (GLfloat)glfwGetTime() * 50.0f, vec3(0.5f, 1.0f, 0.0f)));
		program->SetUniformMat4fv("ml_matrix", model * ToMat4(Rotate(quat(), (GLfloat)glfwGetTime() * 50.0f, vec3(0.5f, 1.0f, 0.0f))));
		//program->SetUniformMat4fv("ml_matrix", model);
		program->SetUniformMat4fv("vw_matrix", Translate(view, vec3(0.0f, 0.0f, -3.0f)));
		program->SetUniformMat4fv("pr_matrix", projection);

		VAO.Bind();
		glDrawArrays(GL_TRIANGLES, 0, 36);
		//glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		VAO.Unbind();

		//program->Disable();

		lampProgram->Enable();
		lampProgram->SetUniformMat4fv("ml_matrix", lampModel);
		lampProgram->SetUniformMat4fv("vw_matrix", Translate(view, vec3(0.0f, 0.0f, -3.0f)));
		lampProgram->SetUniformMat4fv("pr_matrix", projection);

		lampVAO.Bind();
		glDrawArrays(GL_TRIANGLES, 0, 36);
		lampVAO.Unbind();
#endif // DRAW_ELEMENT

		window->Update();
	}

	program->Disable();

	delete program;
	delete window;

	return 0;
}