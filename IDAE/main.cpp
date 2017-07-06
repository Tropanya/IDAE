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
	//glViewport(0, 0, 800, 600);
	
	CShaderProgram* program = new CShaderProgram("res/test.vert", "res/test.frag");

	mat4 model(1.0f);
	mat4 view(1.0f);
	mat4 projection;
	mat4 ortho;

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

	CVertexArray VAO;
	CBuffer* VBO = new CBuffer(vertices, 108, 3);

	VAO.AddBuffer(VBO, 0);
#endif // DRAW_ELEMENT

	projection = Perspective(45.0f, 800.0f / 600.0f, 0.1f, 100.0f);
	ortho = Ortho(-1.0f, 1.0f, -1.0f, 1.0f);
	
	while (window->Closed() == window->IsKeyPressed(GLFW_KEY_ESCAPE))
	{
		glClearColor(0.2f, 0.3f, 0.3f, 1.0f);

		window->Clear();
		program->Enable();
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
		program->SetUniformMat4fv("ml_matrix", model * ToMat4(Rotate(quat(), (GLfloat)glfwGetTime() * 50.0f, vec3(0.5f, 1.0f, 0.0f))) /*Rotate(model, (GLfloat)glfwGetTime() * 50.0f, vec3(0.5f, 1.0f, 0.0f)*/);
		program->SetUniformMat4fv("vw_matrix", Translate(view, vec3(0.0f, 0.0f, -3.0f)));
		program->SetUniformMat4fv("pr_matrix", projection);

		VAO.Bind();
		glDrawArrays(GL_TRIANGLES, 0, 36);
		VAO.Unbind();
#endif // DRAW_ELEMENT

		window->Update();
	}

	program->Disable();

	delete program;
	delete window;

	return 0;
}