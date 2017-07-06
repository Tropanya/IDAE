#ifndef BUFFER_H
#define BUFFER_H

#include <GLEW/glew.h>

namespace idaeg
{
	class CBuffer
	{
	private:
		GLuint _bufferID;
		GLuint _componentCount;
	public:
		CBuffer(GLfloat* data, GLsizei count, GLuint componentCount);
		~CBuffer();

		void Bind();
		void Unbind();

		GLuint GetComponentCount();
	};
}

#endif // BUFFER_H