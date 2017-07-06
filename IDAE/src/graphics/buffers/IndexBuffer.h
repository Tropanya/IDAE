#ifndef INDEX_BUFFER_H
#define INDEX_BUFFER_H

#include <GLEW/glew.h>

namespace idaeg
{
	class CIndexBuffer
	{
	private:
		GLuint _bufferID;
		GLuint _count;
	public:
		CIndexBuffer(GLuint* data, GLsizei count);
		~CIndexBuffer();

		void Bind();
		void Unbind();

		GLuint GetCount();
	};
}

#endif // INDEX_BUFFER_H