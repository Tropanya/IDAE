#include "Buffer.h"

namespace idaeg
{
	CBuffer::CBuffer(GLfloat* data, GLsizei count, GLuint componentCount):
		_componentCount(componentCount)
	{
		glGenBuffers(1, &_bufferID);
		glBindBuffer(GL_ARRAY_BUFFER, _bufferID);
		glBufferData(GL_ARRAY_BUFFER, count * sizeof(GLfloat), data, GL_STATIC_DRAW);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}

	CBuffer::~CBuffer()
	{
		glDeleteBuffers(1, &_bufferID);
	}

	void CBuffer::Bind()
	{
		glBindBuffer(GL_ARRAY_BUFFER, _bufferID);
	}

	void CBuffer::Unbind()
	{
		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}
	GLuint CBuffer::GetComponentCount()
	{
		return _componentCount;
	}
}