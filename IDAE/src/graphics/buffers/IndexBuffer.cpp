#include "IndexBuffer.h"

namespace idaeg
{
	CIndexBuffer::CIndexBuffer(GLuint* data, GLsizei count) :
		_count(count)
	{
		glGenBuffers(1, &_bufferID);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _bufferID);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, count * sizeof(GLuint), data, GL_STATIC_DRAW);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	}

	CIndexBuffer::~CIndexBuffer()
	{
		glDeleteBuffers(1, &_bufferID);
	}

	void CIndexBuffer::Bind()
	{
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _bufferID);
	}

	void CIndexBuffer::Unbind()
	{
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	}
	GLuint CIndexBuffer::GetCount()
	{
		return _count;
	}
}