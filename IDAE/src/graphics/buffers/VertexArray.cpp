#include "VertexArray.h"

namespace idaeg
{
	CVertexArray::CVertexArray()
	{
		glGenVertexArrays(1, &_ArrayID);
	}

	CVertexArray::~CVertexArray()
	{
		for (size_t i = 0; i < _buffers.size(); i++)
			delete _buffers[i];

		glDeleteVertexArrays(1, &_ArrayID);
	}

	void CVertexArray::AddBuffer(CBuffer* buffer, GLuint index)
	{
		Bind();
		buffer->Bind();

		glVertexAttribPointer(index, buffer->GetComponentCount(), GL_FLOAT, GL_FALSE, 0, 0);
		glEnableVertexAttribArray(index);

		buffer->Unbind();
		Unbind();
	}

	void CVertexArray::Bind()
	{
		glBindVertexArray(_ArrayID);
	}

	void CVertexArray::Unbind()
	{
		glBindVertexArray(0);
	}
}