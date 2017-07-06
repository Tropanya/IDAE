#ifndef VERTEX_ARRAY_H
#define VERTEX_ARRAY_H

#include <vector>

#include "Buffer.h"

namespace idaeg
{
	class CVertexArray
	{
	private:
		GLuint _ArrayID;
		std::vector<CBuffer*> _buffers;
	public:
		CVertexArray();
		~CVertexArray();

		void AddBuffer(CBuffer* buffer, GLuint index);

		void Bind();
		void Unbind();
	};
}

#endif // VERTEX_ARRAY_H