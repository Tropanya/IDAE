#ifndef SHADER_PROGRAM_H
#define SHADER_PROGRAM_H

#include <string>
#include <GLEW/glew.h>

#include "../utils/FileUtils.h"
#include "../math/IDAEMath.h"

/*
TODO:
++Uniforms
error check
*/

namespace idaeg
{
	class CShaderProgram
	{
	private:
		GLuint _programID;
		std::string _vertSource, _fragSource;
	private:
		GLuint _load();
		GLint _getUniformLocation(const GLchar* textUniform);
	public:
		void SetUniform1i(const GLchar* name, const int& value);
		void SetUniform1f(const GLchar* name, const float& value);
		void SetUniform2f(const GLchar* name, const idaem::vec2& vector);
		void SetUniform3f(const GLchar* name, const idaem::vec3& vector);
		void SetUniform4f(const GLchar* name, const idaem::vec4& vector);

		void SetUniform1iv(const GLchar* name, const int& value);
		void SetUniform1fv(const GLchar* name, const float& value);
		void SetUniform2fv(const GLchar* name, const idaem::vec2& vector);
		void SetUniform3fv(const GLchar* name, const idaem::vec3& vector);
		void SetUniform4fv(const GLchar* name, const idaem::vec4& vector);

		void SetUniformMat2fv(const GLchar* name, const idaem::mat2& matrix);
		void SetUniformMat3fv(const GLchar* name, const idaem::mat3& matrix);
		void SetUniformMat4fv(const GLchar* name, const idaem::mat4& matrix);
	public:
		CShaderProgram(const std::string& vertPath, const std::string& fragPath);
		~CShaderProgram();

		void Enable();
		void Disable();
	};
}// namespace idaeg

#endif