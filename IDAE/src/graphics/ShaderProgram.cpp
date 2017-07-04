#include "ShaderProgram.h"

namespace idaeg
{
	CShaderProgram::CShaderProgram(const std::string& vertPath, const std::string& fragPath):
		_vertSource(vertPath), _fragSource(fragPath)
	{
		_programID = _load();
	}

	CShaderProgram::~CShaderProgram()
	{
		glDeleteProgram(_programID);
	}

	GLuint CShaderProgram::_load()
	{
		int success;
		char infoLog[512];

		std::string vertSource = idaeu::CFileUtils::read_file(_vertSource.c_str());
		std::string fragSource = idaeu::CFileUtils::read_file(_fragSource.c_str());

		const char* vertShaderSource = vertSource.c_str();
		const char* fragShaderSource = fragSource.c_str();

		GLint vertexShader = glCreateShader(GL_VERTEX_SHADER);
		glShaderSource(vertexShader, 1, &vertShaderSource, NULL);
		glCompileShader(vertexShader);

		glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
		if (!success)
		{
			glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
			std::cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" << infoLog << std::endl;
		}

		GLint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
		glShaderSource(fragmentShader, 1, &fragShaderSource, NULL);
		glCompileShader(fragmentShader);

		glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
		if (!success)
		{
			glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
			std::cout << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n" << infoLog << std::endl;
		}

		GLuint shaderProgram = glCreateProgram();

		glAttachShader(shaderProgram, vertexShader);
		glAttachShader(shaderProgram, fragmentShader);
		glLinkProgram(shaderProgram);

		glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
		if (!success) {
			glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
			std::cout << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n" << infoLog << std::endl;
		}

		glDeleteShader(vertexShader);
		glDeleteShader(fragmentShader);

		return shaderProgram;
	}

	GLint CShaderProgram::_getUniformLocation(const GLchar* name)
	{
		return glGetUniformLocation(_programID, name);
	}

	void CShaderProgram::SetUniform1i(const GLchar* name, const int& value)
	{
		glUniform1i(_getUniformLocation(name), value);
	}

	void CShaderProgram::SetUniform1f(const GLchar* name, const float& value)
	{
		glUniform1f(_getUniformLocation(name), value);
	}

	void CShaderProgram::SetUniform2f(const GLchar* name, const idaem::vec2& vector)
	{
		glUniform2f(_getUniformLocation(name), vector.x, vector.y);
	}

	void CShaderProgram::SetUniform3f(const GLchar* name, const idaem::vec3& vector)
	{
		glUniform3f(_getUniformLocation(name), vector.x, vector.y, vector.z);
	}

	void CShaderProgram::SetUniform4f(const GLchar* name, const idaem::vec4& vector)
	{
		glUniform4f(_getUniformLocation(name), vector.x, vector.y, vector.z, vector.w);
	}

	void CShaderProgram::SetUniform1iv(const GLchar* name, const int& value)
	{
		glUniform1iv(_getUniformLocation(name), 1, &value);
	}

	void CShaderProgram::SetUniform1fv(const GLchar* name, const float& value)
	{
		glUniform1fv(_getUniformLocation(name), 1, &value);
	}

	void CShaderProgram::SetUniform2fv(const GLchar* name, const idaem::vec2& vector)
	{
		glUniform2fv(_getUniformLocation(name), 2, vector.element);
	}

	void CShaderProgram::SetUniform3fv(const GLchar* name, const idaem::vec3& vector)
	{
		glUniform3fv(_getUniformLocation(name), 3, vector.element);
	}

	void CShaderProgram::SetUniform4fv(const GLchar* name, const idaem::vec4& vector)
	{
		glUniform4fv(_getUniformLocation(name), 4, vector.element);
	}

	void CShaderProgram::SetUniformMat2fv(const GLchar* name, const idaem::mat2& matrix)
	{
		glUniformMatrix2fv(_getUniformLocation(name), 4, GL_FALSE, matrix._1D);
	}

	void CShaderProgram::SetUniformMat3fv(const GLchar* name, const idaem::mat3& matrix)
	{
		glUniformMatrix3fv(_getUniformLocation(name), 9, GL_FALSE, matrix._1D);
	}

	void CShaderProgram::SetUniformMat4fv(const GLchar* name, const idaem::mat4& matrix)
	{
		glUniformMatrix4fv(_getUniformLocation(name), 16, GL_FALSE, matrix._1D);
	}

	void CShaderProgram::Enable()
	{
		glUseProgram(_programID);
	}

	void CShaderProgram::Disable()
	{
		glUseProgram(0);
	}
}