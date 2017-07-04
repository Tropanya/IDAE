#include "FileUtils.h"

namespace idaeu
{
	std::string CFileUtils::read_file(const char* filePath)
	{
		std::ifstream file;
		file.open(filePath);
		std::stringstream fileStream;
		fileStream << file.rdbuf();
		file.close();

		return fileStream.str();
	}
}