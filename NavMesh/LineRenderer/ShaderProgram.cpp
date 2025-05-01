#include "ShaderProgram.h"
#include "Utilities.h"
#include <iostream>

ShaderProgram::ShaderProgram(std::string vertexFilename, std::string fragmentFilename)
{
	loadedSuccessfully = true;

	vertexShader = glCreateShader(GL_VERTEX_SHADER);
	fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);

	shaderProgram = glCreateProgram();
	
	std::string vertexSource = LoadFileAsString(vertexFilename);
	std::string fragmentSource = LoadFileAsString(fragmentFilename);

	if (vertexSource == "" || fragmentSource == "")
	{
		std::cout << "Failed to open one or more shader source files.\n";
		std::cout << "Is your working directory set up correctly?\n";
		loadedSuccessfully = false;
	}
	else
	{
		const char* vertexSourceC = vertexSource.c_str();

		glShaderSource(vertexShader, 1, &vertexSourceC, nullptr);
		glCompileShader(vertexShader);

		GLchar errorLog[512];
		GLint success = 0;
		glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
		if (!success)
		{
			//Something failed with the vertex shader compilation
			std::cout << "Vertex shader " << vertexFilename << " failed with error:\n";
			glGetShaderInfoLog(vertexShader, 512, nullptr, errorLog);
			std::cout << errorLog << '\n';
			loadedSuccessfully = false;
		}
		else
		{
			std::cout << "\"" << vertexFilename << "\" compiled successfully.\n";
		}

		const char* fragmentSourceC = fragmentSource.c_str();

		glShaderSource(fragmentShader, 1, &fragmentSourceC, nullptr);
		glCompileShader(fragmentShader);

		glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
		if (!success)
		{
			//Something failed with the fragment shader compilation
			std::cout << "Fragment shader " << fragmentFilename << " failed with error:\n";
			glGetShaderInfoLog(fragmentShader, 512, nullptr, errorLog);
			std::cout << errorLog << '\n';
			loadedSuccessfully = false;
		}
		else
		{
			std::cout << "\"" << fragmentFilename << "\" compiled successfully.\n";
		}
		glAttachShader(shaderProgram, vertexShader);
		glAttachShader(shaderProgram, fragmentShader);
		glLinkProgram(shaderProgram);
		glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
		if (success == GL_FALSE)
		{
			std::cout << "Error linking shaders \"" << vertexFilename << "\" and \"" << fragmentFilename << "\"\n";
			glGetProgramInfoLog(shaderProgram, 512, nullptr, errorLog);
			std::cout << errorLog << std::endl;
			loadedSuccessfully = false;
		}
	}

	if (loadedSuccessfully)
	{
		std::cout << "Shaders compiled and linked successfully.\n";
	}
	else
	{
		//If there was a problem, tell OpenGL that we don't need those resources after all.
		glDeleteShader(vertexShader);
		glDeleteShader(fragmentShader);
		glDeleteProgram(shaderProgram);
	}

}

ShaderProgram::~ShaderProgram()
{
	if (loadedSuccessfully)
	{
		glDeleteShader(vertexShader);
		glDeleteShader(fragmentShader);
		glDeleteProgram(shaderProgram);
	}
}

ShaderProgram::ShaderProgram(ShaderProgram&& other) noexcept
{
	this->shaderProgram = other.shaderProgram;
	this->fragmentShader = other.fragmentShader;
	this->vertexShader = other.vertexShader;
	this->loadedSuccessfully = other.loadedSuccessfully;

	other.loadedSuccessfully = false;	//Invalidate the other shader - it's about to have its destructor called, and we need it to not call the glDelete functions.
}

ShaderProgram& ShaderProgram::operator=(ShaderProgram&& other) noexcept
{
	if (&other == this)	//Don't do the move if we're moving this shader to itself.
	{
		return *this;
	}
	if (loadedSuccessfully)
	{
		//If the shader we're moving over the top of is an old valid shader, it should be deleted.
		glDeleteShader(vertexShader);
		glDeleteShader(fragmentShader);
		glDeleteProgram(shaderProgram);
	}
	this->shaderProgram = other.shaderProgram;
	this->fragmentShader = other.fragmentShader;
	this->vertexShader = other.vertexShader;
	this->loadedSuccessfully = other.loadedSuccessfully;

	other.loadedSuccessfully = false;	//Invalidate the other shader - it's about to have its destructor called, and we need it to not call the glDelete functions.
	return *this;
}

void ShaderProgram::UseShader()
{
	glUseProgram(shaderProgram);
}

void ShaderProgram::SetMat4Uniform(std::string varName, float* matrix)
{
	GLuint varLoc = glGetUniformLocation(shaderProgram, varName.c_str());
	glUniformMatrix4fv(varLoc, 1, GL_FALSE, matrix);
}
