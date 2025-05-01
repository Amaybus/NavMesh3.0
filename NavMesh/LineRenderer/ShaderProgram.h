#pragma once

#include "Graphics.h"
#include <string>
#include "Maths.h"

class ShaderProgram
{
private:
	//Remember, OpenGL manages its resources itself and gives you handles to them.
	//That's what all this 'GLuint' business is about.
	GLuint vertexShader = 0;
	GLuint fragmentShader = 0;
	GLuint shaderProgram = 0;

	bool loadedSuccessfully = false;

public:
	ShaderProgram() {}
	ShaderProgram(std::string vertexFilename, std::string fragmentFilename);

	~ShaderProgram();

	//We don't want to be duplicating our shader around - that will screw us over.
	ShaderProgram(const ShaderProgram& other) = delete;
	ShaderProgram& operator= (const ShaderProgram& other) = delete;

	//Moving it around is fine though, we just have to do it properly.
	ShaderProgram(ShaderProgram&& other) noexcept;
	ShaderProgram& operator= (ShaderProgram&& other) noexcept;

	bool IsLoadedSuccessfully() const { return loadedSuccessfully; }

	void UseShader();

	void SetMat4Uniform(std::string varName, float* matrix);
};
