#pragma once
class LineRenderer;
#include "Vec2.h"
#include "Colour.h"
#include <string>
#include <vector>

class TextStream
{
private:
	LineRenderer* lines;
	Vec2 cursorPos;
	float leftPos;
	float size;
	Colour currentColour;

	char floatFormat[5] = "%.2f";	//This relates to the old C-style string formatting system, which I use for converting floats.
public:
	TextStream(LineRenderer* linesPointer, Vec2 startPos, float sizeInit, Colour initialColour = Colour::WHITE);

	void SetDecimalPlaces(int places);

	TextStream& operator<<(float num);
	TextStream& operator<<(const char* text);
	TextStream& operator<<(const std::string& text);
	TextStream& operator<<(int num);
	TextStream& operator<<(Vec2 vec);
	TextStream& operator<<(char character);
	TextStream& operator<<(Colour newColour);

	//TODO set this up to allow any container class of elements so long as it has a standard iterator.
	template <typename T>
	TextStream& operator<<(std::vector<T> elements)
	{
		*this << '[';
		for (int i = 0; i + 1 < elements.size(); i++)
		{
			*this << elements[i] << ", ";
		}
		if (elements.size() > 0) *this << elements.back();
		*this << ']';
		return *this;
	}
};