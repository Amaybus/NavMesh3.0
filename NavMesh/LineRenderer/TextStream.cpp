#include "TextStream.h"
#include <string>
#include "LineRenderer.h"

TextStream::TextStream(LineRenderer* linesPointer, Vec2 startPos, float sizeInit, Colour initialColour)
    : lines(linesPointer), cursorPos(startPos), size(sizeInit), currentColour(initialColour)
{
    leftPos = cursorPos.x;
}

void TextStream::SetDecimalPlaces(int places)
{
    //Mildly weird string manipulation - this is the location of the precision character
    //in the string to be passed to snprintf.
    floatFormat[2] = Clamp(places, 1, 9) + '0';
}

TextStream& TextStream::operator<<(float num)
{
    char toPrint[30];
    snprintf(toPrint, 30, floatFormat, num);
    return *this << toPrint;
}

TextStream& TextStream::operator<<(const char* text)
{
    for (int i = 0; text[i]; i++) *this << text[i];
    return *this;
}

TextStream& TextStream::operator<<(const std::string& text)
{
    for (int i = 0; i < text.size(); i++) *this << text[i];
    return *this;
}

TextStream& TextStream::operator<<(int num)
{
    cursorPos.x += lines->DrawText(std::to_string(num), cursorPos, size, currentColour);
    return *this;
}

TextStream& TextStream::operator<<(Vec2 vec)
{
    return *this << '(' << vec.x << ", " << vec.y << ')';
}

TextStream& TextStream::operator<<(char character)
{
    if (character == '\n')
    {
        cursorPos.y -= size * 2.0f;
        cursorPos.x = leftPos;
    }
    else
    {
        cursorPos.x += lines->DrawChar(character, cursorPos, size, currentColour);
    }
    return *this;
}

TextStream& TextStream::operator<<(Colour newColour)
{
    currentColour = newColour;
    return *this;
}
