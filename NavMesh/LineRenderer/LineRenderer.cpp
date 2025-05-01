#include "LineRenderer.h"
#include <iostream>

void LineRenderer::Initialise()
{
	glGenBuffers(1, &positionBufferID);
	glGenBuffers(1, &colourBufferID);
	initialised = true;
}
LineRenderer::~LineRenderer()
{
	if (initialised)
	{
		glDeleteBuffers(1, &positionBufferID);
		glDeleteBuffers(1, &colourBufferID);
	}
}


void LineRenderer::SetColour(Colour colour)
{
	currentColour = colour;
}

void LineRenderer::DrawLineSegment(Vec2 start, Vec2 end)
{
	DrawLineSegment(start, end, currentColour);
}

void LineRenderer::DrawLineSegment(Vec2 start, Vec2 end, Colour colour)
{
	positions.push_back(start);
	positions.push_back(end);
	colours.push_back(colour);
	colours.push_back(colour);
}

void LineRenderer::DrawLineWithArrow(Vec2 start, Vec2 end, float arrowSize)
{
	DrawLineWithArrow(start, end, currentColour, arrowSize);
}

void LineRenderer::DrawLineWithArrow(Vec2 start, Vec2 end, Colour colour, float arrowSize)
{
	Vec2 lineDirection = (end - start).GetNormalised();
	Vec2 linePerp = lineDirection.GetRotatedBy90() * 0.5f;

	DrawLineSegment(start, end, colour);
	DrawLineSegment(end, end - (lineDirection - linePerp) * arrowSize, colour);
	DrawLineSegment(end, end - (lineDirection + linePerp) * arrowSize, colour);
	
}

void LineRenderer::FinishLineStrip()
{
	if (lineActive)
	{
		lineActive = false;
	}
}

void LineRenderer::FinishLineLoop()
{
	if (lineActive)
	{
		positions.push_back(lastPos);
		positions.push_back(firstPos);
		colours.push_back(lastColour);
		colours.push_back(firstColour);
		lineActive = false;
	}
}

void LineRenderer::AddPointToLine(Vec2 point)
{
	AddPointToLine(point, currentColour);
}

void LineRenderer::AddPointToLine(Vec2 point, Colour colour)
{
	if (lineActive)
	{
		positions.push_back(lastPos);
		positions.push_back(point);
		colours.push_back(lastColour);
		colours.push_back(colour);
		lastPos = point;
		lastColour = colour;
	}
	else
	{
		lineActive = true;
		lastPos = point;
		firstPos = point;
		lastColour = colour;
		firstColour = colour;
	}
}

void LineRenderer::DrawCross(Vec2 centre, float size)
{
	DrawCross(centre, size, currentColour);
}

void LineRenderer::DrawCross(Vec2 centre, float size, Colour colour)
{
	positions.push_back({ centre.x - size, centre.y - size });
	positions.push_back({ centre.x + size, centre.y + size });
	positions.push_back({ centre.x + size, centre.y - size });
	positions.push_back({ centre.x - size, centre.y + size });
	colours.push_back(colour);
	colours.push_back(colour);
	colours.push_back(colour);
	colours.push_back(colour);
}

void LineRenderer::DrawCircle(Vec2 centre, float size)
{
	DrawCircle(centre, size, currentColour);
}

void LineRenderer::DrawCircle(Vec2 centre, float size, Colour colour)
{
	DrawCircle(centre, size, colour, GetCircleSegmentCount(size));
}

void LineRenderer::DrawCircle(Vec2 centre, float size, int segmentCount)
{
	DrawCircle(centre, size, currentColour, segmentCount);
}

void LineRenderer::DrawCircle(Vec2 centre, float size, Colour colour, int segmentCount)
{
	if (centre.x > cameraXMin - size && centre.x < cameraXMax + size && centre.y > cameraYMin - size && centre.y < cameraYMax + size)
	{
		float cosAngle = cos(2 * PI / segmentCount);
		float sinAngle = sin(2 * PI / segmentCount);

		Vec2 plotPoint(0, size);

		for (int i = 0; i < segmentCount - 1; i++)
		{
			positions.push_back(centre + plotPoint);
			plotPoint.RotateBy(cosAngle, sinAngle);
			positions.push_back(centre + plotPoint);
			colours.push_back(colour);
			colours.push_back(colour);
		}
		positions.push_back(centre + plotPoint);
		positions.push_back(centre + Vec2(0, size));
		colours.push_back(colour);
		colours.push_back(colour);
	}
}

void LineRenderer::DrawCircleArc(Vec2 centre, float size, float startBearing, float endBearing, int fullSegmentCount)
{
	DrawCircleArc(centre, size, startBearing, endBearing, currentColour, fullSegmentCount);
}

void LineRenderer::DrawCircleArc(Vec2 centre, float size, float startBearing, float endBearing, Colour colour, int fullSegmentCount)
{
	float angleDiff = endBearing - startBearing;
	if (angleDiff < 0) angleDiff += 2.0f * PI;

	if (angleDiff > 2.0f * PI) angleDiff = 2.0f * PI;

	float circleFraction = angleDiff / (2.0f * PI);
	int segmentCount = (int)ceilf(fullSegmentCount * circleFraction);

	float angleChange = angleDiff / segmentCount;
	
	Vec2 plotPoint(size, 0);
	plotPoint.RotateBy(startBearing);
	float sinAngle = sinf(angleChange);
	float cosAngle = cosf(angleChange);

	for (int i = 0; i < segmentCount; i++)
	{
		positions.push_back(centre + plotPoint);
		plotPoint.RotateBy(cosAngle, sinAngle);
		positions.push_back(centre + plotPoint);
		colours.push_back(colour);
		colours.push_back(colour);
	}

}

void LineRenderer::Clear()
{
	lineActive = false;
	positions.clear();
	colours.clear();
}

void LineRenderer::Compile()
{
	if (positions.size() != 0)
	{
		glBindBuffer(GL_ARRAY_BUFFER, positionBufferID);
		glBufferData(GL_ARRAY_BUFFER, sizeof(Vec2) * positions.size(), positions.data(), GL_DYNAMIC_DRAW);
		glBindBuffer(GL_ARRAY_BUFFER, colourBufferID);
		glBufferData(GL_ARRAY_BUFFER, sizeof(Colour) * colours.size(), colours.data(), GL_DYNAMIC_DRAW);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}
}

void LineRenderer::Draw()
{
	if (positions.size() != 0)
	{
		glBindBuffer(GL_ARRAY_BUFFER, positionBufferID);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(Vec2), 0);

		glBindBuffer(GL_ARRAY_BUFFER, colourBufferID);
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Colour), 0);

		glBindBuffer(GL_ARRAY_BUFFER, 0);

		glDrawArrays(GL_LINES, 0, (GLsizei)positions.size());
	}
}

void LineRenderer::UpdateWithCameraInfo(Vec2 pos, float height, float aspect)
{
	float halfHeight = height / 2;
	float halfWidth = halfHeight * aspect;
	cameraXMin = pos.x - halfWidth;
	cameraXMax = pos.x + halfWidth;
	cameraYMin = pos.y - halfHeight;
	cameraYMax = pos.y + halfHeight;
	
	cameraZoomFactor = 10.0f / height;
}



float LineRenderer::DrawText(std::string text, Vec2 pos, float size)
{
	return DrawText(text, pos, size, currentColour);
}

float LineRenderer::DrawText(std::string text, Vec2 pos, float size, Colour colour)
{
	Vec2 currentPos = pos;
	for (int i = 0; i < text.size(); i++)
	{
		currentPos.x += DrawChar(text[i], currentPos, size, colour);
	}
	return currentPos.x - pos.x;
}

float LineRenderer::DrawChar(char character, Vec2 pos, float size)
{
	return DrawChar(character, pos, size, currentColour);
}

float LineRenderer::DrawChar(char character, Vec2 pos, float size, Colour colour)
{
	std::vector<Vec2> points = GetGlyph(character);
	if (points.size() > 0)
	{
		for (int i = 0; i + 1 < points.size(); i += 2)
		{
			DrawLineSegment(points[i] * size + pos, points[i + 1] * size + pos, colour);
		}
		return size;
	}
	else
	{
		if (character == ' ') return size;
		else return 0.0f;
	}
}

std::vector<Vec2> LineRenderer::GetGlyph(char character)
{
	switch (character)
	{
	case 'a':
	case 'A':
		return { {0.2f, 0.2f}, {0.5f, 1.6f}, {0.5f, 1.6f}, {0.8f, 0.2f} , {0.3f, 0.8f}, {0.7f, 0.8f} };
	case 'b':
	case 'B':
		return { {0.2f, 0.2f}, {0.2f, 1.6f}, {0.2f, 1.6f}, {0.6f, 1.6f}, {0.8f, 1.3f}, {0.6f, 1.6f}, {0.8f, 1.3f}, {0.6f, 1.0f}, {0.6f, 1.0f}, {0.2f, 1.0f}, {0.6f, 1.0f}, {0.9f, 0.6f}, {0.9f, 0.6f}, {0.7f, 0.2f}, {0.7f, 0.2f}, {0.2f, 0.2f} };
	case 'c':
	case 'C':
		return { {0.8f, 1.4f}, {0.6f, 1.6f}, {0.3f, 1.6f}, {0.1f, 1.3f}, {0.1f, 1.3f}, {0.1f, 0.5f}, {0.1f, 0.5f}, {0.3f, 0.2f}, {0.6f, 0.2f}, {0.8f, 0.4f}, {0.3f, 0.2f}, {0.6f, 0.2f}, {0.3f, 1.6f}, {0.6f, 1.6f} };
	case 'd':
	case 'D':
		return { {0.2f, 1.6f}, {0.5f, 1.6f}, {0.8f, 1.3f}, {0.5f, 1.6f}, {0.8f, 1.3f}, {0.8f, 0.5f}, {0.8f, 0.5f}, {0.5f, 0.2f}, {0.5f, 0.2f}, {0.2f, 0.2f}, {0.2f, 0.2f}, {0.2f, 1.6f} };
	case 'e':
	case 'E':
		return { {0.8f, 1.6f}, {0.2f, 1.6f}, {0.2f, 1.6f}, {0.2f, 0.2f}, {0.2f, 0.2f}, {0.8f, 0.2f}, {0.7f, 0.9f}, {0.2f, 0.9f} };
	case 'f':
	case 'F':
		return { {0.8f, 1.6f}, {0.2f, 1.6f}, {0.2f, 1.6f}, {0.2f, 0.2f}, {0.7f, 0.9f}, {0.2f, 0.9f} };
	case 'g':
	case 'G':
		return { {0.8f, 1.3f}, {0.6f, 1.5f}, {0.6f, 1.5f}, {0.3f, 1.5f}, {0.3f, 1.5f}, {0.1f, 1.1f}, {0.1f, 1.1f}, {0.1f, 0.6f}, {0.1f, 0.6f}, {0.2f, 0.3f}, {0.2f, 0.3f}, {0.5f, 0.2f}, {0.8f, 0.5f}, {0.5f, 0.2f}, {0.8f, 0.2f}, {0.8f, 0.5f}, {0.5f, 0.5f}, {0.8f, 0.5f} };
	case 'h':
	case 'H':
		return { {0.2f, 1.5f}, {0.2f, 0.2f}, {0.8f, 1.5f}, {0.8f, 0.2f}, {0.2f, 0.9f}, {0.8f, 0.9f} };
	case 'i':
	case 'I':
		return { {0.5f, 1.5f}, {0.5f, 0.2f}, {0.4f, 1.5f}, {0.6f, 1.5f}, {0.4f, 0.2f}, {0.6f, 0.2f} };
	case 'j':
	case 'J':
		return { {0.7f, 1.5f}, {0.8f, 0.5f}, {0.4f, 1.5f}, {0.8f, 1.5f}, {0.6f, 0.2f}, {0.3f, 0.2f}, {0.2f, 0.5f}, {0.3f, 0.2f}, {0.8f, 0.5f}, {0.6f, 0.2f} };
	case 'k':
	case 'K':
		return { {0.2f, 1.5f}, {0.2f, 0.2f}, {0.8f, 1.4f}, {0.2f, 0.8f}, {0.8f, 0.2f}, {0.3f, 0.9f} };
	case 'l':
	case 'L':
		return { {0.2f, 1.5f}, {0.2f, 0.2f}, {0.8f, 0.2f}, {0.2f, 0.2f} };
	case 'm':
	case 'M':
		return { {0.2f, 1.4f}, {0.2f, 0.2f}, {0.8f, 1.4f}, {0.8f, 0.2f}, {0.5f, 0.8f}, {0.8f, 1.4f}, {0.2f, 1.4f}, {0.5f, 0.8f} };
	case 'n':
	case 'N':
		return { {0.2f, 1.4f}, {0.2f, 0.2f}, {0.8f, 0.2f}, {0.8f, 1.4f}, {0.2f, 1.4f}, {0.8f, 0.2f} };
	case 'o':
	case 'O':
		return { {0.5f, 1.5f}, {0.2f, 1.2f}, {0.8f, 1.2f}, {0.5f, 1.5f}, {0.5f, 0.1f}, {0.8f, 0.4f}, {0.2f, 0.4f}, {0.5f, 0.1f}, {0.2f, 0.4f}, {0.2f, 1.2f}, {0.8f, 0.4f}, {0.8f, 1.2f} };
	case 'p':
	case 'P':
		return { {0.2f, 1.6f}, {0.6f, 1.6f}, {0.8f, 1.3f}, {0.6f, 1.6f}, {0.6f, 0.9f}, {0.8f, 1.3f}, {0.2f, 0.9f}, {0.6f, 0.9f}, {0.2f, 1.6f}, {0.2f, 0.2f} };
	case 'q':
	case 'Q':
		return { {0.5f, 1.6f}, {0.1f, 1.2f}, {0.1f, 0.5f}, {0.4f, 0.2f}, {0.9f, 0.5f}, {0.9f, 1.2f}, {0.5f, 1.6f}, {0.9f, 1.2f}, {0.1f, 1.2f}, {0.1f, 0.5f}, {0.4f, 0.2f}, {0.9f, 0.5f}, {0.5f, 0.5f}, {0.8f, 0.2f} };
	case 'r':
	case 'R':
		return { {0.2f, 1.6f}, {0.7f, 1.6f}, {0.9f, 1.2f}, {0.7f, 0.9f}, {0.2f, 0.9f}, {0.7f, 0.9f}, {0.7f, 1.6f}, {0.9f, 1.2f}, {0.4f, 0.9f}, {0.8f, 0.2f}, {0.2f, 0.2f}, {0.2f, 1.6f} };
	case 's':
	case 'S':
		return { {0.9f, 1.3f}, {0.7f, 1.6f}, {0.3f, 1.6f}, {0.1f, 1.3f}, {0.2f, 0.9f}, {0.8f, 0.7f}, {0.9f, 0.3f}, {0.5f, 0.1f}, {0.1f, 0.3f}, {0.5f, 0.1f}, {0.8f, 0.7f}, {0.9f, 0.3f}, {0.7f, 1.6f}, {0.3f, 1.6f}, {0.1f, 1.3f}, {0.2f, 0.9f} };
	case 't':
	case 'T':
		return { {0.1f, 1.5f}, {0.9f, 1.5f}, {0.5f, 1.5f}, {0.5f, 0.2f} };
	case 'u':
	case 'U':
		return { {0.8f, 1.5f}, {0.8f, 0.6f}, {0.4f, 0.2f}, {0.2f, 0.6f}, {0.2f, 1.5f}, {0.2f, 0.6f}, {0.8f, 0.6f}, {0.6f, 0.2f}, {0.4f, 0.2f}, {0.6f, 0.2f} };
	case 'v':
	case 'V':
		return { {0.9f, 1.5f}, {0.5f, 0.2f}, {0.5f, 0.2f}, {0.1f, 1.5f} };
	case 'w':
	case 'W':
		return { {0.9f, 1.5f}, {0.8f, 0.2f}, {0.2f, 0.2f}, {0.1f, 1.5f}, {0.5f, 0.8f}, {0.8f, 0.2f}, {0.5f, 0.8f}, {0.2f, 0.2f} };
	case 'x':
	case 'X':
		return { {0.8f, 0.2f}, {0.2f, 1.5f}, {0.2f, 0.2f}, {0.8f, 1.5f} };
	case 'y':
	case 'Y':
		return { {0.2f, 1.5f}, {0.5f, 0.9f}, {0.8f, 1.5f}, {0.5f, 0.9f}, {0.5f, 0.2f}, {0.5f, 0.9f} };
	case 'z':
	case 'Z':
		return { {0.2f, 1.5f}, {0.8f, 1.5f}, {0.8f, 0.2f}, {0.2f, 0.2f}, {0.8f, 1.5f}, {0.2f, 0.2f} };
	case '1':
		return { {0.3f, 1.3f}, {0.5f, 1.4f}, {0.5f, 1.4f}, {0.5f, 0.2f}, {0.3f, 0.2f}, {0.7f, 0.2f} };
	case '2':
		return { {0.2f, 1.3f}, {0.4f, 1.5f}, {0.6f, 0.8f}, {0.7f, 1.3f}, {0.2f, 0.3f}, {0.6f, 0.8f}, {0.8f, 0.3f}, {0.2f, 0.3f}, {0.4f, 1.5f}, {0.7f, 1.3f} };
	case '3':
		return { {0.2f, 1.3f}, {0.5f, 1.5f}, {0.4f, 0.9f}, {0.8f, 1.2f}, {0.5f, 1.5f}, {0.8f, 1.2f}, {0.7f, 0.7f}, {0.4f, 0.9f}, {0.8f, 0.4f}, {0.7f, 0.7f}, {0.2f, 0.3f}, {0.4f, 0.2f}, {0.8f, 0.4f}, {0.4f, 0.2f} };
	case '4':
		return { {0.6f, 1.5f}, {0.2f, 0.8f}, {0.8f, 0.8f}, {0.2f, 0.8f}, {0.6f, 0.9f}, {0.5f, 0.2f} };
	case '5':
		return { {0.8f, 1.4f}, {0.2f, 1.4f}, {0.2f, 0.9f}, {0.6f, 0.8f}, {0.8f, 0.5f}, {0.6f, 0.2f}, {0.6f, 0.8f}, {0.8f, 0.5f}, {0.2f, 1.4f}, {0.2f, 0.9f}, {0.1f, 0.3f}, {0.6f, 0.2f} };
	case '6':
		return { {0.7f, 1.5f}, {0.3f, 1.2f}, {0.2f, 0.8f}, {0.2f, 0.4f}, {0.5f, 0.2f}, {0.8f, 0.4f}, {0.9f, 0.7f}, {0.6f, 0.9f}, {0.8f, 0.4f}, {0.9f, 0.7f}, {0.2f, 0.4f}, {0.5f, 0.2f}, {0.3f, 1.2f}, {0.2f, 0.8f}, {0.2f, 0.8f}, {0.6f, 0.9f} };
	case '7':
		return { {0.2f, 1.4f}, {0.8f, 1.4f}, {0.5f, 0.8f}, {0.4f, 0.2f}, {0.5f, 0.8f}, {0.8f, 1.4f} };
	case '8':
		return { {0.5f, 1.5f}, {0.3f, 1.3f}, {0.3f, 1.f}, {0.8f, 0.6f}, {0.8f, 0.3f}, {0.5f, 0.1f}, {0.2f, 0.3f}, {0.5f, 0.1f}, {0.2f, 0.6f}, {0.2f, 0.3f}, {0.8f, 0.6f}, {0.8f, 0.3f}, {0.7f, 1.3f}, {0.7f, 1.f}, {0.2f, 0.6f}, {0.7f, 1.f}, {0.5f, 1.5f}, {0.7f, 1.3f}, {0.3f, 1.3f}, {0.3f, 1.f} };
	case '9':
		return { {0.2f, 1.3f}, {0.4f, 1.5f}, {0.7f, 1.5f}, {0.8f, 1.1f}, {0.7f, 0.6f}, {0.2f, 0.2f}, {0.2f, 1.f}, {0.5f, 0.9f}, {0.8f, 1.1f}, {0.5f, 0.9f}, {0.8f, 1.1f}, {0.7f, 0.6f}, {0.2f, 1.3f}, {0.2f, 1.f}, {0.4f, 1.5f}, {0.7f, 1.5f} };
	case '0':
		return { {0.5f, 1.5f}, {0.2f, 1.1f}, {0.2f, 0.5f}, {0.5f, 0.2f}, {0.8f, 0.5f}, {0.8f, 1.1f}, {0.5f, 1.5f}, {0.8f, 1.1f}, {0.2f, 1.1f}, {0.2f, 0.5f}, {0.8f, 0.5f}, {0.5f, 0.2f}, {0.2f, 0.2f}, {0.8f, 1.5f} };
	case '!':
		return { {0.5f, 1.5f}, {0.5f, 0.6f}, {0.4f, 0.3f}, {0.6f, 0.1f}, {0.6f, 0.3f}, {0.4f, 0.1f} };
	case '"':
		return { {0.4f, 1.4f}, {0.4f, 1.2f}, {0.6f, 1.4f}, {0.6f, 1.2f} };
	case '#':
		return { {0.3f, 1.3f}, {0.3f, 0.6f}, {0.7f, 1.3f}, {0.7f, 0.6f}, {0.1f, 1.1f}, {0.9f, 1.1f}, {0.1f, 0.8f}, {0.9f, 0.8f} };
	case '$':
		return { {0.5f, 1.5f}, {0.8f, 1.3f}, {0.2f, 1.2f}, {0.2f, 0.9f}, {0.8f, 0.7f}, {0.8f, 0.4f}, {0.5f, 0.2f}, {0.2f, 0.4f}, {0.5f, 0.2f}, {0.8f, 0.4f}, {0.2f, 0.9f}, {0.8f, 0.7f}, {0.5f, 1.5f}, {0.2f, 1.2f}, {0.4f, 1.5f}, {0.4f, 0.2f}, {0.6f, 1.5f}, {0.6f, 0.2f} };
	case '%':
		return { {0.8f, 1.4f}, {0.2f, 0.3f}, {0.3f, 1.1f}, {0.2f, 1.0f}, {0.3f, 0.9f}, {0.4f, 1.0f}, {0.2f, 1.0f}, {0.3f, 0.9f}, {0.3f, 1.1f}, {0.4f, 1.0f}, {0.6f, 0.7f}, {0.7f, 0.8f}, {0.7f, 0.6f}, {0.8f, 0.7f}, {0.6f, 0.7f}, {0.7f, 0.6f}, {0.7f, 0.8f}, {0.8f, 0.7f} };
	case '&':
		return { {0.5f, 1.5f}, {0.2f, 1.3f}, {0.2f, 1.f}, {0.6f, 0.7f}, {0.9f, 0.5f}, {0.4f, 0.2f}, {0.2f, 0.4f}, {0.4f, 0.2f}, {0.2f, 0.6f}, {0.2f, 0.4f}, {0.6f, 0.7f}, {0.8f, 0.2f}, {0.2f, 0.6f}, {0.7f, 1.f}, {0.5f, 1.5f}, {0.7f, 1.3f}, {0.2f, 1.3f}, {0.2f, 1.f} };
	case '\'':
		return { {0.5f, 1.4f}, {0.5f, 1.2f} };
	case '(':
		return { {0.5f, 1.5f}, {0.2f, 1.1f}, {0.2f, 0.6f}, {0.5f, 0.2f}, {0.2f, 1.1f}, {0.2f, 0.6f} };
	case ')':
		return { {0.5f, 1.5f}, {0.8f, 1.1f}, {0.8f, 0.6f}, {0.5f, 0.2f}, {0.8f, 1.1f}, {0.8f, 0.6f} };
	case '*':
		return { {0.4f, 1.4f}, {0.6f, 1.0f}, {0.6f, 1.4f}, {0.4f, 1.0f}, {0.3f, 1.2f}, {0.7f, 1.2f} };
	case '+':
		return { {0.5f, 1.1f}, {0.5f, 0.5f}, {0.2f, 0.8f}, {0.8f, 0.8f} };
	case ',':
		return { {0.6f, 0.4f}, {0.4f, 0.2f} };
	case '-':
		return { {0.2f, 0.9f}, {0.8f, 0.9f} };
	case '.':
		return { {0.4f, 0.3f}, {0.6f, 0.3f}, {0.5f, 0.4f}, {0.5f, 0.2f} };
	case '/':
		return { {0.8f, 1.4f}, {0.2f, 0.2f} };
	case ':':
		return { {0.6f, 1.3f}, {0.5f, 1.2f}, {0.4f, 1.3f}, {0.5f, 1.4f}, {0.5f, 1.4f}, {0.6f, 1.3f}, {0.4f, 1.3f}, {0.5f, 1.2f}, {0.5f, 0.5f}, {0.4f, 0.4f}, {0.5f, 0.3f}, {0.6f, 0.4f}, {0.5f, 0.5f}, {0.6f, 0.4f}, {0.4f, 0.4f}, {0.5f, 0.3f} };
	case ';':
		return { {0.6f, 1.3f}, {0.5f, 1.2f}, {0.4f, 1.3f}, {0.5f, 1.4f}, {0.5f, 1.4f}, {0.6f, 1.3f}, {0.4f, 1.3f}, {0.5f, 1.2f}, {0.6f, 0.5f}, {0.4f, 0.2f} };
	case '<':
		return { {0.8f, 1.2f}, {0.1f, 0.9f}, {0.8f, 0.6f}, {0.1f, 0.9f} };
	case '=':
		return { {0.8f, 1.2f}, {0.2f, 1.2f}, {0.8f, 0.6f}, {0.2f, 0.6f} };
	case '>':
		return { {0.9f, 0.9f}, {0.2f, 1.2f}, {0.9f, 0.9f}, {0.2f, 0.6f} };
	case '?':
		return { {0.1f, 1.2f}, {0.3f, 1.5f}, {0.7f, 1.5f}, {0.9f, 1.3f}, {0.3f, 1.5f}, {0.7f, 1.5f}, {0.8f, 1.0f}, {0.9f, 1.3f}, {0.5f, 0.8f}, {0.8f, 1.0f}, {0.5f, 0.5f}, {0.5f, 0.8f}, {0.5f, 0.3f}, {0.4f, 0.2f}, {0.6f, 0.2f}, {0.5f, 0.1f}, {0.6f, 0.2f}, {0.5f, 0.3f}, {0.4f, 0.2f}, {0.5f, 0.1f} };
	case '@':
		return { {0.8f, 1.0f}, {0.5f, 1.2f}, {0.3f, 1.0f}, {0.3f, 0.6f}, {0.5f, 0.5f}, {0.8f, 0.7f}, {0.7f, 1.5f}, {0.8f, 0.7f}, {0.5f, 1.2f}, {0.3f, 1.0f}, {0.5f, 0.5f}, {0.3f, 0.6f}, {0.4f, 1.5f}, {0.7f, 1.5f}, {0.1f, 1.1f}, {0.4f, 1.5f}, {0.1f, 0.5f}, {0.1f, 1.1f}, {0.4f, 0.2f}, {0.8f, 0.3f}, {0.1f, 0.5f}, {0.4f, 0.2f} };
	case '[':
		return { {0.5f, 1.5f}, {0.2f, 1.5f}, {0.2f, 0.2f}, {0.5f, 0.2f}, {0.2f, 0.2f}, {0.2f, 1.5f} };
	case ']':
		return { {0.5f, 1.5f}, {0.8f, 1.5f}, {0.8f, 0.2f}, {0.5f, 0.2f}, {0.8f, 0.2f}, {0.8f, 1.5f} };
	case '\\':
		return { {0.2f, 1.5f}, {0.8f, 0.2f} };
	case '^':
		return { {0.5f, 1.5f}, {0.3f, 1.2f}, {0.7f, 1.2f}, {0.5f, 1.5f} };
	case '_':
		return { {0.2f, 0.2f}, {0.8f, 0.2f} };
	case '`':
		return { {0.4f, 1.4f}, {0.6f, 1.2f} };
	case '{':
		return { {0.5f, 0.2f}, {0.3f, 0.3f}, {0.2f, 0.9f}, {0.3f, 0.7f}, {0.3f, 0.7f}, {0.3f, 0.3f}, {0.3f, 1.0f}, {0.3f, 1.3f}, {0.3f, 1.3f}, {0.5f, 1.4f}, {0.3f, 1.0f}, {0.2f, 0.8f} };
	case '}':
		return { {0.5f, 0.2f}, {0.7f, 0.3f}, {0.8f, 0.9f}, {0.7f, 0.7f}, {0.7f, 0.7f}, {0.7f, 0.3f}, {0.7f, 1.0f}, {0.7f, 1.3f}, {0.7f, 1.3f}, {0.5f, 1.4f}, {0.7f, 1.0f}, {0.8f, 0.8f} };
	case '|':
		return { {0.5f, 0.2f}, {0.5f, 1.4f} };
	case '~':
		return { {0.1f, 0.9f}, {0.3f, 1.1f}, {0.7f, 0.9f}, {0.9f, 1.1f}, {0.3f, 1.1f}, {0.7f, 0.9f} };
	}
	return {};
}

int LineRenderer::GetCircleSegmentCount(float radius) const
{
	return Clamp((int)(sqrtf(radius * cameraZoomFactor) * 32 + 4), 5, 128);
}
