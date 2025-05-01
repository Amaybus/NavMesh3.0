#pragma once

#include "Maths.h"
#include <vector>
#include "Graphics.h"
#include "Colour.h"
#include <string>

class LineRenderer
{
private:
	std::vector<Vec2> positions;
	std::vector<Colour> colours;
	bool initialised = false;

	Colour currentColour = { 1.0f,1.0f,1.0f };

	Vec2 firstPos;
	Vec2 lastPos;
	Colour firstColour = { 0.0f,0.0f,0.0f };
	Colour lastColour = { 0.0f,0.0f,0.0f };
	bool lineActive = false;

	GLuint positionBufferID = 0;
	GLuint colourBufferID = 0;

	float cameraXMin = -FLT_MAX;
	float cameraXMax = FLT_MAX;
	float cameraYMin = -FLT_MAX;
	float cameraYMax = FLT_MAX;
	float cameraZoomFactor = 1.0f;

public:
	LineRenderer() = default;
	~LineRenderer();
	LineRenderer(const LineRenderer&) = delete;
	const LineRenderer& operator=(const LineRenderer&) = delete;

	void Initialise();

	void SetColour(Colour colour);

	void DrawLineSegment(Vec2 start, Vec2 end);
	void DrawLineSegment(Vec2 start, Vec2 end, Colour colour);

	void DrawLineWithArrow(Vec2 start, Vec2 end, float arrowSize = 0.3f);
	void DrawLineWithArrow(Vec2 start, Vec2 end, Colour colour, float arrowSize = 0.3f);

	void FinishLineStrip();
	void FinishLineLoop();

	void AddPointToLine(Vec2 point);
	void AddPointToLine(Vec2 point, Colour colour);

	void DrawCross(Vec2 centre, float size);
	void DrawCross(Vec2 centre, float size, Colour colour);

	void DrawCircle(Vec2 centre, float size);
	void DrawCircle(Vec2 centre, float size, Colour colour);

	void DrawCircleArc(Vec2 centre, float size, float startBearing, float endBearing, int fullSegmentCount = 64);
	void DrawCircleArc(Vec2 centre, float size, float startBearing, float endBearing, Colour colour, int fullSegmentCount = 64);

	float DrawText(std::string text, Vec2 pos, float size);
	float DrawText(std::string text, Vec2 pos, float size, Colour colour);
	float DrawChar(char character, Vec2 pos, float size);
	float DrawChar(char character, Vec2 pos, float size, Colour colour);

	void Clear();
	void Compile();
	void Draw();

	void UpdateWithCameraInfo(Vec2 pos, float height, float aspect);


private:
	static std::vector<Vec2> GetGlyph(char character);

	int GetCircleSegmentCount(float radius) const;

	void DrawCircle(Vec2 centre, float size, int segmentCount);
	void DrawCircle(Vec2 centre, float size, Colour colour, int segmentCount);
};