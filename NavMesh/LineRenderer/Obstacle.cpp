#include "Obstacle.h"
#include "Utility.h"

Obstacle::Obstacle(std::vector<Vec2> points) : mPoints(points)
{
}

Obstacle::~Obstacle()
{
}

void Obstacle::Draw(LineRenderer* lines)
{
	for (Vec2& v : mPoints)
	{
		lines->AddPointToLine(v, Colour::GREEN);
	}

	lines->FinishLineLoop();
}

void Obstacle::AddListPoints(std::vector<Vec2> pointList)
{
	for (Vec2 p : pointList)
	{
		mPoints.push_back(p);
	}
}
