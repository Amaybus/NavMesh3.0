#pragma once

#include "Vec2.h"
#include "LineRenderer.h"
#include <vector>

class Obstacle
{
	std::vector<Vec2> mPoints;

public:
	Obstacle(std::vector<Vec2> points);
	~Obstacle();
	Obstacle(const Obstacle& other) = default;
	Obstacle& operator=(const Obstacle& other) = default;

	void Draw(LineRenderer* lines);

	std::vector<Vec2>& GetPoints() { return mPoints; }
	void AddSinglePoint(const Vec2 point) { mPoints.push_back(point); }
	void AddListPoints(const std::vector<Vec2> pointList);
};

