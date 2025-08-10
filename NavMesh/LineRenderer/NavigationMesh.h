#pragma once

#include "Vec2.h"
#include <vector>
#include "LineRenderer.h"

class Grid;
class Obstacle;
struct Triangle;

class NavigationMesh
{
	std::vector<Vec2> mPoints;
	std::vector<Triangle*> mTriangles;

public:
	NavigationMesh();

	~NavigationMesh();
	NavigationMesh(const NavigationMesh& other) = default;
	NavigationMesh& operator=(const NavigationMesh& other) = default;

	void Draw(LineRenderer* lines);
	void Build(std::vector<Obstacle*>& obstacles);

	std::vector<Vec2>& GetPoints() { return mPoints; }
	std::vector<Triangle*>& GetTriangles() { return mTriangles; }

	void AddPoint(Vec2 point) { mPoints.push_back(point); }
	void AddPointList(std::vector<Vec2> pointList);
	int GetNumberOfTriangles() { return (int)mTriangles.size(); }
	Triangle* GetTriangleAtIndex(int index) { return mTriangles[index]; }
};

