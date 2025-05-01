#pragma once

#include "Vec2.h"
#include <vector>
#include "LineRenderer.h"

struct Triangle;

class NavigationMesh
{
	std::vector<Vec2> mPoints;
	std::vector<Triangle> mTriangles;
	std::vector<Vec2*> mBorder;

public:
	NavigationMesh();

	virtual ~NavigationMesh() = default;
	NavigationMesh(const NavigationMesh& other) = default;
	NavigationMesh& operator=(const NavigationMesh& other) = default;

	void Draw(LineRenderer* lines);
	void Build();

	std::vector<Vec2>& GetPoints() { return mPoints; }
	std::vector<Triangle>& GetTriangles() { return mTriangles; }
	std::vector<Vec2*>& GetBorder() { return mBorder; }

	void AddPoint(Vec2 point) { mPoints.push_back(point); }
	void AddPointList(std::vector<Vec2> pointList);
};

