#pragma once

#include "Vec2.h"
#include <vector>

struct Triangle;

struct TriEdge
{
	Vec2 mPoints[2];
	std::vector<Triangle*> mTriangles;
};

