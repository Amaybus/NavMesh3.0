#pragma once

#include "Vec2.h"
#include <vector>

struct Triangle;

struct Edge
{
	Vec2 mVerts[2];
	std::vector<Triangle*> mTriangles;
};

