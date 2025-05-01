#pragma once

#include "Vec2.h"

struct Edge;

struct Triangle
{
	Vec2 mPoints[3];
	std::vector<Triangle*> mAdjTris[3];
	Edge* mEdgeList[3];
};