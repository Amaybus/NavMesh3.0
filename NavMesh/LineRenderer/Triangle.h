#pragma once

#include "Vec2.h"
#include "Edge.h"


struct Triangle
{
	Vec2 mPoints[3];
	std::vector<Triangle*> mAdjTris;
	TriEdge mEdgeList[3];
};