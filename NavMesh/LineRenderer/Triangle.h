#pragma once

#include "Vec2.h"
#include "TriEdge.h"


struct Triangle
{
	Vec2 mPoints[3];
	std::vector<Triangle*> mAdjTris;
	std::vector<int> adjTriIndices;
	TriEdge mEdgeList[3];
};