#pragma once

#include "Vec2.h"
#include "TriEdge.h"


struct Triangle
{
	Vec2 mPoints[3];
	TriEdge mEdgeList[3];
};