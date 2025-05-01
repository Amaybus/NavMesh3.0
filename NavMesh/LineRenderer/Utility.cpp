#include "Utility.h"

#include <cmath>

bool Vector2IsEqual(Vec2 a, Vec2 b)
{
	float tolerance = 0.0001f;
	if (abs(a.x - b.x) >= tolerance || abs(a.y - b.y) >= tolerance) { return false; }

	return true;
}
