#pragma once

#include <math.h>
#include "Vec2.h"
const float PI = 3.14159265f;

float Remap(float val, float oldMin, float oldMax, float newMin, float newMax);


//These could be done with eg Remap(degrees, 0, 360, 0, 2 * PI).
constexpr float DegToRad(float degrees) { return degrees / 360.0f * (2 * PI); }
constexpr float RadToDeg(float radians) { return radians / (2 * PI) * 360.0f; }

template <typename T>
T Clamp(T val, T min, T max)
{
	if (val < min) return min;
	if (val > max) return max;
	return val;
}


template <typename T>
T Min(T a, T b)
{
	return a < b ? a : b;
}
template <typename T>
T Max(T a, T b)
{
	return a > b ? a : b;
}

template <typename T>
T Lerp(T a, T b, float t)
{
	return a * (1.0f - t) + b * t;
}