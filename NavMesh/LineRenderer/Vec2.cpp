#include "Vec2.h"
#include <math.h>


float Vec2::GetMagnitude() const
{
	return sqrtf(x * x + y * y);
}

float Vec2::GetMagnitudeSquared() const
{
	return x * x + y * y;
}

Vec2& Vec2::Normalise()
{
	float mag = GetMagnitude();
	if (mag > 0.0f) *this /= mag;
	return *this;
}

Vec2 Vec2::GetNormalised() const
{
	float mag = GetMagnitude();
	if (mag > 0.0f) return *this / mag;
	else return *this;
}

Vec2& Vec2::RotateBy90()
{
	float swap = x;
	x = -y;
	y = swap;
	return *this;
}

Vec2& Vec2::RotateBy270()
{
	float swap = x;
	x = y;
	y = -swap;
	return *this;
}

Vec2& Vec2::RotateBy(float angle)
{
	//Optimisation note: there's a native assembly function for calculating
	//sine and cosine of the same angle simultaneously that is very quick,
	//but generally an optimising compiler will be able to substitute that
	//for you - there's no need to call a special 'sincos' function, although
	//you might find reference to it in places because it used to exist.
	float cosAngle = cosf(angle);
	float sinAngle = sinf(angle);
	float oldX = x;
	x = x * cosAngle - y * sinAngle;
	y = oldX * sinAngle + y * cosAngle;
	return *this;
}

Vec2& Vec2::RotateBy(float cosAngle, float sinAngle)
{
	//This is for if you have to rotate something by the same angle a lot,
	//so you want to avoid recalculating the trig values.
	float oldX = x;
	x = x * cosAngle - y * sinAngle;
	y = oldX * sinAngle + y * cosAngle;
	return *this;
}

Vec2 Vec2::GetRotatedBy90() const
{
	return Vec2(-y, x);
}

Vec2 Vec2::GetRotatedBy270() const
{
	return Vec2(y, -x);
}

Vec2 Vec2::GetRotatedBy(float angle) const
{
	float cosAngle = cosf(angle);
	float sinAngle = sinf(angle);
	return Vec2(
		x * cosAngle - y * sinAngle,
		x * sinAngle + y * cosAngle
	);
}

Vec2 Vec2::GetRotatedBy(float cosAngle, float sinAngle) const
{
	return Vec2(
		x * cosAngle - y * sinAngle,
		x * sinAngle + y * cosAngle
	);
}

void Vec2::SetMagnitude(float mag)
{
	Normalise();
	*this *= mag;
}

Vec2& Vec2::operator+=(Vec2 v)
{
	x += v.x;
	y += v.y;
	return *this;
}

Vec2& Vec2::operator-=(Vec2 v)
{
	x -= v.x;
	y -= v.y;
	return *this;
}

Vec2& Vec2::operator*=(float s)
{
	x *= s;
	y *= s;
	return *this;
}

Vec2& Vec2::operator/=(float s)
{
	x /= s;
	y /= s;
	return *this;
}

Vec2 operator+(Vec2 a, Vec2 b)
{
	return Vec2(a.x + b.x, a.y + b.y);
}

Vec2 operator-(Vec2 a, Vec2 b)
{
	return Vec2(a.x - b.x, a.y - b.y);
}

Vec2 operator*(Vec2 v, float s)
{
	return Vec2(v.x * s, v.y * s);
}

Vec2 operator*(float s, Vec2 v)
{
	return Vec2(v.x * s, v.y * s);
}

Vec2 operator/(Vec2 v, float s)
{
	return Vec2(v.x / s, v.y / s);
}

//Unary negative operator, so you can write a vector as "-vel" or whatever.
Vec2 operator-(Vec2 v)
{
	return Vec2(-v.x, -v.y);
}

float Dot(Vec2 a, Vec2 b)
{
	return a.x * b.x + a.y * b.y;
}

float PseudoCross(Vec2 a, Vec2 b)
{
	//This isn't a 'true' cross product, because they don't exist in
	//2D space. This interpretation essentially treats the two
	//input vectors as 3D vectors on the XY plane (so 'z' is zero),
	//does a 3D cross product, and then returns the z value of that
	//result (since the x and y of the result will always be zero).
	//It can be useful for stuff like checking if something is a
	//left or right turn.
	return a.x * b.y - a.y * b.x;
}

float AngleBetween(Vec2 a, Vec2 b)
{
	return acosf(Dot(a, b));
}