#pragma once

class Vec2
{
public:
	float x = 0.0f;
	float y = 0.0f;

	Vec2() = default;
	Vec2(float xInit, float yInit) :x(xInit), y(yInit) {}
	float GetMagnitude() const;
	float GetMagnitudeSquared() const;
	Vec2& Normalise();
	[[nodiscard]] Vec2 GetNormalised() const;

	Vec2& RotateBy90();
	Vec2& RotateBy270();
	Vec2& RotateBy(float angle);
	Vec2& RotateBy(float cosAngle, float sinAngle);

	[[nodiscard]] Vec2 GetRotatedBy90() const;
	[[nodiscard]] Vec2 GetRotatedBy270() const;
	[[nodiscard]] Vec2 GetRotatedBy(float angle) const;
	[[nodiscard]] Vec2 GetRotatedBy(float cosAngle, float sinAngle) const;

	void SetMagnitude(float mag);

	Vec2& operator+=(Vec2 v);
	Vec2& operator-=(Vec2 v);
	Vec2& operator*=(float s);
	Vec2& operator/=(float s);
};

Vec2 operator+(Vec2 a, Vec2 b);
Vec2 operator-(Vec2 a, Vec2 b);
Vec2 operator*(Vec2 v, float s);
Vec2 operator*(float s, Vec2 v);
Vec2 operator/(Vec2 v, float s);
Vec2 operator-(Vec2 v);

float Dot(Vec2 a, Vec2 b);
float PseudoCross(Vec2 a, Vec2 b);
float AngleBetween(Vec2 a, Vec2 b);
