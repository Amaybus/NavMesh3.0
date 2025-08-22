#pragma once
#include "PathAgent.h"

class Vehicle : public PathAgent
{
	Vec2 mVelocity;
	Vec2 mAcceleration;
	float mMaxSpeed;
	float mMaxForce;

public:
	Vehicle(NodeGraph* nodeGraph, float width, Colour colour, World* world);

	void ApplyForce();
	void Seek();
	void Update(float deltaTime) override;
};

