#include "Vehicle.h"

Vehicle::Vehicle(NodeGraph* nodeGraph, float width, Colour colour, World* world) : PathAgent(nodeGraph, width, colour, world)
{
}

void Vehicle::ApplyForce()
{
}

void Vehicle::Seek()
{
}

void Vehicle::Update(float deltaTime)
{
	mPosition += mVelocity * deltaTime;
}
