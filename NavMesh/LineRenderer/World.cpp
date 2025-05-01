#include "World.h"

World::World()
{
}

World::~World()
{
	delete mNodeGraph;
	delete mNavMesh;

	for (Obstacle* ob : mObstacles)
	{
		delete ob;
	}

	for (PathAgent* agent : mPathAgents)
	{
		delete agent;
	}
}

void World::Initialise()
{
}

void World::Update(float delta)
{
}

void World::Draw(LineRenderer* lines)
{
}

std::vector<Vec2> World::LineTrace(Vec2 startPos, Grid& grid)
{
	return std::vector<Vec2>();
}
