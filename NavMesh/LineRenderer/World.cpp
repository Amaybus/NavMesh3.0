#include "World.h"
#include "Utility.h"
#include "Grid.h"

World::World()
{
	appInfo.grid.show = false;
	cameraHeight = 1000.0f;
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
	// Create level from file
	Grid level(100);
	level.LoadFromImage("Test.png");

	// Create nav mesh with dimensions of the file
	NavigationMesh* navMesh = new NavigationMesh();
	mNavMesh = navMesh;

	// Init the level
	bool isInnerRegionDefined = false;
	for (int y = 0; y < level.GetHeight(); y++)
	{
		for (int x = 0; x < level.GetWidth(); x++)
		{
			// Ignore outside tiles and empty tiles if the inside has been traced already
			if (level.At(x, y) == TileType::OUTSIDE || (level.At(x, y) == TileType::EMPTY && isInnerRegionDefined))
			{
				continue;
			}
	
			// Start defining the inner region
			if (level.At(x, y) == TileType::EMPTY && !isInnerRegionDefined)
			{
				std::vector<Vec2> navPoints = LineTrace(Vec2(x, y), level);
				mNavMesh->AddPointList(navPoints);
	
				isInnerRegionDefined = true;
				continue;
			}
	
			// Define obstacle regions
			if (level.At(x, y) == TileType::OBSTACLE)
			{
				std::vector<Vec2> obPoints = LineTrace(Vec2(x, y), level);
				Obstacle* ob = new Obstacle(obPoints);
				mObstacles.push_back(ob);
	
				continue;
			}
		}
	}

	mNavMesh->Build();
}

void World::Update(float delta)
{
	Draw(lines);
}

void World::Draw(LineRenderer* lines)
{
	for (Obstacle* ob : mObstacles)
	{
		ob->Draw(lines);
	}

	for (PathAgent* agent : mPathAgents)
	{
		agent->Draw(lines);
	}

	if (mNavMesh != nullptr)
	{
		mNavMesh->Draw(lines);
	}
}

std::vector<Vec2> World::LineTrace(Vec2 startPos, Grid& grid)
{
	//float halfSize = grid.GetCellSize() * 0.5f;
	std::vector<Vec2> returnPoints;
	// Add our first point to the list
	returnPoints.push_back(Vec2(startPos.x - 0.5f, startPos.y - 0.5f));

	int primIndex = MoveDirection::RIGHT;
	int secIndex = MoveDirection::UP;

	Vec2 primDir = Vec2(1, 0);
	Vec2 secDir = Vec2(0, -1);
	//Vec2 currentPos = startPos;
	Vec2 currentPos = pos;
	bool shouldSwitchDirection = false;

	// Start moving right
	while (!Vector2IsEqual(currentPos + primDir, startPos))
	{
		if (shouldSwitchDirection)
		{
			primDir = SwitchDirection(primIndex);
			secDir = SwitchDirection(secIndex);
			shouldSwitchDirection = false;
		}

		if ((grid.At(currentPos.x + secDir.x, currentPos.y + secDir.y) == TileType::OBSTACLE || grid.At(currentPos.x + secDir.x, currentPos.y + secDir.y) == TileType::OUTSIDE) &&
			grid.At(currentPos.x + primDir.x, currentPos.y + primDir.y) == TileType::EMPTY)
		{
			// Move forward a tile
			currentPos += primDir;
			pos = currentPos;
		}

		// Hit a concave corner, turn clockwise
		else if ((grid.At(currentPos.x + secDir.x, currentPos.y + secDir.y) == TileType::OBSTACLE || grid.At(currentPos.x + secDir.x, currentPos.y + secDir.y) == TileType::OUTSIDE) &&
			(grid.At(currentPos.x + primDir.x, currentPos.y + primDir.y) == TileType::OBSTACLE || grid.At(currentPos.x + primDir.x, currentPos.y + primDir.y) == TileType::OUTSIDE))
		{
			// Find mid point of tile at primary position and tile at secondary position
			Vec2 pointToAdd = ((currentPos + secDir) + (currentPos + primDir)) * 0.5;
			returnPoints.push_back(pointToAdd);

			// Update movement values
			primIndex = (primIndex + 1) % 4;
			secIndex = (secIndex + 1) % 4;
			shouldSwitchDirection = true;
		}

		// Hit a convex corner
		else if ((grid.At(currentPos.x + secDir.x, currentPos.y + secDir.y) == TileType::OBSTACLE || grid.At(currentPos.x + secDir.x, currentPos.y + secDir.y) == TileType::OUTSIDE))
		{
			// Find mid point of tile at primary position and tile at secondary position
			Vec2 flippedPrim = SwitchDirection(primIndex - 2);

			Vec2 pointToAdd = ((currentPos + secDir) + (currentPos + flippedPrim) * 0.5);
			returnPoints.push_back(pointToAdd);

			// Update movement values
			primIndex = (primIndex - 1) % 4;
			secIndex = (secIndex - 1) % 4;
			shouldSwitchDirection = true;
		}
	}

	return returnPoints;
}

Vec2 World::SwitchDirection(int moveDirIdex)
{
	switch (moveDirIdex)
	{
	case 0:
		return Vec2(1, 0);

	case 1:
		return Vec2(0, 1);

	case 2:
		return Vec2(-1, 0);

	case 3:
		return Vec2(0, -1);
	}
}

