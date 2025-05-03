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
	Grid level(1000);
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
				std::vector<Vec2> navPoints = LineTrace(Vec2(x, y), level, TileType::EMPTY);
				mNavMesh->AddPointList(navPoints);

isInnerRegionDefined = true;
continue;
			}

			// Define obstacle regions
			if (level.At(x, y) == TileType::OBSTACLE)
			{
				// Skip any tiles which are already in an obstacle
				if (IsPointInObstacle(Vec2(x, y), mObstacles, level.GetWidth())) { continue; }

				std::vector<Vec2> obPoints = LineTrace(Vec2(x, y), level, TileType::OBSTACLE);
				Obstacle* ob = new Obstacle(obPoints);
				mObstacles.push_back(ob);

				continue;
			}
		}
	}

	// scale points and obstacles to fix the level size
	for (Obstacle* ob : mObstacles)
	{
		std::vector<Vec2>& points = ob->GetPoints();
		for (Vec2& v : points)
		{
			//v = Vec2(v.x, -(v.y)) * level.GetCellSize();
			v = Vec2((v.x - (level.GetWidth() * 0.5f)), -(v.y - (level.GetHeight() * 0.5f))) * level.GetCellSize();
		}
	}

	for (Vec2& v : mNavMesh->GetPoints())
	{
		v = Vec2((v.x - (level.GetWidth() * 0.5f)), -(v.y - (level.GetHeight() * 0.5f))) * level.GetCellSize();
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

std::vector<Vec2> World::LineTrace(Vec2 startPos, Grid& grid, TileType tileType)
{
	std::vector<Vec2> returnPoints;
	// Add our first point to the list
	returnPoints.push_back(Vec2(startPos.x - 0.5f, startPos.y - 0.5f));

	int primIndex = MoveDirection::RIGHT;
	int secIndex = MoveDirection::UP;

	Vec2 primDir = Vec2(1, 0);
	Vec2 secDir = Vec2(0, -1);
	Vec2 currentPos = startPos;
	bool shouldSwitchDirection = false;

	// Start moving right
	while (!Vector2IsEqual(currentPos + primDir, startPos))
	{
		if (returnPoints.size() == 3)
		{
			int i = 0;
		}

		if (shouldSwitchDirection)
		{
			primDir = SwitchDirection(primIndex);
			secDir = SwitchDirection(secIndex);
			shouldSwitchDirection = false;

			if (Vector2IsEqual(currentPos + primDir, startPos)) { break; }

			// Handles any single tile obstacles
			if (returnPoints[0] == returnPoints[returnPoints.size() - 1])
			{
				returnPoints.erase(returnPoints.begin() + returnPoints.size() - 1);
				return returnPoints;
			}

			// Only move forward if it is available to us
			if (grid.At(currentPos.x + primDir.x, currentPos.y + primDir.y) == tileType)
			{
				currentPos += primDir;
			}
		}

		if (grid.At(currentPos.x + secDir.x, currentPos.y + secDir.y) != tileType && grid.At(currentPos.x + primDir.x, currentPos.y + primDir.y) == tileType)
		{
			// Move forward a tile
			currentPos += primDir;
		}

		// Hit a concave corner, turn clockwise
		else if (grid.At(currentPos.x + secDir.x, currentPos.y + secDir.y) != tileType && grid.At(currentPos.x + primDir.x, currentPos.y + primDir.y) != tileType)
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
		else if ((grid.At(currentPos.x + secDir.x, currentPos.y + secDir.y) == tileType && grid.At(currentPos.x + secDir.x, currentPos.y + secDir.y) == tileType))
		{
			// Find mid point of tile at primary position and tile at secondary position
			Vec2 flippedPrim = SwitchDirection((primIndex + 2) % 4);

			Vec2 pointToAdd = (((currentPos + secDir) + (currentPos + flippedPrim)) * 0.5);
			returnPoints.push_back(pointToAdd);

			// Update movement values
			primIndex = (primIndex + 3) % 4;
			secIndex = (secIndex + 3) % 4;
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

