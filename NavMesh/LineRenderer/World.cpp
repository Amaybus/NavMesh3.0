#include "World.h"
#include "Utility.h"
#include "Grid.h"
#include "Triangle.h"
#include "ImGui.h"
#include "Key.h"
#include <algorithm>
#include "PathAgent.h"
#include "DelaunayTriangulation.h"

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

float GetRandomFloat(float min, float max)
{
	return (rand() / (float)RAND_MAX) * (max - min) + min;
}

void World::Initialise()
{
	// Create level from file
	Grid level(1000);
	level.LoadFromImage("Test.png");

	// Create nav mesh with dimensions of the file
	NavigationMesh* navMesh = new NavigationMesh();
	mNavMesh = navMesh;

	// Border 
	mNavMesh->AddPoint(Vec2(-0.5f, level.GetHeight() - 0.5f));
	mNavMesh->AddPoint(Vec2(level.GetWidth() - 0.5f, level.GetHeight() - 0.5f));
	mNavMesh->AddPoint(Vec2(level.GetWidth() - 0.5f, -0.5f));
	mNavMesh->AddPoint(Vec2(-0.5f, -0.5f));

	// Init the level
	for (int y = 0; y < level.GetHeight(); y++)
	{
		for (int x = 0; x < level.GetWidth(); x++)
		{
			// Define obstacle regions
			if (level.At(x, y) == TileType::OBSTACLE)
			{
				// Skip any tiles which are already in an obstacle
				if (IsPointInObstacle(Vec2((float)x, (float)y), mObstacles, level.GetWidth())) { continue; }

				std::vector<Vec2> obPoints = LineTrace(Vec2((float)x, (float)y), level, TileType::OBSTACLE);

				Obstacle* ob = new Obstacle(obPoints);
				mObstacles.push_back(ob);

				continue;
			}
		}
	}

	mNavMesh->Build(mObstacles);

	// scale points and obstacles to adjust the level size
	for (Obstacle* ob : mObstacles)
	{
		std::vector<Vec2>& points = ob->GetPoints();
		for (Vec2& v : points)
		{
			v = Vec2((v.x - (level.GetWidth() * 0.5f)), -(v.y - (level.GetHeight() * 0.5f))) * level.GetCellSize();
		}
	}

	for (Vec2& v : mNavMesh->GetPoints())
	{
		v = Vec2((v.x - (level.GetWidth() * 0.5f)), -(v.y - (level.GetHeight() * 0.5f))) * level.GetCellSize();
	}

	for (Triangle* t : mNavMesh->GetTriangles())
	{
		for (Vec2& v : t->mPoints)
		{
			v = Vec2((v.x - (level.GetWidth() * 0.5f)), -(v.y - (level.GetHeight() * 0.5f))) * level.GetCellSize();
		}
	}

	// Construct node graph using the navmesh
	mNodeGraph = new NodeGraph(mNavMesh);
}

void World::Update(float delta)
{
	Draw(lines);

	if (ImGui::Begin("Tools"))
	{
		if (ImGui::CollapsingHeader("CIRCUMCIRLCE"))
		{
			ImGui::Checkbox("Show circumcircle", &bShowCircumcircle);

			if (bShowCircumcircle)
			{
				ImGui::SliderInt("Triangle Index", &mTriangleIndex, 0, (int)mNavMesh->GetNumberOfTriangles() - 1);
			}
		}
	}
	
	//if (ImGui::CollapsingHeader("NAVMESH"))
	//{
	//	ImGui::Checkbox("Show triangles", &mNavMesh->bShowTriangles);
	//	ImGui::Checkbox("Show triangle index", &mNavMesh->bShowTriangleNumbers);
	//	ImGui::Checkbox("Show points", &mNavMesh->bShowPoints);
	//	ImGui::Checkbox("Show point position", &mNavMesh->bShowPointPositionValue);
	//}

	if (ImGui::CollapsingHeader("AGENT GENERATION"))
	{
		if (ImGui::Button("Add Agent", ImVec2(100, 20)))
		{
			PathAgent* agent = new PathAgent(mNodeGraph, 300.0f, Colour::BLUE, this);
			mPathAgents.push_back(agent);
		}
	}

	if (ImGui::CollapsingHeader("PATHFINDING"))
	{
		ImGui::Checkbox("Show single node connections", &bShowSingleNodeConnections);
		ImGui::Checkbox("Show all node connections", &bShowAllNodeConnections);
		ImGui::Checkbox("Show agent paths", &bShowAgentPaths);

		if (bShowAgentPaths && mNumberOfAgents > 0)
		{
			ImGui::SliderInt("Node Index", &mAgentIndex, 0, mNumberOfAgents - 1);
		}

		if (bShowSingleNodeConnections && mNodeGraph)
		{
			bShowAllNodeConnections = false;
			ImGui::SliderInt("Node Index", &mNodeIndex, 0, mNodeGraph->GetNodeListSize() - 1);
		}
	}

	ImGui::End();

	for (PathAgent* agents : mPathAgents)
	{
		agents->Update(delta);
	}
}

void World::Draw(LineRenderer* lines)
{
	for (PathAgent* agent : mPathAgents)
	{
		agent->Draw(lines);
	}

	if (mNavMesh != nullptr)
	{
		mNavMesh->Draw(lines);
	}

	for (Obstacle* ob : mObstacles)
	{
		ob->Draw(lines);
	}

	if (bShowCircumcircle)
	{
		DrawCircumcircles(lines);
	}

	if (bShowAllNodeConnections)
	{
		for (Node* n : mNodeGraph->GetNodeList())
		{
			for (Edge e : n->mConnections)
			{
				lines->DrawLineWithArrow(n->mPosition, e.mTarget->mPosition, Colour::BLUE.Lighten(), 100);
			}
		}
	}

	if (bShowSingleNodeConnections)
	{
		Node* node = mNodeGraph->GetNodeAt(mNodeIndex);
		for (Edge e : node->GetConnections())
		{
			lines->DrawLineWithArrow(node->mPosition, e.mTarget->mPosition, Colour::BLUE.Lighten(), 100);
		}
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
		if (shouldSwitchDirection)
		{
			primDir = SwitchDirection(primIndex);
			secDir = SwitchDirection(secIndex);
			shouldSwitchDirection = false;

			// Handles any single tile obstacles
			if (returnPoints[0] == returnPoints[returnPoints.size() - 1])
			{
				returnPoints.erase(returnPoints.begin() + returnPoints.size() - 1);
				return returnPoints;
			}

			if (Vector2IsEqual(currentPos + primDir, startPos))
			{
				break;
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

		if (Vector2IsEqual(currentPos + primDir, startPos) && grid.At((currentPos + primDir + secDir).x, (currentPos + primDir + secDir).y) == tileType)
		{
			primDir = SwitchDirection(primIndex);
			secDir = SwitchDirection(secIndex);
			shouldSwitchDirection = false;
			currentPos += primDir;
		}
	}

	Vec2 pointToAdd = Vec2((currentPos + secDir + primDir) + (currentPos + primDir + primDir)) * 0.5;
	if (!Vector2IsEqual(pointToAdd, returnPoints[0]))
	{
		returnPoints.push_back(pointToAdd);
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

	return Vec2(0, 0);
}

void World::DrawCircumcircles(LineRenderer* lines)
{
	Triangle* tri = mNavMesh->GetTriangleAtIndex(mTriangleIndex);

	Vec2 a = tri->mPoints[0];
	Vec2 b = tri->mPoints[1];
	Vec2 c = tri->mPoints[2];

	float d = 2 * ((a.x * (b.y - c.y)) + (b.x * (c.y - a.y)) + (c.x * (a.y - b.y)));
	float x = ((((a.x * a.x) + (a.y * a.y)) * (b.y - c.y)) + ((b.x * b.x + b.y * b.y) * (c.y - a.y)) + (((c.x * c.x) + (c.y * c.y)) * (a.y - b.y))) / d;
	float y = (((a.x * a.x + a.y * a.y) * (c.x - b.x)) + ((b.x * b.x + b.y * b.y) * (a.x - c.x)) + ((c.x * c.x + c.y * c.y) * (b.x - a.x))) / d;

	Vec2 circumcenter = Vec2{ x,y };

	// Get lengths of each side
	float ab = (tri->mPoints[1] - tri->mPoints[0]).GetMagnitude();
	float bc = (tri->mPoints[2] - tri->mPoints[1]).GetMagnitude();
	float ca = (tri->mPoints[0] - tri->mPoints[2]).GetMagnitude();

	// Triangles half perimeter
	float s = (ab + bc + ca) * 0.5f;

	// Find area using Heron's formula
	float area = (float)sqrt((s * (s - ab) * (s - bc) * (s - ca)));

	// Find the radius of the circumcircle
	float radius = (ab * bc * ca) / (4 * area);


	lines->DrawCircle(circumcenter, radius, Colour::MAGENTA);
	lines->DrawCircle(a, 0.1f, Colour::RED);
	lines->DrawCircle(b, 0.1f, Colour::RED);
	lines->DrawCircle(c, 0.1f, Colour::RED);


	Vec2 normals[3];
	normals[0] = (tri->mEdgeList[0].mPoints[0] - tri->mEdgeList[0].mPoints[1]).GetRotatedBy270().GetNormalised();
	normals[1] = (tri->mEdgeList[1].mPoints[0] - tri->mEdgeList[1].mPoints[1]).GetRotatedBy270().GetNormalised();
	normals[2] = (tri->mEdgeList[2].mPoints[0] - tri->mEdgeList[2].mPoints[1]).GetRotatedBy270().GetNormalised();

	Vec2 edgeMidPoints[3];
	edgeMidPoints[0] = ((tri->mEdgeList[0].mPoints[0] * 1.22f) + tri->mEdgeList[0].mPoints[1]) / 2.22f;
	edgeMidPoints[1] = ((tri->mEdgeList[1].mPoints[0] * 1.22f) + tri->mEdgeList[1].mPoints[1]) / 2.22f;
	edgeMidPoints[2] = ((tri->mEdgeList[2].mPoints[0] * 1.22f) + tri->mEdgeList[2].mPoints[1]) / 2.22f;

	for (int i = 0; i < 3; i++)
	{
		lines->DrawLineWithArrow(edgeMidPoints[i] - (normals[i] * 0.01f), edgeMidPoints[i] + normals[i] * 2, Colour::MAGENTA, 0.1f);
	}
}

void World::OnLeftClick()
{
	if (mPathAgents.empty()) { return; }
	mPathAgents[mAgentIndex]->GoTo(cursorPos);
}
