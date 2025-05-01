#pragma once

#include "Application.h"
#include "Grid.h"
#include "Obstacle.h"
#include "PathAgent.h"
#include "NodeGraph.h"
#include "NavigationMesh.h"
#include "Vec2.h"

#include <vector>

class World : public Application
{
	enum MoveDirection
	{
		RIGHT,
		DOWN,
		LEFT,
		UP
	};


	NodeGraph* mNodeGraph = nullptr;
	NavigationMesh* mNavMesh = nullptr;
	std::vector<Obstacle*> mObstacles;
	std::vector<PathAgent*> mPathAgents;

	bool shouldrun = false;
	Vec2 pos = Vec2(2,2); 

public:
	World();
	~World();
	World(const World& other) = default;
	World& operator=(const World& other) = default;

	void Initialise() override;
	void Update(float delta) override;
	void Draw(LineRenderer* lines);

	std::vector<Vec2> LineTrace(Vec2 startPos, Grid& grid);
	Vec2 SwitchDirection(int moveDirIdex);
	std::vector<Obstacle*>& GetObstacles() { return mObstacles; }
};

