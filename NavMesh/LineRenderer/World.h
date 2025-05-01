#pragma once

#include "Application.h"
#include "Grid.h"
#include "PathAgent.h"
#include "NodeGraph.h"
#include "NavMesh.h"

#include <vector>

class World : public Application
{
	NodeGraph* mNodeGraph = nullptr;
	NavMesh* mNavMesh = nullptr;
	std::vector<Obstacle*> mObstacles;
	std::vector<PathAgent*> mPathAgents;

public:
	World();
	~World();
	World(const World& other) = default;
	World& operator=(const World& other) = default;

	void Initialise() override;
	void Update(float delta) override;
	void Draw(LineRenderer* lines);

	std::vector<Vec2> LineTrace(Vec2 startPos, Grid& grid);
};

