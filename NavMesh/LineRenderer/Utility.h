#pragma once

#include "Vec2.h"

#include <vector>

class Obstacle;
struct Edge;

bool Vector2IsEqual(Vec2 a, Vec2 b);

bool IsPointInObstacle(Vec2 point, std::vector<Obstacle*> obstacles, float levelWidth);
bool IsPointInConvexObstacle(Vec2 point, std::vector<Obstacle*>& obstacles);
std::vector<Edge> ConstructObstacleEdges(Obstacle* ob);
