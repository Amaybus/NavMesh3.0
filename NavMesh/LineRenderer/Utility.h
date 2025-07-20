#pragma once

#include "Vec2.h"

#include <vector>

class Obstacle;
struct TriEdge;
struct Triangle;

bool Vector2IsEqual(Vec2 a, Vec2 b);

float FindAngle(Vec2 point, Vec2 next, Vec2 previous);

bool IsPointInObstacle(Vec2 point, const std::vector<Obstacle*>& obstacles, float levelWidth);
bool IsPointInConvexObstacle(Vec2 point, std::vector<Obstacle*>& obstacles);
bool IsTriangleInObstacle(Triangle* triangle, const std::vector<Obstacle*>& obstacles);
std::vector<TriEdge> ConstructObstacleEdges(Obstacle* ob);
std::vector<Vec2> AddBufferToObstacles(std::vector<Obstacle*>& obstacles);
