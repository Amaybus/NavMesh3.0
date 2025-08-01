#pragma once

#include "Node.h"

class Obstacle;
class PathAgent;

std::vector<Vec2> AStarSearch(PathAgent* agent, Node* startNode, Node* endNode, const std::vector<Obstacle*>& obstacles);
std::vector<Vec2> StringPull(PathAgent* agent, std::vector<Node*>& path, const std::vector<Obstacle*>& obstacles);

Vec2 FindShortestPortal(const Vec2& left, const Vec2& right, Vec2& endPos);
Vec2 ReturnRightPoint(const std::vector<Vec2>& points, const Vec2& position);
Vec2 ReturnLeftPoint(const std::vector<Vec2>& points, const Vec2& position);
