#pragma once

#include "Node.h"
#include <vector>

class Obstacle;
class LineRenderer;
class NavMesh;
struct Triangle;

class NodeGraph
{
	// List of all the nodes in the map
	NavMesh* mNavMesh = nullptr;
	std::vector<Node*> mNodes;

public:
	NodeGraph(NavMesh* navMesh);
	Node* GetNodeAt(int index) const;
	void ConstructNodeNeighbours(std::vector<Triangle>& triangleList);

	void AddNode(Node* node) { mNodes.push_back(node); }

	std::vector<Node*> GetNodeList() { return mNodes; }
	int GetNodeListSize() const { return (int)mNodes.size(); }

	Node* GetClosestNode(Vec2 pos);
	Node* GetNode(Vec2 pos);
	NavMesh* GetNavMesh() { return mNavMesh; }
};

