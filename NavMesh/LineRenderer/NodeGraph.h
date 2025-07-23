#pragma once

#include "Node.h"
#include <vector>

class Obstacle;
class LineRenderer;
class NavigationMesh;
struct Triangle;

class NodeGraph
{
	// List of all the nodes in the map
	NavigationMesh* mNavMesh = nullptr;
	std::vector<Node*> mNodes;

public:
	NodeGraph(NavigationMesh* navMesh);
	Node* GetNodeAt(int index) const;
	void ConstructNodeNeighbours(std::vector<Triangle*>& triangleList);

	void AddNode(Node* node) { mNodes.push_back(node); }

	std::vector<Node*> GetNodeList() { return mNodes; }
	int GetNodeListSize() const { return (int)mNodes.size(); }

	Node* GetClosestNode(Vec2 pos);
	Node* GetNode(Vec2 pos);
	NavigationMesh* GetNavMesh() { return mNavMesh; }
};

