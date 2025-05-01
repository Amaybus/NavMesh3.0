#pragma once

#include "Vec2.h"
#include <vector>

struct Triangle;
struct Node;

struct Edge
{
	Node* mTarget;
	float mCost;

	Edge(Node* target, float cost) : mTarget(target), mCost(cost) {}
};

struct Node
{
	// Position will be triangle centre
	Vec2 mPosition;
	// Connections will be the triangles that share an edge
	std::vector<Edge> mConnections;

	Triangle& mTriangle;

	// Used in pathfinding calculations
	Node* mPrevious = nullptr;

	// Distance from current node to this node
	float gScore = 0;
	// Distance from this node to end pos
	float hScore = 0;
	// F = G + H
	float fScore = 0;

	Node(Vec2 position, Triangle& triangle) : mPosition(position), mTriangle(triangle) {}
	Node(float xPos, float yPos, Triangle& triangle) : mPosition(Vec2(xPos, yPos)), mTriangle(triangle) {}

	void ConnectToNode(Node* other, float cost) { mConnections.push_back(Edge(other, cost)); }
	std::vector<Edge>& GetConnections() { return mConnections; }
};
