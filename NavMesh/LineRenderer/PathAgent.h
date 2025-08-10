#pragma once


#include "Vec2.h"
#include "Colour.h"
#include "NodeGraph.h"
#include "Node.h"
#include <vector>

class World;
class LineRenderer;

class PathAgent
{
	Vec2 mPosition;
	float mSpeed = 1000;
	Node* mCurrentNode;
	NodeGraph* mNodeGraph;
	World* mCurrentWorld;
	std::vector<Node*> mNodePath;
	std::vector<Vec2> mPointPath;

	Colour mColour;
	int mCurrentIndex;
	float mRadius = 0;
	Vec2 mEndPos;

	// DEBUG
	std::vector<Vec2> portalLeft;
	std::vector<Vec2> portalRight;


public:
	PathAgent(NodeGraph* nodeGraph, float width, Colour colour, World* world);

	void Update(float deltaTime);
	void Draw(LineRenderer* lines);

	void NavigateNodePath(float deltaTime);
	void NavigatePointPath(float deltaTime);

	void GoToNode(Node* destination);
	void GoTo(Vec2 pos);

	void SetSpeed(float speed) { mSpeed = speed; }
	void SetColour(const char* colour);

	float GetRadius() { return mRadius; }
	Vec2 GetPathEnd();
	Vec2 GetPosition() { return mPosition; }
	Vec2 GetEndPos() { return mEndPos; }

	void AddPortalLeft(Vec2 portal) { portalLeft.push_back(portal); }
	void AddPortalRight(Vec2 portal) { portalRight.push_back(portal); }
	void ClearPortals() { portalLeft.clear(); portalRight.clear(); }
	World* GetWorld() { return mCurrentWorld; }
	std::vector<std::vector<Vec2>> pathedges;
	std::vector<Vec2> GetPath() { return mPointPath; }


};

