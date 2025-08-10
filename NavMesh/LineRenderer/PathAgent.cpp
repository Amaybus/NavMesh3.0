#include "PathAgent.h"

#include "dirent.h"
#include "World.h"
#include "NavigationUtilities.h"
#include "TextStream.h"
#include "LineRenderer.h"

PathAgent::PathAgent(NodeGraph* nodeGraph, float width, Colour colour, World* world) : mNodeGraph(nodeGraph), mRadius(width), mColour(colour), mCurrentWorld(world)
{
	mCurrentNode = mNodeGraph->GetNodeAt(0);
	mPosition = mCurrentNode->mPosition;
}

void PathAgent::Update(float deltaTime)
{
	int randNum = rand() % mNodeGraph->GetNodeList().size() - 1;
	if (randNum < 0) { randNum = 0; }
	//if (mNodePath.empty() && mPointPath.empty()) { return; }

	if(mPointPath.empty()) { GoToNode(mNodeGraph->GetNodeAtIndex(randNum)); }

	else if (!mNodePath.empty()) { NavigateNodePath(deltaTime); }
	else { NavigatePointPath(deltaTime); }
}

void PathAgent::NavigateNodePath(float deltaTime)
{
	Node* nextNode;
	if (mCurrentIndex + 1 >= (int)mNodePath.size())
	{
		nextNode = mCurrentNode;
	}
	else
	{
		nextNode = mNodePath[mCurrentIndex + 1];
	}

	Vec2 mag = nextNode->mPosition - mPosition;
	float distance = sqrtf((mag.x * mag.x) + (mag.y * mag.y));
	mag /= distance;

	distance -= mSpeed * deltaTime;

	if (distance > 0)
	{
		mPosition += (mSpeed * deltaTime * mag);
	}
	else
	{
		mCurrentIndex++;
		if (mCurrentIndex >= (int)mNodePath.size())
		{
			mPosition = mCurrentNode->mPosition;
			mNodePath.clear();
		}
		else
		{
			Node* previousNode = mCurrentNode;
			mCurrentNode = mNodePath[mCurrentIndex];

			Vec2 nextMag = previousNode->mPosition - mCurrentNode->mPosition;
			float nextDistance = sqrtf((nextMag.x * nextMag.x) * (nextMag.y * nextMag.y));

			nextMag /= nextDistance;
			mPosition = mCurrentNode->mPosition - distance * nextMag;
		}
	}
}

void PathAgent::NavigatePointPath(float deltaTime)
{
	Vec2 nextPos;
	if (mCurrentIndex + 1 >= (int)mPointPath.size())
	{
		nextPos = mPosition;
	}
	else
	{
		nextPos = mPointPath[mCurrentIndex + 1];
	}

	Vec2 mag = nextPos - mPosition;
	float distance = sqrtf((mag.x * mag.x) + (mag.y * mag.y));
	mag /= distance;

	distance -= mSpeed * deltaTime;

	if (distance > 0)
	{
		mPosition += (mSpeed * deltaTime * mag);
	}
	else
	{
		mCurrentIndex++;
		if (mCurrentIndex >= (int)mPointPath.size())
		{
			mPosition = mPointPath[mCurrentIndex - 1];
			mPointPath.clear();
		}
		else
		{
			Node* previousNode = mCurrentNode;
			mCurrentNode = mNodeGraph->GetClosestNode(mPointPath[mCurrentIndex]);

			Vec2 nextMag = previousNode->mPosition - mPosition;
			float nextDistance = sqrtf((nextMag.x * nextMag.x) * (nextMag.y * nextMag.y));

			nextMag /= nextDistance;
			mPosition = mPointPath[mCurrentIndex] - distance * nextMag;
		}
	}
}

void PathAgent::Draw(LineRenderer* lines)
{
	lines->DrawCircle(mPosition, mRadius, mColour);

	//for (int i = 1; i < mPointPath.size(); i++)
	//{
	//	lines->DrawLineSegment(mPointPath[i - 1], mPointPath[i], Colour::ORANGE);
	//
	//	Vec2 pos = ((mPointPath[i - 1] + mPointPath[i]) * 0.5f);
	//
	//	lines->FinishLineStrip();
	//}

	// DEBUG DRAWING
	//for (Vec2 p : portalRight)
	//{
	//	lines->DrawCircle(p, 100, Colour::RED);
	//}
	//
	//for (Vec2 p : portalLeft)
	//{
	//	lines->DrawCircle(p, 100, Colour::BLUE);
	//}
	//
	//for (std::vector<Vec2> e : pathedges)
	//{
	//	lines->DrawLineWithArrow(e[0], e[1], 500);
	//}
}

void PathAgent::GoToNode(Node* destination)
{
	if (destination == nullptr) { return; }
	mCurrentIndex = 0;
	mCurrentNode = mNodeGraph->GetClosestNode(mPosition);
	mPointPath = AStarSearch(this, mCurrentNode, destination, mCurrentWorld->GetObstacles());
}

void PathAgent::GoTo(Vec2 pos)
{
	mEndPos = pos;
	Node* end = mNodeGraph->GetClosestNode(pos);
	GoToNode(end);
}

void PathAgent::SetColour(const char* colour)
{
	if (colour == "CYAN")
	{
		mColour = Colour::CYAN;
	}
	else if (colour == "BLUE")
	{
		mColour = Colour::BLUE;
	}
	else if (colour == "YELLOW")
	{
		mColour = Colour::YELLOW;
	}
	else if (colour == "ORANGE")
	{
		mColour = Colour::ORANGE;
	}
	else if (colour == "MAGENTA")
	{
		mColour = Colour::MAGENTA;
	}
	else
	{
		mColour = Colour::GREY;
	}
}

Vec2 PathAgent::GetPathEnd()
{
	if (mPointPath.empty()) { return mPosition; }
	else { return mPointPath[mPointPath.size() - 1]; }
}
