#include "NavigationUtilities.h"

#include "Utility.h"
#include "PathAgent.h"
#include "World.h"
#include <iostream>
#include <algorithm>

std::vector<Vec2> AStarSearch(PathAgent* agent, Node* startNode, Node* endNode, const std::vector<Obstacle*>& obstacles)
{
	// https://cdn.aaai.org/AAAI/2006/AAAI06-148.pdf

	std::vector<Vec2> returnPath;

	if (startNode == nullptr || endNode == nullptr) { std::cout << "Start or end node is null."; return returnPath; }

	if (startNode == endNode)
	{
		returnPath.push_back(startNode->mPosition);
		return returnPath;
	}

	// Initialise start node
	startNode->gScore = 0;
	startNode->mPrevious = nullptr;

	// Create temp lists to sort which nodes we are visiting/visited
	std::vector<Node*> mOpenList;
	std::vector<Node*> mClosedList;

	mOpenList.push_back(startNode);

	while (!mOpenList.empty())
	{
		// (this currently works but need to be adjusted as we are not comparing the right thing)
		// Sort by hscore
		std::sort(mOpenList.begin(), mOpenList.end(), [](Node* lhs, Node* rhs) { return lhs->hScore < rhs->hScore; });
		Node* currentNode = mOpenList[0];

		// Reached our destination
		if (currentNode == endNode) { break; }

		// Remove the node we are checking from our list and put into our closed list
		mOpenList.erase(mOpenList.begin());
		mClosedList.push_back(currentNode);

		// Check each connection to find the lowest costing path of travel
		for (Edge c : currentNode->mConnections)
		{
			// Skip the node if we have already visited it
			if (std::find(mClosedList.begin(), mClosedList.end(), c.mTarget) != mClosedList.end()) { continue; }

			c.mTarget->gScore = currentNode->gScore + c.mCost;
			c.mTarget->hScore = (endNode->mPosition - c.mTarget->mPosition).GetMagnitude();
			c.mTarget->fScore = c.mTarget->gScore + c.mTarget->hScore;

			// Haven't visited the node yet
			if (std::find(mOpenList.begin(), mOpenList.end(), c.mTarget) == mOpenList.end())
			{
				c.mTarget->gScore = currentNode->gScore;
				c.mTarget->fScore = currentNode->fScore;
				c.mTarget->mPrevious = currentNode;
				mOpenList.push_back(c.mTarget);
			}

			// Already visited, compare scores to find the shortest path
			else if (currentNode->fScore < c.mTarget->fScore)
			{
				c.mTarget->gScore = currentNode->gScore;
				c.mTarget->fScore = currentNode->hScore;
				c.mTarget->mPrevious = currentNode;
			}
		}
	}

	// Check to see if the path is navigatable
	if (std::find(mOpenList.begin(), mOpenList.end(), endNode) == mOpenList.end())
	{
		std::cout << "Cannot path to end position. \n";
		return std::vector<Vec2>{};
	}

	// Reverse the path
	std::vector<Node*> path;
	Node* currentNode = endNode;

	while (currentNode != nullptr)
	{
		path.insert(path.begin(), currentNode);
		currentNode = currentNode->mPrevious;
	}

	std::vector<Vec2> vectorPath = StringPull(agent,path, obstacles);
	return vectorPath;
}

std::vector<Vec2> StringPull(PathAgent* agent, std::vector<Node*>& path, const std::vector<Obstacle*>& obstacles)
{
	agent->ClearPortals();
	agent->pathedges.clear();

	std::vector<std::vector<Vec2>> portals;

	for (size_t i = path.size() - 1; i >= 1; i--)
	{
		std::vector<Vec2> edge = FindTwoCommonVerts(path[i]->mTriangle, path[i - 1]->mTriangle);
		agent->pathedges.push_back(edge);
		portals.insert(portals.begin(), edge);
	}

	Vec2 funnelTip = agent->GetPosition();
	Vec2 endPos;
	if (IsPointInObstacle(agent->GetEndPos(), agent->GetWorld()->GetObstacles(), 10000))
	{
		endPos = path[path.size() - 1]->mPosition;
	}
	else { endPos = agent->GetEndPos(); }


	// check to see if a direct line of sight is available
	// if yes, move directly to end pos
	std::vector<Vec2> returnPath;
	returnPath.push_back(funnelTip);
	if (!DoLinesIntersect(funnelTip, endPos, obstacles))
	{
		returnPath.push_back(endPos);
		return returnPath;
	}

	// otherwise do funnel algorithm
	float agentRadius = agent->GetRadius() * 2;
	Vec2 portalRight = ReturnRightPoint(portals[0], funnelTip);
	Vec2 portalLeft = ReturnLeftPoint(portals[0], funnelTip);
	Vec2 direction = (portalRight - portalLeft).Normalise();

	portalRight = portalRight - direction * agentRadius;
	portalLeft = portalLeft + direction * agentRadius;

	agent->AddPortalRight(portalRight);
	agent->AddPortalLeft(portalLeft);

	Vec2 left;
	Vec2 right;

	for (int i = 0; i < portals.size(); i++)
	{
		left = ReturnLeftPoint(portals[i], path[i]->mPosition);
		right = ReturnRightPoint(portals[i], path[i]->mPosition);
		direction = (right - left).Normalise();

		left += direction * agentRadius;
		right -= direction * agentRadius;

		agent->AddPortalRight(right);
		agent->AddPortalLeft(left);

		if (PseudoCross(portalLeft - funnelTip, left - funnelTip) <= 0.0f)
		{
			portalLeft = left;
		}
		else
		{
			returnPath.push_back(portalLeft);

			funnelTip = portalLeft;

			if (!DoLinesIntersect(funnelTip, endPos, obstacles))
			{
				returnPath.push_back(path[path.size() - 1]->mPosition);
				return returnPath;
			}
		}

		if (PseudoCross(portalRight - funnelTip, right - funnelTip) >= 0.0f)
		{
			portalRight = right;
		}
		else
		{
			returnPath.push_back(portalRight);
			funnelTip = portalRight;

			if (!DoLinesIntersect(funnelTip, endPos, obstacles))
			{
				returnPath.push_back(path[path.size() - 1]->mPosition);
				return returnPath;
			}
		}
	}

	returnPath.push_back(FindShortestPortal(portalLeft, portalRight, endPos));
	path.erase(path.begin() + path.size() - 1);
	returnPath.push_back(endPos);
	return returnPath;
}

Vec2 FindShortestPortal(const Vec2& left, const Vec2& right, Vec2& endPos)
{
	float mag1 = (left - endPos).GetMagnitude();
	float mag2 = (right - endPos).GetMagnitude();

	return mag1 > mag2 ? right : left;
}

Vec2 ReturnRightPoint(const std::vector<Vec2>& points, const Vec2& position)
{
	Vec2 next = points[1];
	Vec2 current = position;
	Vec2 previous = points[0];

	Vec2 ab = current - next;
	Vec2 ac = previous - next;

	float result = PseudoCross(ab, ac);
	if (result > 0) { return points[0]; }
	else { return points[1]; }
}

Vec2 ReturnLeftPoint(const std::vector<Vec2>& points, const Vec2& position)
{
	Vec2 next = points[1];
	Vec2 current = position;
	Vec2 previous = points[0];

	Vec2 ab = current - next;
	Vec2 ac = previous - next;

	float result = PseudoCross(ab, ac);
	if (result < 0) { return points[0]; }
	else { return points[1]; }
}