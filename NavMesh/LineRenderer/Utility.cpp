#include "Utility.h"
#include "Obstacle.h"
#include "Edge.h"

#include <cmath>

bool Vector2IsEqual(Vec2 a, Vec2 b)
{
	float tolerance = 0.0001f;
	if (abs(a.x - b.x) >= tolerance || abs(a.y - b.y) >= tolerance) { return false; }

	return true;
}

bool IsPointInObstacle(Vec2 point, std::vector<Obstacle*> obstacles, float levelWidth)
{
	for (Obstacle* ob : obstacles)
	{
		std::vector<Edge> currentObEdges;
		Vec2 endPoint = Vec2(point.x + levelWidth, point.y);

		currentObEdges = ConstructObstacleEdges(ob);

		for (Edge e : currentObEdges)
		{
			float result1 = PseudoCross((endPoint - point), (e.mVerts[0] - point));
			float result2 = PseudoCross((endPoint - point), (e.mVerts[1] - point));

			float result3 = PseudoCross((e.mVerts[1] - e.mVerts[0]), (point - e.mVerts[0]));
			float result4 = PseudoCross((e.mVerts[1] - e.mVerts[0]), (endPoint - e.mVerts[0]));

			if (result1 * result2 < 0 && result3 * result4 < 0)
			{
				return true;
			}
		}
	}

	return false;
}

bool IsPointInConvexObstacle(Vec2 point, std::vector<Obstacle*>& obstacles)
{
	for (Obstacle* ob : obstacles)
	{
		std::vector<float> results;
		Vec2 current{};
		Vec2 next{};
		std::vector<Vec2> conHull = ob->GetPoints();
		for (size_t i = 0; i < conHull.size(); i++)
		{
			current = conHull[i];
			if (i == conHull.size() - 1) { next = conHull[0]; }
			else { next = conHull[i + 1]; }

			Vec2 edgeVector = Vec2{ next.x - current.x, next.y - current.y };
			Vec2 pointEdgeVector = Vec2{ point.x - current.x, point.y - current.y };

			results.push_back(PseudoCross(edgeVector, pointEdgeVector));
		}

		int positive = 0;
		int negative = 0;
		for (int i = 0; i < results.size(); i++)
		{
			if (results[i] > 0) { positive++; }
			else { negative++; }
		}
		if (positive == 0 || negative == 0)
		{
			return true;
		}
	}

	return false;
}

std::vector<Edge> ConstructObstacleEdges(Obstacle* ob)
{
	std::vector<Edge> returnEdges;

	std::vector<Vec2> conHull = ob->GetPoints();
	Vec2 next = Vec2();
	for (size_t i = 0; i < conHull.size(); i++)
	{
		Edge tempEdge;
		if (i == conHull.size() - 1) { next = conHull[0]; }
		else { next = conHull[i + 1]; }

		tempEdge.mVerts[0] = conHull[i];
		tempEdge.mVerts[1] = next;

		returnEdges.push_back(tempEdge);
	}

	return returnEdges;
}
