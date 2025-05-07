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

float FindAngle(Vec2 point, Vec2 next, Vec2 previous)
{
	// Normalise vectors to allow acos to work (range of -1 to 1)
	point = point.Normalise();
	next = next.Normalise();
	previous = previous.Normalise();

	// Find lengths for each point (make triangle)
	float ap = abs((next - point).GetMagnitude());
	float an = abs((previous - point).GetMagnitude());
	float np = abs((previous - next).GetMagnitude());

	// Law of cosine to find angle of a
	// acos gives up the results in radians
	float result = acos(((ap * ap) + (an * an) - (np * np)) / (2 * (ap * an)));

	// Convert to degrees
	result *= (180 / PI);

	return result;
}

bool IsPointInObstacle(Vec2 point, std::vector<Obstacle*>& obstacles, float levelWidth)
{
	int count = 0;

	for (Obstacle* ob : obstacles)
	{
		std::vector<TriEdge> currentObEdges;
		Vec2 endPoint = Vec2(point.x + levelWidth, point.y);

		currentObEdges = ConstructObstacleEdges(ob);

		for (TriEdge e : currentObEdges)
		{
			float result1 = PseudoCross((endPoint - point), (e.mPoints[0] - point));
			float result2 = PseudoCross((endPoint - point), (e.mPoints[1] - point));

			float result3 = PseudoCross((e.mPoints[1] - e.mPoints[0]), (point - e.mPoints[0]));
			float result4 = PseudoCross((e.mPoints[1] - e.mPoints[0]), (endPoint - e.mPoints[0]));

			
			if (result1 * result2 < 0 && result3 * result4 < 0)
			{
				count++;
			}
		}
	}

	if (count % 2 > 0)
	{
		return true;
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

std::vector<TriEdge> ConstructObstacleEdges(Obstacle* ob)
{
	std::vector<TriEdge> returnEdges;

	std::vector<Vec2> conHull = ob->GetPoints();
	Vec2 next = Vec2();
	for (size_t i = 0; i < conHull.size(); i++)
	{
		TriEdge tempEdge;
		if (i == conHull.size() - 1) { next = conHull[0]; }
		else { next = conHull[i + 1]; }

		tempEdge.mPoints[0] = conHull[i];
		tempEdge.mPoints[1] = next;

		returnEdges.push_back(tempEdge);
	}

	return returnEdges;
}

