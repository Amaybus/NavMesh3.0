#include "Utility.h"
#include "Obstacle.h"
#include "Triangle.h"
#include "TriEdge.h"

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

bool IsPointInObstacle(Vec2 point, const std::vector<Obstacle*>& obstacles, int levelWidth)
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

bool IsTriangleInObstacle(Triangle* triangle, const std::vector<Obstacle*>& obstacles)
{
	std::vector<TriEdge> overlappedEdges;
	Vec2 normals[3];
	normals[0] = (triangle->mEdgeList[0].mPoints[0] - triangle->mEdgeList[0].mPoints[1]).GetRotatedBy270().GetNormalised();
	normals[1] = (triangle->mEdgeList[1].mPoints[0] - triangle->mEdgeList[1].mPoints[1]).GetRotatedBy270().GetNormalised();
	normals[2] = (triangle->mEdgeList[2].mPoints[0] - triangle->mEdgeList[2].mPoints[1]).GetRotatedBy270().GetNormalised();

	Vec2 edgeMidPoints[3];
	edgeMidPoints[0] = ((triangle->mEdgeList[0].mPoints[0] * 1.22f) + triangle->mEdgeList[0].mPoints[1]) / 2.22f;
	edgeMidPoints[1] = ((triangle->mEdgeList[1].mPoints[0] * 1.22f) + triangle->mEdgeList[1].mPoints[1]) / 2.22f;
	edgeMidPoints[2] = ((triangle->mEdgeList[2].mPoints[0] * 1.22f) + triangle->mEdgeList[2].mPoints[1]) / 2.22f;

	int count = 0;

	for (int i = 0; i < 3; i++)
	{
		Vec2 point = edgeMidPoints[i] - (normals[i] * 0.01f);
		for (Obstacle* ob : obstacles)
		{
			std::vector<TriEdge> currentObEdges;
			Vec2 endPoint = edgeMidPoints[i] + normals[i] * 1000;

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
					overlappedEdges.push_back(e);
				}
			}

			if (count % 2 > 0)
			{
				return true;
			}
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

std::vector<Vec2> AddBufferToObstacles(const std::vector<Obstacle*>& obstacles)
{
	std::vector<Vec2> returnPoints;
	Vec2 next;
	Vec2 previous;
	for (int i = 0; i < obstacles.size(); i++)
	{
		std::vector<Vec2> points = obstacles[i]->GetPoints();
		for (int j = 0; j < points.size(); j++)
		{
			if (j == 0) { previous = points[points.size() - 1]; }
			else { previous = points[j - 1]; }

			if (j == points.size() - 1) { next = points[0]; }
			else { next = points[j + 1]; }

			Vec2 direction = (points[j] - next).Normalise();
			Vec2 direction2 = (points[j] - previous).Normalise();

			Vec2 final = (direction + direction2) / 2;

			returnPoints.push_back(points[j] + final * 1.0f);
		}
	}

	return returnPoints;
}

Vec2 GetTriangleCentre(Triangle*& triangle)
{
	// Average the points location
	float centerX = (triangle->mPoints[0].x + triangle->mPoints[1].x + triangle->mPoints[2].x) / 3;
	float centerY = (triangle->mPoints[0].y + triangle->mPoints[1].y + triangle->mPoints[2].y) / 3;

	return Vec2{ centerX, centerY };
}

std::vector<Vec2> FindTwoCommonVerts(Triangle*& tri1, Triangle*& tri2)
{
	int index = 0;

	for (int i = 0; i < 3; i++)
	{
		if (std::find(std::begin(tri2->mPoints), std::end(tri2->mPoints), tri1->mPoints[i]) == std::end(tri2->mPoints))
		{
			index = i;
		}
	}

	std::vector<Vec2> returnList;
	for (int i = 0; i < 3; i++)
	{
		if (i == index) { continue; }
		returnList.push_back(tri1->mPoints[i]);
	}
	return returnList;
}

bool DoLinesIntersect(Vec2 startPos, Vec2 endPos, const std::vector<Obstacle*>& obstacles)
{
	std::vector<std::vector<Vec2>> constraintEdgeList;
	for (Obstacle* ob : obstacles)
	{
		std::vector<Vec2> convexHull = ob->GetPoints();
		Vec2 next;
		for (size_t i = 0; i < convexHull.size(); i++)
		{
			if (i == convexHull.size() - 1) { next = convexHull[0]; }
			else (next = convexHull[i + 1]);

			std::vector<Vec2> tempList;
			tempList.push_back(convexHull[i]);
			tempList.push_back(next);
			constraintEdgeList.push_back(tempList);
		}
	}

	for (int i = 0; i < constraintEdgeList.size(); i++)
	{
		float oa = PseudoCross(endPos - startPos, constraintEdgeList[i][0] - endPos);
		float ob = PseudoCross(endPos - startPos, constraintEdgeList[i][1] - endPos);
		float oc = PseudoCross(constraintEdgeList[i][1] - constraintEdgeList[i][0], startPos - constraintEdgeList[i][0]);
		float od = PseudoCross(constraintEdgeList[i][1] - constraintEdgeList[i][0], endPos - constraintEdgeList[i][0]);

		if (oa * ob < 0 && oc * od < 0)
		{
			return true;
		}
	}

	return false;
}
