
#include "DelaunayTriangulation.h"
#include "Triangle.h"
#include "Utility.h"
#include "Grid.h"
#include "TriEdge.h"
#include "Vec2.h"
#include <random>
#include <algorithm>

std::vector<Triangle*> DelaunayTriangulate(std::vector<Vec2>& points, const std::vector<Obstacle*>& obstacles)
{
	for (Obstacle* ob : obstacles)
	{
		for (Vec2 v : ob->GetPoints())
		{
			// Dont add any points we already have in the list
			if (std::find(points.begin(), points.end(), v) == points.end())
			{
				points.push_back(v);
			}
		}
	}

	// List of triangles we are expanding when a new triangle is created, to check for the delaunay-ness of them
	std::vector<Triangle*> triangleStack;

	// Create super triangle
	Vec2 superTriangle[3];
	superTriangle[0] = (Vec2{ -1000,-1000 });
	superTriangle[1] = (Vec2{ 0,1000 });
	superTriangle[2] = (Vec2{ 1000,-1000 });
	triangleStack.push_back(CreateClockwiseTriangle(superTriangle));

	for (int i = 0; i < points.size(); i++)
	{
		int index = -1;
		// Find triangle that encloses the point we are checking
		for (int j = 0; j < triangleStack.size(); j++)
		{
			// Search the list of triangles to see if the point is within
			if (PointInTriangle(points[i], triangleStack[j]->mPoints[0], triangleStack[j]->mPoints[1], triangleStack[j]->mPoints[2]))
			{
				index = j;
				break;
			}
		}

		// Create new triangles with the added point to the vertices of the triangle which it falls into
		std::vector<Triangle*> potentialTriangles;
		potentialTriangles.push_back(CreateClockwiseTriangle(std::vector<Vec2> {points[i], triangleStack[index]->mPoints[0], triangleStack[index]->mPoints[1]}));
		potentialTriangles.push_back(CreateClockwiseTriangle(std::vector<Vec2> {points[i], triangleStack[index]->mPoints[1], triangleStack[index]->mPoints[2]}));
		potentialTriangles.push_back(CreateClockwiseTriangle(std::vector<Vec2> {points[i], triangleStack[index]->mPoints[2], triangleStack[index]->mPoints[0]}));

		// Remove the triangle we just replaced (the one that contained the point we are adding)
		triangleStack.erase(triangleStack.begin() + index);

		// Check if any newly created triangles have collinear points
		CollinearTriangleCheck triCheck = CheckListForCollinearPoints(potentialTriangles);

		// Add the valid triangles to the stack
		for (Triangle* tri : triCheck.returnTriangles)
		{
			//ConstructTriangleEdges(tri);
			triangleStack.push_back(tri);
		}

		// If collinear triangle is found, resolve them by creating 2 new triangles out of the collinear triangle and another triangle containing the other two vertices of the collinear triangle that isn't the newly added point
		if (!triCheck.collinearTriangles.empty())
		{
			std::vector<Triangle*> result = ResolveCollinearTriangles(triCheck.collinearTriangles, points[i], triangleStack);
			// Add the newly fixed triangles to the stack
			for (Triangle* tri : result)
			{
				if (!IsDuplicateTriangle(tri, triangleStack))
				{
					triangleStack.push_back(tri);
				}
			}
		}

		// If the stack is empty, then there is nothing left for us to check. Move on to adding the next point in the list
		if (triangleStack.empty()) { continue; }

		// Check if p lies inside its circumcircle
		for (int k = (int)triangleStack.size() - 1; k > 0; k--)
		{
			CheckCircumcircle(triangleStack[k], k, points[i], triangleStack);
		}
	}

	// After all points are added, remove all triangles connected with the super triangle vertices
	std::vector<Triangle*> returnList;
	for (int l = (int)triangleStack.size() - 1; l >= 0; l--)
	{
		if (std::find(std::begin(triangleStack[l]->mPoints), std::end(triangleStack[l]->mPoints), superTriangle[0]) == std::end(triangleStack[l]->mPoints))
		{
			if (std::find(std::begin(triangleStack[l]->mPoints), std::end(triangleStack[l]->mPoints), superTriangle[1]) == std::end(triangleStack[l]->mPoints))
			{
				if (std::find(std::begin(triangleStack[l]->mPoints), std::end(triangleStack[l]->mPoints), superTriangle[2]) == std::end(triangleStack[l]->mPoints))
				{
					returnList.push_back(triangleStack[l]);
				}
			}
		}
	}

	returnList = ConstrainedDelaunayTriangulation(returnList, obstacles);
	RemoveTrianglesFromObstacles(obstacles, returnList);
	RestoreDelauneyness(returnList, points);

	return returnList;
}

std::vector<Vec2> PoissonDisk(Vec2 startPoint, const std::vector<Obstacle*>& obstacles)
{
	std::vector<Vec2> openList;
	openList.push_back(startPoint);
	std::vector<Vec2> closedList;
	int attempts = 100;
	int minDist = 2;
	int maxDist = 5;
	bool isValid = true;

	while (!openList.empty())
	{
		for (int i = 0; i < attempts; i++)
		{
			int theta = rand() % 360;
			isValid = true;
			int range = maxDist - minDist + 1;
			Vec2 point = Vec2(((rand() % range + minDist) + openList[0].x) * cos(DegToRad(theta)), (rand() % range + minDist) + openList[0].y * sin(DegToRad(theta)));

			if (point.x < 0.5 || point.x > 18.5 || point.y < 0.5 || point.y > 14.5) { continue; }

			if (IsPointInObstacle(point, obstacles, 100)) { continue; }

			for (Vec2 v : openList)
			{
				if ((v - point).GetMagnitude() < minDist) { isValid = false; }
			}

			if (!isValid) { continue; }

			for (Vec2 v : closedList)
			{
				if ((v - point).GetMagnitude() < minDist) { isValid = false; }
			}

			if (isValid) { openList.push_back(point); }
		}

		closedList.push_back(openList[0]);
		openList.erase(openList.begin());
	}

	return closedList;
}

std::vector<Triangle*> ConstrainedDelaunayTriangulation(std::vector<Triangle*>& listOfTriangles, const std::vector<Obstacle*>& obstacles)
{
	// Construct edges for all the obstacles
	for (Obstacle* ob : obstacles)
	{
		std::vector<TriEdge> constraintEdgeList;
		for (TriEdge e : ConstructObstacleEdges(ob))
		{
			constraintEdgeList.push_back(e);
		}
		
		for (Triangle* t : listOfTriangles)
		{
			ConstructTriangleEdges(t);
		}
		HandleOverlapsWithObstacleEdges(constraintEdgeList, listOfTriangles, obstacles);
	}

	return listOfTriangles;
}

Triangle* CreateClockwiseTriangle(Vec2 points[])
{
	Vec2 ba = points[1] - points[0];
	Vec2 ca = points[2] - points[0];

	if (PseudoCross(ba, ca) > 0)
	{
		Vec2 temp = points[2];
		points[2] = points[1];
		points[1] = temp;
	}

	Triangle* triangle = new Triangle();

	triangle->mPoints[0] = points[0];
	triangle->mPoints[1] = points[1];
	triangle->mPoints[2] = points[2];

	return triangle;
}

Triangle* CreateClockwiseTriangle(std::vector<Vec2> points)
{
	Vec2 triangle[3];
	triangle[0] = points[0];
	triangle[1] = points[1];
	triangle[2] = points[2];

	return CreateClockwiseTriangle(triangle);
}

bool PointInTriangle(const Vec2& point, const Vec2& a, const Vec2& b, const Vec2& c)
{
	// Get vector for each line in triangle
	Vec2 ab = Vec2{ b.x - a.x, b.y - a.y };
	Vec2 bc = Vec2{ c.x - b.x, c.y - b.y };
	Vec2 ca = Vec2{ a.x - c.x, a.y - c.y };

	// Get vector for each vertex in triangle to point
	Vec2 ap = Vec2{ point.x - a.x, point.y - a.y };
	Vec2 bp = Vec2{ point.x - b.x, point.y - b.y };
	Vec2 cp = Vec2{ point.x - c.x, point.y - c.y };

	// Cross vectors to check
	float one = PseudoCross(ap, ab);
	float two = PseudoCross(bp, bc);
	float three = PseudoCross(cp, ca);

	// If all points return the same signed angle, then the point is inside the triangle
	if ((one <= 0 && two <= 0 && three <= 0) ||
		(one >= 0 && two >= 0 && three >= 0))
	{
		return true;
	}

	return false;
}

CollinearTriangleCheck CheckListForCollinearPoints(const std::vector<Triangle*> trianglesToCheck)
{
	float threshold = 0.0001f;

	CollinearTriangleCheck returnList;
	for (int i = 0; i < trianglesToCheck.size(); i++)
	{
		Vec2 ab = trianglesToCheck[i]->mPoints[1] - trianglesToCheck[i]->mPoints[0];
		Vec2 ac = trianglesToCheck[i]->mPoints[2] - trianglesToCheck[i]->mPoints[0];
		float result = PseudoCross(ab, ac);
		if (abs(result) >= threshold || abs(result) <= threshold)
		{
			returnList.returnTriangles.push_back(trianglesToCheck[i]);
		}
		else
		{
			returnList.collinearTriangles.push_back(trianglesToCheck[i]);
		}
	}

	return returnList;
}

std::vector<Triangle*> ResolveCollinearTriangles(const std::vector<Triangle*>& collinearTriangles, const Vec2& pointToFind, std::vector<Triangle*>& listToCheck)
{
	std::vector<Triangle*> returnTris;
	Triangle* newTri1 = new Triangle();
	Triangle* newTri2 = new Triangle();
	for (Triangle* colTri : collinearTriangles)
	{
		for (int i = 0; i < listToCheck.size(); i++)
		{
			if (std::find(std::begin(listToCheck[i]->mPoints), std::end(listToCheck[i]->mPoints), colTri->mPoints[0]) != std::end(listToCheck[i]->mPoints) && std::find(std::begin(listToCheck[i]->mPoints), std::end(listToCheck[i]->mPoints), colTri->mPoints[1]) != std::end(listToCheck[i]->mPoints)
				&& std::find(std::begin(listToCheck[i]->mPoints), std::end(listToCheck[i]->mPoints), pointToFind) == std::end(listToCheck[i]->mPoints))
			{
				newTri1->mPoints[0] = pointToFind;
				newTri1->mPoints[1] = colTri->mPoints[2];
				newTri1->mPoints[2] = colTri->mPoints[0];
				returnTris.push_back(CreateClockwiseTriangle(newTri1->mPoints));

				newTri2->mPoints[0] = pointToFind;
				newTri2->mPoints[1] = colTri->mPoints[2];
				newTri2->mPoints[2] = colTri->mPoints[1];
				returnTris.push_back(CreateClockwiseTriangle(newTri2->mPoints));

				listToCheck.erase(listToCheck.begin() + i);
				break;
			}

			if (std::find(std::begin(listToCheck[i]->mPoints), std::end(listToCheck[i]->mPoints), colTri->mPoints[0]) != std::end(listToCheck[i]->mPoints) && std::find(std::begin(listToCheck[i]->mPoints), std::end(listToCheck[i]->mPoints), colTri->mPoints[2]) != std::end(listToCheck[i]->mPoints)
				&& std::find(std::begin(listToCheck[i]->mPoints), std::end(listToCheck[i]->mPoints), pointToFind) == std::end(listToCheck[i]->mPoints))
			{
				newTri1->mPoints[0] = pointToFind;
				newTri1->mPoints[1] = listToCheck[i]->mPoints[1];
				newTri1->mPoints[2] = listToCheck[i]->mPoints[0];
				returnTris.push_back(CreateClockwiseTriangle(newTri1->mPoints));

				newTri2->mPoints[0] = pointToFind;
				newTri2->mPoints[1] = listToCheck[i]->mPoints[1];
				newTri2->mPoints[2] = listToCheck[i]->mPoints[2];
				returnTris.push_back(CreateClockwiseTriangle(newTri2->mPoints));

				listToCheck.erase(listToCheck.begin() + i);
				break;
			}

			if (std::find(std::begin(listToCheck[i]->mPoints), std::end(listToCheck[i]->mPoints), colTri->mPoints[1]) != std::end(listToCheck[i]->mPoints) && std::find(std::begin(listToCheck[i]->mPoints), std::end(listToCheck[i]->mPoints), colTri->mPoints[2]) != std::end(listToCheck[i]->mPoints)
				&& std::find(std::begin(listToCheck[i]->mPoints), std::end(listToCheck[i]->mPoints), pointToFind) == std::end(listToCheck[i]->mPoints))
			{
				newTri1->mPoints[0] = pointToFind;
				newTri1->mPoints[1] = listToCheck[i]->mPoints[1];
				newTri1->mPoints[2] = listToCheck[i]->mPoints[0];
				returnTris.push_back(CreateClockwiseTriangle(newTri1->mPoints));

				newTri2->mPoints[0] = pointToFind;
				newTri2->mPoints[1] = listToCheck[i]->mPoints[2];
				newTri2->mPoints[2] = listToCheck[i]->mPoints[0];
				returnTris.push_back(CreateClockwiseTriangle(newTri2->mPoints));

				listToCheck.erase(listToCheck.begin() + i);
				break;
			}
		}
	}
	return returnTris;
}

void ConstructTriangleEdges(Triangle* triangle)
{
	Vec2 next;

	for (size_t i = 0; i < 3; i++)
	{
		if (i == 3 - 1) { next = triangle->mPoints[0]; }
		else(next = triangle->mPoints[i + 1]);

		triangle->mEdgeList[i].mPoints[0] = triangle->mPoints[i];
		triangle->mEdgeList[i].mPoints[1] = next;
	}
}

void RestoreDelauneyness(std::vector<Triangle*>& listOfTriangles, std::vector<Vec2> points)
{
	int safety = 0;
	int successCount = 0;

	while (successCount < ((listOfTriangles.size()) * (points.size())))
	{
		safety++;
		successCount = 0;
		for (Vec2 p : points)
		{
			for (int i = 0; i < listOfTriangles.size(); i++)
			{
				if (CheckCircumcircle(listOfTriangles[i], i, p, listOfTriangles))
				{
					successCount++;
				}
			}
		}
		if (safety > 100) { return; }
	}
}

bool CheckCircumcircle(Triangle* triangle, int triangleIndex, const Vec2& pointToCheck, std::vector<Triangle*>& listToCheck)
{
	// Don't check any triangles that contain the point already
	if (std::find(std::begin(triangle->mPoints), std::end(triangle->mPoints), pointToCheck) != std::end(triangle->mPoints)) { return true; }

	// https://stackoverflow.com/questions/56224824/how-do-i-find-the-circumcenter-of-the-triangle-using-python-without-external-lib
	Vec2 a = triangle->mPoints[0];
	Vec2 b = triangle->mPoints[1];
	Vec2 c = triangle->mPoints[2];

	float d = 2 * ((a.x * (b.y - c.y)) + (b.x * (c.y - a.y)) + (c.x * (a.y - b.y)));
	float x = ((((a.x * a.x) + (a.y * a.y)) * (b.y - c.y)) + ((b.x * b.x + b.y * b.y) * (c.y - a.y)) + (((c.x * c.x) + (c.y * c.y)) * (a.y - b.y))) / d;
	float y = (((a.x * a.x + a.y * a.y) * (c.x - b.x)) + ((b.x * b.x + b.y * b.y) * (a.x - c.x)) + ((c.x * c.x + c.y * c.y) * (b.x - a.x))) / d;

	Vec2 circumcenter = Vec2{ x,y };

	// Get lengths of each side
	float ab = (triangle->mPoints[1] - triangle->mPoints[0]).GetMagnitude();
	float bc = (triangle->mPoints[2] - triangle->mPoints[1]).GetMagnitude();
	float ca = (triangle->mPoints[0] - triangle->mPoints[2]).GetMagnitude();

	// Triangles half perimeter
	float s = (ab + bc + ca) * 0.5f;

	// Find area using Heron's formula
	float area = sqrt((s * (s - ab) * (s - bc) * (s - ca)));

	// Find the radius of the circumcircle
	float radius = (ab * bc * ca) / (4 * area);

	float displacement = (pointToCheck - circumcenter).GetMagnitude();
	if (displacement <= radius)
	{
		SwapTriangles(triangle, triangleIndex, pointToCheck, listToCheck);
		return false;
	}
	return true;
}

void SwapTriangles(Triangle* triangleToSwap, int triangleIndex, const Vec2& point, std::vector<Triangle*>& listToCheck)
{
	TriangleAndIndex adjacentTriangle = FindAdjacentTriangle(listToCheck, triangleToSwap, point);

	// Move on if no adjacent triangle is found
	if (adjacentTriangle.index == -1)
	{
		return;
	}

	Vec2 connectPoint = Vec2();

	for (Vec2 p : triangleToSwap->mPoints)
	{
		if (std::find(std::begin(adjacentTriangle.triangle->mPoints), std::end(adjacentTriangle.triangle->mPoints), p) == std::end(adjacentTriangle.triangle->mPoints))
		{
			connectPoint = p;
			break;
		}
	}


	Vec2 adjPoints[2];
	int index = 0;
	// Remove the point we are checking so that we aren't including it twice when we recreate the triangles below
	for (size_t j = 0; j < 3; j++)
	{
		if (!Vector2IsEqual(adjacentTriangle.triangle->mPoints[j], point))
		{
			adjPoints[index] = adjacentTriangle.triangle->mPoints[j];
			index++;
		}
	}

	// Remove the triangles from the stack
	// Ensure we remove the higher value first as to not ruin the indices
	if (triangleIndex > adjacentTriangle.index)
	{
		listToCheck.erase(listToCheck.begin() + triangleIndex);
		listToCheck.erase(listToCheck.begin() + adjacentTriangle.index);
	}
	else
	{
		listToCheck.erase(listToCheck.begin() + adjacentTriangle.index);
		listToCheck.erase(listToCheck.begin() + triangleIndex);
	}

	// Add the new triangles to the stack, with the diagonal swapped
	Triangle* triangle1 = CreateClockwiseTriangle(std::vector<Vec2>{ point, connectPoint, adjPoints[0]});
	Triangle* triangle2 = CreateClockwiseTriangle(std::vector<Vec2>{ point, connectPoint, adjPoints[1]});

	if (!IsDuplicateTriangle(triangle1, listToCheck) && !IsTriangleCollinear(triangle1))
	{
		listToCheck.push_back(triangle1);
	}

	if (!IsDuplicateTriangle(triangle2, listToCheck) && !IsTriangleCollinear(triangle2))
	{
		listToCheck.push_back(triangle2);
	}
}

void SwapTrianglesWithKnownAdjacent(Triangle* tri1, int tri1Index, Triangle* tri2, int tri2Index, std::vector<Triangle*>& listToCheck, const TriEdge& edge)
{
	Vec2 point1;
	Vec2 point2;

	for (int i = 2; i >= 0; i--)
	{
		if (!Vector2IsEqual(tri1->mPoints[i], edge.mPoints[0]) && !Vector2IsEqual(tri1->mPoints[i], edge.mPoints[1]))
		{
			point1 = tri1->mPoints[i];
			break;
		}
	}

	for (int i = 2; i >= 0; i--)
	{
		if (!Vector2IsEqual(tri2->mPoints[i], edge.mPoints[0]) && !Vector2IsEqual(tri2->mPoints[i], edge.mPoints[1]))
		{
			point2 = tri2->mPoints[i];
			break;
		}
	}

	// Remove the triangles from the stack
	// Ensure we remove the higher value first as to not ruin the indices
	if (tri1Index > tri2Index)
	{

		listToCheck.erase(listToCheck.begin() + tri1Index);
		listToCheck.erase(listToCheck.begin() + tri2Index);
	}
	else
	{
		listToCheck.erase(listToCheck.begin() + tri2Index);
		listToCheck.erase(listToCheck.begin() + tri1Index);
	}

	// Add the new triangles to the stack, with the diagonal swapped
	Triangle* triangle1 = CreateClockwiseTriangle(std::vector<Vec2>{ edge.mPoints[0], point1, point2});
	Triangle* triangle2 = CreateClockwiseTriangle(std::vector<Vec2>{ edge.mPoints[1], point1, point2});

	if (!IsDuplicateTriangle(triangle1, listToCheck) && !IsTriangleCollinear(triangle1))
	{
		listToCheck.push_back(triangle1);
	}

	if (!IsDuplicateTriangle(triangle2, listToCheck) && !IsTriangleCollinear(triangle2))
	{
		listToCheck.push_back(triangle2);
	}
}

TriangleAndIndex FindAdjacentTriangle(const std::vector<Triangle*>& triangleStack, Triangle* triangle, const Vec2& pointToCheck)
{
	for (int i = 0; i < triangleStack.size(); i++)
	{
		// Skip if the triangle we are checking is the same as the triangle we want to swap
		if (IsSameTriangle(triangle, triangleStack[i])) { continue; }

		// Swaps with concave quadrilaterals do not work
		if (!IsConvexQuadrilateral(triangleStack[i], triangle)) { continue; }

		// Make sure we are returning the triangle that contains the point
		if (std::find(std::begin(triangleStack[i]->mPoints), std::end(triangleStack[i]->mPoints), pointToCheck) != std::end(triangleStack[i]->mPoints))
		{
			// Find the triangle that contains the two common points shared by the adjacent triangle and the point we are checking
			if (std::find(std::begin(triangleStack[i]->mPoints), std::end(triangleStack[i]->mPoints), triangle->mPoints[0]) != std::end(triangleStack[i]->mPoints) && std::find(std::begin(triangleStack[i]->mPoints), std::end(triangleStack[i]->mPoints), triangle->mPoints[1]) != std::end(triangleStack[i]->mPoints) ||
				std::find(std::begin(triangleStack[i]->mPoints), std::end(triangleStack[i]->mPoints), triangle->mPoints[0]) != std::end(triangleStack[i]->mPoints) && std::find(std::begin(triangleStack[i]->mPoints), std::end(triangleStack[i]->mPoints), triangle->mPoints[2]) != std::end(triangleStack[i]->mPoints) ||
				std::find(std::begin(triangleStack[i]->mPoints), std::end(triangleStack[i]->mPoints), triangle->mPoints[1]) != std::end(triangleStack[i]->mPoints) && std::find(std::begin(triangleStack[i]->mPoints), std::end(triangleStack[i]->mPoints), triangle->mPoints[2]) != std::end(triangleStack[i]->mPoints))
			{
				// Return the found triangle
				return TriangleAndIndex{ i, triangleStack[i] };
			}
		}
	}

	// Return empty struct if nothing found
	return TriangleAndIndex{ -1, nullptr };
}

bool IsTriangleCollinear(const Triangle* triangleToCheck)
{
	Vec2 ab = triangleToCheck->mPoints[1] - triangleToCheck->mPoints[0];
	Vec2 ac = triangleToCheck->mPoints[2] - triangleToCheck->mPoints[0];

	return PseudoCross(ab, ac) == 0;
}

bool IsDuplicateTriangle(const Triangle* triangleToCheck, const std::vector<Triangle*>& listToCheck)
{
	for (Triangle* triangle : listToCheck)
	{
		if (std::find(std::begin(triangle->mPoints), std::end(triangle->mPoints), triangleToCheck->mPoints[0]) != std::end(triangle->mPoints) &&
			std::find(std::begin(triangle->mPoints), std::end(triangle->mPoints), triangleToCheck->mPoints[1]) != std::end(triangle->mPoints) &&
			std::find(std::begin(triangle->mPoints), std::end(triangle->mPoints), triangleToCheck->mPoints[2]) != std::end(triangle->mPoints))
		{
			return true;
		}
	}

	return false;
}

bool IsSameTriangle(const Triangle* tri1, const Triangle* tri2)
{
	if (std::find(std::begin(tri1->mPoints), std::end(tri1->mPoints), tri2->mPoints[0]) != std::end(tri1->mPoints) &&
		std::find(std::begin(tri1->mPoints), std::end(tri1->mPoints), tri2->mPoints[1]) != std::end(tri1->mPoints) &&
		std::find(std::begin(tri1->mPoints), std::end(tri1->mPoints), tri2->mPoints[2]) != std::end(tri1->mPoints))
	{
		return true;
	}

	return false;
}

bool IsConvexQuadrilateral(const Triangle* tri1, const Triangle* tri2)
{
	Vec2 quad[4];
	for (int i = 0; i < 3; i++)
	{
		if (std::find(std::begin(tri2->mPoints), std::end(tri2->mPoints), tri1->mPoints[i]) == std::end(tri2->mPoints))
		{
			quad[0] = tri1->mPoints[i];
			break;
		}
	}

	quad[1] = tri2->mPoints[0];
	quad[2] = tri2->mPoints[1];
	quad[3] = tri2->mPoints[2];

	Vec2 start = quad[0];
	for (Vec2 p : quad)
	{
		// Assigns start to the point with the lowest y-value if their x-value is identical
		if (p.x == start.x)
		{
			if (p.y < start.y) { start = p; }
			continue;
		}

		if (p.x < start.x) { start = p; }
	}

	Vec2 next;
	Vec2 previous = start;
	std::vector<Vec2> list;
	list.push_back(start);

	for (int i = 0; i < 3; i++)
	{
		next = quad[0];
		if (Vector2IsEqual(next, previous)) { next = quad[1]; }

		for (int j = 0; j < 4; j++)
		{
			// We don't want to check the point we are already using
			if (Vector2IsEqual(quad[j], next)) { continue; }
			Vec2 ab = quad[j] - next;
			Vec2 ac = next - previous;

			float result = PseudoCross(ab, ac);

			// If the result is positive, then the point falls outside the line, meaning 'next' will fall inside the convex hull
			if (result > 0) { next = quad[j]; }
		}
		list.push_back(next);
		previous = next;
	}

	float result1 = FindAngle(list[0], list[1], list[3]);
	float result2 = FindAngle(list[1], list[2], list[0]);
	float result3 = FindAngle(list[2], list[3], list[1]);
	float result4 = FindAngle(list[3], list[0], list[2]);

	if (result1 < 180 && result2 < 180 && result3 < 180 && result4 < 180)
	{
		return true;
	}

	return false;
}

void AssignTrianglesToEdges(Triangle* triangle, std::vector<Triangle*>& listOfTriangles)
{
	for (TriEdge& edge : triangle->mEdgeList)
	{
		edge.mTriangles.push_back(triangle);

		for (int i = 0; i < listOfTriangles.size(); i++)
		{
			Triangle* t = listOfTriangles[i];
			if (IsSameTriangle(t, triangle)) { continue; }

			if (std::find(std::begin(t->mPoints), std::end(t->mPoints), edge.mPoints[0]) != std::end(t->mPoints) && std::find(std::begin(t->mPoints), std::end(t->mPoints), edge.mPoints[1]) != std::end(t->mPoints) ||
				(std::find(std::begin(t->mPoints), std::end(t->mPoints), edge.mPoints[0]) != std::end(t->mPoints) && std::find(std::begin(t->mPoints), std::end(t->mPoints), edge.mPoints[2]) != std::end(t->mPoints)) ||
				(std::find(std::begin(t->mPoints), std::end(t->mPoints), edge.mPoints[2]) != std::end(t->mPoints) && std::find(std::begin(t->mPoints), std::end(t->mPoints), edge.mPoints[0]) != std::end(t->mPoints)))
			{
				edge.mTriangles.push_back(t);
				edge.mTriIndex.push_back(i);
				return;
			}
		}
	}
}

void AssignTrianglesToEdges(TriEdge& edge, std::vector<Triangle*>& listOfTriangles)
{
	edge.mTriangles.clear();

	for (int i = 0; i < listOfTriangles.size(); i++)
	{
		Triangle* t = listOfTriangles[i];
		if (std::find(std::begin(edge.mPoints), std::end(edge.mPoints), t->mPoints[0]) != std::end(edge.mPoints) && std::find(std::begin(edge.mPoints), std::end(edge.mPoints), t->mPoints[1]) != std::end(edge.mPoints) ||
			std::find(std::begin(edge.mPoints), std::end(edge.mPoints), t->mPoints[0]) != std::end(edge.mPoints) && std::find(std::begin(edge.mPoints), std::end(edge.mPoints), t->mPoints[2]) != std::end(edge.mPoints) ||
			std::find(std::begin(edge.mPoints), std::end(edge.mPoints), t->mPoints[1]) != std::end(edge.mPoints) && std::find(std::begin(edge.mPoints), std::end(edge.mPoints), t->mPoints[2]) != std::end(edge.mPoints))
		{
			edge.mTriangles.push_back(t);
			edge.mTriIndex.push_back(i);

			if (edge.mTriangles.size() == 2)
			{
				break;
			}
		}
	}
}

std::vector<TriEdge> FindOverlapsWithConstraints(std::vector<TriEdge> constraintEdges, std::vector<Triangle*>& listOfTriangles)
{
	std::vector<TriEdge> overlappingEdges;

	// Add each edge to the triangulation 
	for (size_t i = 0; i < constraintEdges.size(); i++)
	{
		for (size_t j = 0; j < listOfTriangles.size(); j++)
		{
			// Check if the points overlap with the edge
			// Get the vector of the constraint edge
			for (TriEdge tEdge : listOfTriangles[j]->mEdgeList)
			{
				Vec2 edgePoint1 = tEdge.mPoints[0];
				Vec2 edgePoint2 = tEdge.mPoints[1];

				float result1 = PseudoCross((edgePoint2 - edgePoint1), (constraintEdges[i].mPoints[0] - edgePoint1));
				float result2 = PseudoCross((edgePoint2 - edgePoint1), (constraintEdges[i].mPoints[1] - edgePoint1));

				float result3 = PseudoCross((constraintEdges[i].mPoints[1] - constraintEdges[i].mPoints[0]), (edgePoint1 - constraintEdges[i].mPoints[0]));
				float result4 = PseudoCross((constraintEdges[i].mPoints[1] - constraintEdges[i].mPoints[0]), (edgePoint2 - constraintEdges[i].mPoints[0]));

				if (result1 * result2 < 0 && result3 * result4 < 0)
				{
					overlappingEdges.push_back(tEdge);
				}
			}
		}
	}

	RemoveDuplicateEdges(overlappingEdges);

	return overlappingEdges;
}

std::vector<TriEdge> RemoveDuplicateEdges(std::vector<TriEdge>& edges)
{
	std::vector<int> indices;

	for (int i = (int)edges.size() - 1; i >= 0; i--)
	{
		if (i >= (int)edges.size()) { i = (int)edges.size() - 1; }

		TriEdge edge1 = edges[i];

		for (int j = i - 1; j >= 0; j--)
		{
			TriEdge edge2 = edges[j];

			if ((edge1.mPoints[0] == edge2.mPoints[0] || edge1.mPoints[1] == edge2.mPoints[0]) &&
				(edge1.mPoints[0] == edge2.mPoints[1] || edge1.mPoints[1] == edge2.mPoints[1]))
			{
				edges.erase(edges.begin() + j);
			}
		}
	}
	return edges;
}

void HandleOverlapsWithConstraints(std::vector<TriEdge>& edges, std::vector<TriEdge>& constraints, std::vector<Triangle*>& listOfTriangles)
{
	for (size_t i = 0; i < edges.size(); i++)
	{
		std::vector<int> results = FindSharedEdgeTriangles(edges[i], listOfTriangles);
		if (results.empty() || results.size() == 1) { continue; }

		SwapTrianglesWithKnownAdjacent(listOfTriangles[results[0]], results[0], listOfTriangles[results[1]], results[1], listOfTriangles, edges[i]);
	}
}

void SwapEdge(TriEdge& edge, std::vector<Triangle*>& listOfTriangles)
{
	Triangle* tri1 = listOfTriangles[edge.mTriIndex[0]];
	Triangle* tri2 = listOfTriangles[edge.mTriIndex[1]];

	Vec2 connectPoint1;
	Vec2 connectPoint2;

	for (Vec2 v : tri1->mPoints)
	{
		if ((v != edge.mPoints[0]) && (v != edge.mPoints[1]))
		{
			connectPoint1 = v;
		}
	}

	for (Vec2 v : tri2->mPoints)
	{
		if ((v != edge.mPoints[0]) && (v != edge.mPoints[1]))
		{
			connectPoint2 = v;
		}
	}

	if (edge.mTriIndex[0] > edge.mTriIndex[1])
	{
		listOfTriangles.erase(listOfTriangles.begin() + edge.mTriIndex[0]);
		listOfTriangles.erase(listOfTriangles.begin() + edge.mTriIndex[1]);
	}
	else
	{
		listOfTriangles.erase(listOfTriangles.begin() + edge.mTriIndex[1]);
		listOfTriangles.erase(listOfTriangles.begin() + edge.mTriIndex[0]);
	}

	std::vector<Triangle*> potentialTris;
	potentialTris.push_back(CreateClockwiseTriangle(std::vector<Vec2>{edge.mPoints[0], connectPoint1, connectPoint2}));
	potentialTris.push_back(CreateClockwiseTriangle(std::vector<Vec2>{edge.mPoints[1], connectPoint1, connectPoint2}));

	if (!IsDuplicateTriangle(potentialTris[0], listOfTriangles) && IsTriangleCollinear(potentialTris[0]))
	{
		listOfTriangles.push_back(potentialTris[0]);
	}

	if (!IsDuplicateTriangle(potentialTris[1], listOfTriangles) && IsTriangleCollinear(potentialTris[1]))
	{
		listOfTriangles.push_back(potentialTris[1]);
	}
}

void HandleOverlappingEdges(std::vector<Triangle*>& listOfTriangles)
{
	if (listOfTriangles.size() == 0) { return; }

	for (size_t i = 0; i < listOfTriangles.size() - 1; i++)
	{
		// Construct edges for the triangle
		for (size_t j = i + 1; j <= listOfTriangles.size() - 1; j++)
		{
			// Check if the points overlap with the edge
			// Get the vector of the constraint edge
			for (TriEdge vEdge : listOfTriangles[i]->mEdgeList)
			{
				for (TriEdge vEdge2 : listOfTriangles[j]->mEdgeList)
				{
					float result1 = PseudoCross((vEdge.mPoints[1] - vEdge.mPoints[0]), (vEdge2.mPoints[0] - vEdge.mPoints[0]));
					float result2 = PseudoCross((vEdge.mPoints[1] - vEdge.mPoints[0]), (vEdge2.mPoints[1] - vEdge.mPoints[0]));

					float result3 = PseudoCross((vEdge2.mPoints[1] - vEdge2.mPoints[0]), (vEdge.mPoints[0] - vEdge2.mPoints[0]));
					float result4 = PseudoCross((vEdge2.mPoints[1] - vEdge2.mPoints[0]), (vEdge.mPoints[1] - vEdge2.mPoints[0]));

					bool result = (result1 * result2 < 0 && result3 * result4 < 0);
					if (result)
					{
						Vec2 point{};
						for (size_t a = 0; a < 3; a++)
						{
							if (std::find(std::begin(listOfTriangles[i]->mPoints), std::end(listOfTriangles[i]->mPoints), listOfTriangles[j]->mPoints[a]) == std::end(listOfTriangles[i]->mPoints))
							{
								point = listOfTriangles[j]->mPoints[a];
							}
						}

						Triangle* newTriangle = CreateClockwiseTriangle(std::vector<Vec2>{vEdge.mPoints[0], vEdge.mPoints[1], point});

						if (!IsDuplicateTriangle(newTriangle, listOfTriangles) && !IsTriangleCollinear(newTriangle))
						{
							listOfTriangles.push_back(newTriangle);
						}

						listOfTriangles.erase(listOfTriangles.begin() + j);
					}
				}
			}
		}
	}
}

void RemoveTrianglesFromObstacles(std::vector<Obstacle*> obstacles, std::vector<Triangle*>& listOfTriangles)
{
	std::vector<int> triIndexRemoval;

	for (int i = 0; i < listOfTriangles.size(); i++)
	{
		if (IsTriangleInObstacle(listOfTriangles[i], obstacles))
		{
			triIndexRemoval.push_back(i);
		}
		else if (IsTriangleCollinear(listOfTriangles[i]))
		{
			triIndexRemoval.push_back(i);
		}
	}

	for (int i = (int)triIndexRemoval.size() - 1; i >= 0; i--)
	{
		listOfTriangles.erase(listOfTriangles.begin() + triIndexRemoval[i]);
	}
}

std::vector<int> FindSharedEdgeTriangles(const TriEdge& edge, const std::vector<Triangle*>& listOfTriangles)
{
	std::vector<int> returnInts;

	for (int i = 0; i < (int)listOfTriangles.size(); i++)
	{
		if (std::find(std::begin(listOfTriangles[i]->mPoints), std::end(listOfTriangles[i]->mPoints), edge.mPoints[0]) != std::end(listOfTriangles[i]->mPoints) &&
			std::find(std::begin(listOfTriangles[i]->mPoints), std::end(listOfTriangles[i]->mPoints), edge.mPoints[1]) != std::end(listOfTriangles[i]->mPoints))
		{
			returnInts.push_back(i);
		}
	}

	return returnInts;
}

void HandleOverlapsWithObstacleEdges(std::vector<TriEdge> constraintEdges, std::vector<Triangle*>& listOfTriangles, const std::vector<Obstacle*>& obstacles)
{
	// Add each edge to the triangulation 
	for (size_t i = 0; i < constraintEdges.size(); i++)
	{
		std::vector<Triangle*> overlappingTriangles;
		for (size_t j = 0; j < listOfTriangles.size(); j++)
		{
			// Check if the points overlap with the edge
			// Get the vector of the constraint edge
			for (TriEdge tEdge : listOfTriangles[j]->mEdgeList)
			{
				Vec2 edgePoint1 = tEdge.mPoints[0];
				Vec2 edgePoint2 = tEdge.mPoints[1];


				if (edgePoint1 == constraintEdges[i].mPoints[0] || edgePoint1 == constraintEdges[i].mPoints[1])
				{
					if (edgePoint2 == constraintEdges[i].mPoints[0] || edgePoint1 == constraintEdges[i].mPoints[1])
					{
						continue;
					}
				}

				float result1 = PseudoCross((edgePoint2 - edgePoint1), (constraintEdges[i].mPoints[0] - edgePoint1));
				float result2 = PseudoCross((edgePoint2 - edgePoint1), (constraintEdges[i].mPoints[1] - edgePoint1));

				float result3 = PseudoCross((constraintEdges[i].mPoints[1] - constraintEdges[i].mPoints[0]), (edgePoint1 - constraintEdges[i].mPoints[0]));
				float result4 = PseudoCross((constraintEdges[i].mPoints[1] - constraintEdges[i].mPoints[0]), (edgePoint2 - constraintEdges[i].mPoints[0]));

				if (result1 * result2 <= 0 && result3 * result4 <= 0)
				{
					if (Vector2IsEqual(edgePoint1, constraintEdges[i].mPoints[0]) || Vector2IsEqual(edgePoint1, constraintEdges[i].mPoints[1]) ||
						Vector2IsEqual(edgePoint2, constraintEdges[i].mPoints[0]) || Vector2IsEqual(edgePoint2, constraintEdges[i].mPoints[1]))
					{
						continue;
					}

					// Above results will be true for collinear lines, below cross products check to see if they are not collinear
					float alt1 = PseudoCross(edgePoint1 - constraintEdges[i].mPoints[0], edgePoint2 - constraintEdges[i].mPoints[0]);
					float alt2 = PseudoCross(edgePoint1 - constraintEdges[i].mPoints[1], edgePoint2 - constraintEdges[i].mPoints[1]);

					// If both results are 0 then the line is collinear and not crossing
					if (alt1 == 0 && alt2 == 0) { continue; }

					if (!IsDuplicateTriangle(listOfTriangles[j], overlappingTriangles))
					{
						overlappingTriangles.push_back(listOfTriangles[j]);
						continue;
					}
				}
			}
		}

		if (overlappingTriangles.empty()) { continue; }
		
		std::vector<Vec2> a;
		std::vector<Vec2> b;
		Vec2 normal = (constraintEdges[i].mPoints[1] - constraintEdges[i].mPoints[0]).GetRotatedBy270().GetNormalised();
		Vec2 midPoint = (constraintEdges[i].mPoints[1] + constraintEdges[i].mPoints[0]) * 0.5f;
		
		for (Vec2 v : constraintEdges[i].mPoints)
		{
			a.push_back(v);
			b.push_back(v);
		}
		
		for (Triangle* t : overlappingTriangles)
		{
			for (Vec2 point : t->mPoints)
			{
				if (point == constraintEdges[i].mPoints[0] || point == constraintEdges[i].mPoints[1]) { continue; }
		
				float result = Dot(normal, (midPoint - point).Normalise());
				if (result > 0 && std::find(a.begin(), a.end(), point) == a.end())
				{
					a.push_back(point);
				}
		
				else if (result < 0 && std::find(b.begin(), b.end(), point) == b.end())
				{
					b.push_back(point);
				}
			}
		
			auto it = std::find(listOfTriangles.begin(), listOfTriangles.end(), t);
			if (it != listOfTriangles.end())
			{
				listOfTriangles.erase(it);
			}
		}
		
		std::vector<Obstacle*> obs;
		for (Triangle* t : DelaunayTriangulate(a, obs))
		{
			listOfTriangles.push_back(t);
		}
		
		for (Triangle* t : DelaunayTriangulate(b, obs))
		{
			listOfTriangles.push_back(t);
		}
	}
}

Triangle* FindAdjacentTriangleToEdge(int currentTriangleIndex, const std::vector<Vec2>& edge, const std::vector<Triangle*>& triangleList)
{
	for (int i = 0; i < triangleList.size(); i++)
	{
		if (i == currentTriangleIndex) { continue; }

		if (std::find(std::begin(triangleList[i]->mPoints), std::end(triangleList[i]->mPoints), edge[0]) != std::end(triangleList[i]->mPoints) && std::find(std::begin(triangleList[i]->mPoints), std::end(triangleList[i]->mPoints), edge[1]) != std::end(triangleList[i]->mPoints))
		{
			return triangleList[i];
		}
	}

	return nullptr;
}
