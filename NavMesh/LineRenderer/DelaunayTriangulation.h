#pragma once

#include "Vec2.h"
#include <vector>
#include "Triangle.h"
#include "Obstacle.h"

class Grid;



struct CollinearTriangleCheck
{
	std::vector<Triangle*> returnTriangles;
	std::vector<Triangle*> collinearTriangles;
};

struct TriangleAndIndex
{
	int index;
	Triangle* triangle;
};


// If we have a random set of points
std::vector<Triangle*> DelaunayTriangulate(std::vector<Vec2>& points, const std::vector<Obstacle*>& obstacles);

// Poisson Disk
std::vector<Vec2> PoissonDisk(Vec2 startPoint, const std::vector<Obstacle*>& obstacles);

// If we want constraints
std::vector<Triangle*> ConstrainedDelaunayTriangulation(std::vector<Triangle*>& listOfTriangles, const std::vector<Obstacle*>& obstacles);

void AssignEdgesAndAdjTris(std::vector<Triangle*>& listOfTriangles);

// To ensure all triangles are in the correct winding order
Triangle* CreateClockwiseTriangle(Vec2 points[]);
Triangle* CreateClockwiseTriangle(std::vector<Vec2> points);

// Does the point fall inside the triangle
bool PointInTriangle(const Vec2& point, const Vec2& a, const Vec2& b, const Vec2& c);

// Checks a list of points to see if they fall on the same line
CollinearTriangleCheck CheckListForCollinearPoints(const std::vector<Triangle*> trianglesToCheck);

// Stops triangles from being added that are a straight line
std::vector<Triangle*> ResolveCollinearTriangles(const std::vector<Triangle*>& collinearTriangles, const Vec2& pointToFind, std::vector<Triangle*>& listToCheck);

// Creates and assigns the edges of a triangle
void ConstructTriangleEdges(Triangle* triangle);

void RestoreDelauneyness(std::vector<Triangle*>& listOfTriangles, std::vector<Vec2> points);

// Test if points lie in the circumcircle of a triangle
bool CheckCircumcircle(Triangle* triangle, int triangleIndex, const Vec2& pointToCheck, std::vector<Triangle*>& listToCheck);

// Used for when a point falls within the circumcircle of a triangle
void SwapTriangles(Triangle* triangleToSwap, int triangleIndex, const Vec2& point, std::vector<Triangle*>& listToCheck);
void SwapTrianglesWithKnownAdjacent(Triangle* tri1, int tri1Index, Triangle* tri2, int tri2Index, std::vector<Triangle*>& listToCheck, const TriEdge& edge);

// Gives us the triangle which contains two of the same points
TriangleAndIndex FindAdjacentTriangle(const std::vector<Triangle*>& triangleStack, Triangle* triangle, const Vec2& pointToCheck);

// Checks to see if all points of the triangle fall on the same line
bool IsTriangleCollinear(const Triangle* triangleToCheck);

// Used for when we are adding a potential triangle to our list, to ensure we aren't doubling up
bool IsDuplicateTriangle(const Triangle* triangleToCheck, const std::vector<Triangle*>& listToCheck);

// Check if two triangles are the same
bool IsSameTriangle(const Triangle* tri1, const Triangle* tri2);

// To ensure we are swapping the diagonals of the right triangles
bool IsConvexQuadrilateral(const Triangle* tri1, const Triangle* tri2);

// Assigns the adjacent triangles to this triangle
void ConstructAdjacentTriangles(Triangle* triangle, const std::vector<Triangle*>& listOfTriangles);

// Assigns the triangles which share the same edge
void AssignTrianglesToEdges(Triangle* triangle, std::vector<Triangle*>& listOfTriangles);
void AssignTrianglesToEdges(TriEdge& edge, std::vector<Triangle*>& listOfTriangles);

// Finds all triangle edges that overlap with a constraint edge
std::vector<TriEdge> FindOverlapsWithConstraints(std::vector<TriEdge> constraintEdges, std::vector<Triangle*>& listOfTriangles);

// Removes any edges that have been added to the list twice
std::vector<TriEdge> RemoveDuplicateEdges(std::vector<TriEdge>& edges);

// Removes all overlapping edges
void HandleOverlapsWithConstraints(std::vector<TriEdge>& edges, std::vector<TriEdge>& constraints, std::vector<Triangle*>& listOfTriangles);

// Swap adjacent triangles 
void SwapEdge(TriEdge& edge, std::vector<Triangle*>& listOfTriangles);

// Cycles through the overlapping edge list and will swap triangles that share the same edge
void HandleOverlappingEdges(std::vector<Triangle*>& listOfTriangles);

// Removes any triangles that are inside of the obstacles
void RemoveTrianglesFromObstacles(std::vector<Obstacle*> obstacles, std::vector<Triangle*>& listOfTriangles);

std::vector<int> FindSharedEdgeTriangles(const TriEdge& edge, const std::vector<Triangle*>& listOfTriangles);

void HandleOverlapsWithObstacleEdges(std::vector<TriEdge>, std::vector<Triangle*>& listOfTriangles, const std::vector<Obstacle*>& obstacles);

Triangle* FindAdjacentTriangleToEdge(int currentTriangleIndex, const std::vector<Vec2>& edge, const std::vector<Triangle*>& triangleList);