#include "NavigationMesh.h"
#include "Triangle.h"

NavigationMesh::NavigationMesh()
{
}

void NavigationMesh::Draw(LineRenderer* lines)
{
	for (int i = 0; i < mPoints.size(); i++)
	{
		lines->DrawCircle(mPoints[i], 100, Colour::GREEN);
		lines->AddPointToLine(mPoints[i], Colour::BLUE);
	}
	lines->FinishLineLoop();
	for (Triangle t : mTriangles)
	{
		lines->AddPointToLine(t.mPoints[0], Colour::WHITE);
		lines->AddPointToLine(t.mPoints[1], Colour::WHITE);
		lines->AddPointToLine(t.mPoints[2], Colour::WHITE);
		lines->FinishLineLoop();
	}
}

void NavigationMesh::Build()
{
	// Trianglulate
	// CDT
}

void NavigationMesh::AddPointList(std::vector<Vec2> pointList)
{
	for (Vec2 p : pointList)
	{
		AddPoint(p);
	}
}
