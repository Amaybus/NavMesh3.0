#include "NavigationMesh.h"
#include "Triangle.h"
#include "DelauneyTriangulation.h"
#include "Grid.h"
#include "TextStream.h"

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

	for (Vec2 v : mPoints)
	{
		TextStream output(lines, Vec2(v.x + 10, v.y + 10), 10.0f, Colour::RED);
		output << v.x << ", " << v.y;
	}
}

void NavigationMesh::Build(Grid& grid)
{
	mTriangles = DelauneyTriangulate(mPoints, grid);
}

void NavigationMesh::AddPointList(std::vector<Vec2> pointList)
{
	for (Vec2 p : pointList)
	{
		AddPoint(p);
	}
}
