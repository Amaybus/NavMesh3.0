#include "NavigationMesh.h"
#include "Triangle.h"
#include "DelaunayTriangulation.h"
#include "Obstacle.h"
#include "TextStream.h"

NavigationMesh::NavigationMesh()
{
}

void NavigationMesh::Draw(LineRenderer* lines)
{
	for (int i = 0; i < mPoints.size(); i++)
	{
		lines->DrawCircle(mPoints[i], 100, Colour::GREEN);
		//lines->AddPointToLine(mPoints[i], Colour::BLUE);
	}
	//lines->FinishLineLoop();
	int index = 0;
	for (Triangle* t : mTriangles)
	{
		lines->AddPointToLine(t->mPoints[0], Colour::WHITE);
		lines->AddPointToLine(t->mPoints[1], Colour::WHITE);
		lines->AddPointToLine(t->mPoints[2], Colour::WHITE);
		lines->FinishLineLoop();

		Vec2 result = t->mPoints[0] + t->mPoints[1] + t->mPoints[2];
		result /= 3;
		TextStream output(lines, Vec2(result.x, result.y), 100.0f, Colour::YELLOW);
		output << index;
		index++;
	}

	//for (Vec2 v : mPoints)
	//{
	//	TextStream output(lines, Vec2(v.x, v.y), 0.3f, Colour::RED);
	//	output << v.x << ", " << v.y;
	//}
}

void NavigationMesh::Build(std::vector<Obstacle*>& obstacles)
{
	mTriangles = DelaunayTriangulate(mPoints, obstacles);
}

void NavigationMesh::AddPointList(std::vector<Vec2> pointList)
{
	for (Vec2 p : pointList)
	{
		AddPoint(p);
	}
}
