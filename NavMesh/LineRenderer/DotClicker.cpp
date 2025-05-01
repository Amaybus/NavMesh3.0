#include "DotClicker.h"
#include "LineRenderer.h"
#include "imgui.h"


DotClicker::DotClicker()
{
	appInfo.appName = "Dot clicker";
}

void DotClicker::Initialise()
{

}

void DotClicker::Update(float delta)
{

	if (currentlyGrabbedPoint != -1)
	{
		points[currentlyGrabbedPoint] = cursorPos;
	}
	lines->SetColour({ 1, 1, 1 });
	for (Vec2 point : points)
	{
		lines->DrawCircle(point, pointRadius);
	}

}
void DotClicker::OnLeftClick()
{
	//Try to grab vert here, or place a new one if there isn't one.
	int thisClickIndex = GetPointAtPosition(cursorPos);
	if (thisClickIndex == -1)
	{
		points.push_back(cursorPos);
	}
	else
	{
		currentlyGrabbedPoint = thisClickIndex;
	}
}
void DotClicker::OnRightClick()
{
	//Try to delete vert here.
	int thisClickIndex = GetPointAtPosition(cursorPos);
	if (thisClickIndex != -1)
	{
		currentlyGrabbedPoint = -1;	//Ungrab anything.
		points.erase(points.begin() + thisClickIndex);
	}
}

void DotClicker::OnLeftRelease()
{
	currentlyGrabbedPoint = -1;
}

int DotClicker::GetPointAtPosition(Vec2 position)
{
	for (int i = 0; i < points.size(); i++)
	{
		if ((points[i] - position).GetMagnitudeSquared() < pointRadius * pointRadius)
		{
			return i;
		}
	}
	return -1;
}