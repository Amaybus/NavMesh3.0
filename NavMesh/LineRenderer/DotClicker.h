#pragma once

#include "Maths.h"
#include "Application.h"
#include <vector>

//A simple project that allows the user to place points by clicking, drag them around,
//and remove them by right-clicking on them. The order of the dots is stable but there's
//no built-in way to mess with the order.

//Can serve as a base for demonstrating other algorithms, like gift-wrapping or ear-pruning.

class DotClicker : public Application
{
	std::vector<Vec2> points;
	int currentlyGrabbedPoint = -1;
	float pointRadius = 0.1f;

public:
	DotClicker();
	void Initialise() override;

	virtual void Update(float delta) override;

	void OnLeftClick() override;
	void OnLeftRelease() override;
	void OnRightClick() override;

	int GetPointAtPosition(Vec2 position);
};