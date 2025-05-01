#pragma once
#include "Graphics.h"
#include "Maths.h"
#include "LineRenderer.h"
#include "ShaderProgram.h"
#include "Application.h"
#include "Key.h"

class ApplicationHarness
{
private:
	static GLFWwindow* window;

	Application* app = nullptr;

	ShaderProgram simpleShader;
	float aspectRatio;

	LineRenderer grid;
	LineRenderer lines;

	unsigned int fixedFramerate;
	bool showGrid;

	void PopulateCameraTransform(float* matrix) const;


public:
	ApplicationHarness(Application* appInit);
	~ApplicationHarness();

	ApplicationHarness(const ApplicationHarness& other) = delete;
	ApplicationHarness& operator=(const ApplicationHarness& other) = delete;
	
	void Run();

	void Update(float delta);
	void Render();

	bool IsRunning() const;

	//1 for left, 2 for right, 3 for middle. Other button codes will get sent for other buttons but I'm not sure how they'd map to a particular mouse.
	void OnMouseClick(int mouseButton);
	void OnMouseRelease(int mouseButton);
	void OnMouseScroll(bool positive);

	void OnKeyPress(Key key);
	void OnKeyRelease(Key key);

	static bool IsKeyDown(Key key);
	static float GetInputAxis(Key negativeKey, Key positiveKey);
	static Vec2 GetInputDirection(Key upKey, Key leftKey, Key downKey, Key rightKey, bool normalise = true);

	void SetPixelDensityScale(float scale);
};
