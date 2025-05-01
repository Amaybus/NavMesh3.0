#pragma once
#include "Maths.h"
#include "AppInfo.h"
enum class Key;
class LineRenderer;

class Application
{
public:
	virtual void Initialise() {}
	virtual void Update(float delta) = 0;
	virtual ~Application() = default;
	bool leftMouseDown;
	bool rightMouseDown;
	Vec2 cursorPos;
	LineRenderer* lines = nullptr;

	float appTime = 0.0f;

	virtual void OnLeftClick() {}
	virtual void OnLeftRelease() {}
	virtual void OnRightClick() {}
	virtual void OnRightRelease() {}
	virtual void OnMiddleClick() {}
	virtual void OnMiddleRelease() {}

	virtual void OnKeyPress(Key key) {}
	virtual void OnKeyRelease(Key key) {}

	virtual void OnMouseScroll(bool positive) {}

	const AppInfo& GetAppInfo() const {	return appInfo; };

	float GetCameraHeight() const { return cameraHeight; }
	Vec2 GetCameraPos() const { return cameraCentre; }


	//Used by application harness for optional default camera controls
	void ScaleCameraHeight(float scale) { cameraHeight *= scale; }
	void MoveCameraScaled(Vec2 movement) { cameraCentre += movement * cameraHeight; }

protected:
	float cameraHeight = 10.0f;
	Vec2 cameraCentre = { 0.0f, 0.0f };
	AppInfo appInfo;
};