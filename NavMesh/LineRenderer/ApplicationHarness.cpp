#include "ApplicationHarness.h"
#include "GLFWCallbacks.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "Maths.h"
#include "Utilities.h"
#include <iostream>

GLFWwindow* ApplicationHarness::window = nullptr;

ApplicationHarness::ApplicationHarness(Application* appInit) : app(appInit)
{
	if (window != nullptr)
	{
		std::cout << "Failed to create application harness - static window pointer already set!\n";
		return;
	}
	if (!glfwInit())
	{
		std::cout << "Failed to create application harness - glfw initialisation failed.\n";
		return;
	}
	if (app == nullptr)
	{
		std::cout << "Failed to create application harness - passed null application pointer.\n";
		return;
	}

	AppInfo appInfo = app->GetAppInfo();

	fixedFramerate = appInfo.fixedFramerate;
	//Can choose resolution here.
	window = glfwCreateWindow(
		appInfo.horizontalResolution, 
		appInfo.verticalResolution, 
		appInfo.appName, nullptr, nullptr);

	//This is the somewhat hacky oldschool way of making callbacks work without everything having to be global. Look
	//at the way the function callbacks work to get an idea of what's going on.
	glfwSetWindowUserPointer(window, (void*)this);	

	//These functions will be able to reference the testbed object via the user pointer.
	glfwSetWindowSizeCallback(window, WindowResizeCallback);
	glfwSetMouseButtonCallback(window, MouseButtonCallback);
	glfwSetScrollCallback(window, MouseWheelCallback);
	glfwSetKeyCallback(window, KeyPressCallback);
	glfwSetWindowContentScaleCallback(window, PixelDensityChangeCallback);

	if (!window)
	{
		//If the window failed to create for some reason, abandon ship.
		glfwTerminate();
		return;
	}

	//We have to tell glfw to use the OpenGL context from the window.
	glfwMakeContextCurrent(window);

	//This is where GLAD gets set up. After this point we can use openGL functions.
	if (!gladLoadGL())
	{
		return;
	}
	glfwSwapInterval(1);

	float xScale, yScale;
	glfwGetWindowContentScale(window, &xScale, &yScale);
	float pixelDensityScale = sqrtf(xScale * yScale);

	ImGui::CreateContext();

	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init();
	ImGui::GetIO().FontGlobalScale = pixelDensityScale;

	simpleShader = ShaderProgram("Simple.vsd", "Simple.fsd");
	simpleShader.UseShader();

	lines.Initialise();
	app->lines = &lines;

	glClearColor(
		appInfo.backgroundColour.r,
		appInfo.backgroundColour.g,
		appInfo.backgroundColour.b, 1);

	glLineWidth(appInfo.lineWidth * pixelDensityScale);

	grid.Initialise();
	
	float gridPosExtreme = appInfo.grid.extent * appInfo.grid.unit;
	float gridUnit = appInfo.grid.unit;
	int gridExtent = (int)appInfo.grid.extent;
	for (int i = -gridExtent; i <= gridExtent; i++)
	{
		
		Colour colour = (i == 0) ? appInfo.grid.mainAxesColour : appInfo.grid.linesColour;
		grid.DrawLineSegment({ i * gridUnit, -gridPosExtreme }, { i * gridUnit, gridPosExtreme }, colour);
		grid.DrawLineSegment({ -gridPosExtreme, i * gridUnit }, { gridPosExtreme, i * gridUnit }, colour);
	}
	if (appInfo.grid.showBasisLines)
	{
		grid.DrawLineSegment(Vec2(), Vec2(appInfo.grid.basisLineLength, 0), appInfo.grid.positiveXLineColour);
		grid.DrawLineSegment(Vec2(), Vec2(0, appInfo.grid.basisLineLength), appInfo.grid.positiveYLineColour);
	}
	grid.Compile();

	showGrid = appInfo.grid.show;

	app->Initialise();
}

ApplicationHarness::~ApplicationHarness()
{
	delete app;
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
	glfwTerminate();
	window = nullptr;
}

void ApplicationHarness::Run()
{
	double fixedDelta = 1.0 / fixedFramerate;
	double currentTime = glfwGetTime();
	double accumulator = 0.0;

	while (IsRunning())
	{

		double newTime = glfwGetTime();
		double frameTime = newTime - currentTime;
		currentTime = newTime;

		accumulator += frameTime;

		while (accumulator >= fixedDelta)
		{
			Update((float)fixedDelta);
			accumulator -= fixedDelta;
		}

		if (IsRunning())
		{
			Render();
		}
	}
}

void ApplicationHarness::Update(float delta)
{
	int width, height;
	glfwGetWindowSize(window, &width, &height);
	aspectRatio = width / (float)height;

	double cursorX, cursorY;
	glfwGetCursorPos(window, &cursorX, &cursorY);
	
	//Because we know the camera is always orthographic (and axis-aligned) we can
	//do what would normally be a matrix inverse operation more easily with some remaps.
	Vec2 cameraCentre = app->GetCameraPos();
	float cameraHeight = app->GetCameraHeight();
	float cameraWidth = cameraHeight * aspectRatio;
	float cameraLeft = cameraCentre.x - cameraWidth / 2.0f;
	float cameraRight = cameraCentre.x + cameraWidth / 2.0f;
	app->cursorPos.x = Remap((float)cursorX, 0, (float)width, cameraLeft, cameraRight);
	float cameraTop = cameraCentre.y + cameraHeight / 2.0f;
	float cameraBottom = cameraCentre.y - cameraHeight / 2.0f;
	app->cursorPos.y = Remap((float)cursorY, 0, (float)height, cameraTop, cameraBottom);

	if (!ImGui::GetIO().WantCaptureMouse)
	{
		app->leftMouseDown = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
		app->rightMouseDown = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
	}

	app->appTime = (float)glfwGetTime();
	lines.Clear();

	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();

	if (!app->GetAppInfo().camera.disable)
	{
		app->MoveCameraScaled(GetInputDirection(Key::W, Key::A, Key::S, Key::D) * delta * app->GetAppInfo().camera.cameraSpeed);
	}
	lines.UpdateWithCameraInfo(cameraCentre, cameraHeight, aspectRatio);

	app->Update(delta);

	lines.Compile();

	ImGui::Render();
}

void ApplicationHarness::Render()
{
	glClear(GL_COLOR_BUFFER_BIT);
	float orthoMat[16];
	PopulateCameraTransform(orthoMat);
	simpleShader.SetMat4Uniform("vpMatrix", orthoMat);
	if (showGrid) grid.Draw();
	lines.Draw();


	//Render tends to get called once or twice before Update
	//gets called, so we need to make sure this info exists.
	ImDrawData* drawData = ImGui::GetDrawData();
	if (drawData) ImGui_ImplOpenGL3_RenderDrawData(drawData);
	
	glfwSwapBuffers(window);
	glfwPollEvents();
}

bool ApplicationHarness::IsRunning() const
{
	return !glfwWindowShouldClose(window);
}

void ApplicationHarness::OnMouseClick(int mouseButton)
{
	if (!ImGui::GetIO().WantCaptureMouse)
	{
		switch (mouseButton)
		{
		case 0:
			app->OnLeftClick();
			break;
		case 1:
			app->OnRightClick();
			break;
		case 2:
			app->OnMiddleClick();
			break;
		}
	}
}

void ApplicationHarness::OnMouseRelease(int mouseButton)
{
	switch (mouseButton)
	{
	case 0:
		app->OnLeftRelease();
		break;
	case 1:
		app->OnRightRelease();
		break;
	case 2:
		app->OnMiddleRelease();
	}
}

void ApplicationHarness::OnMouseScroll(bool positive)
{
	if (!app->GetAppInfo().camera.disable)
	{
		float zoom = app->GetAppInfo().camera.zoomFactor;
		app->ScaleCameraHeight(positive ? 1.0f / zoom : zoom);
	}
	app->OnMouseScroll(positive);
}

void ApplicationHarness::OnKeyPress(Key key)
{
	app->OnKeyPress(key);
}
void ApplicationHarness::OnKeyRelease(Key key)
{
	app->OnKeyRelease(key);
}

bool ApplicationHarness::IsKeyDown(Key key)
{
	if (!ImGui::GetIO().WantCaptureKeyboard)
	{
		return glfwGetKey(window, (int)key) == GLFW_PRESS;
	}
	else
	{
		return false;
	}
}

float ApplicationHarness::GetInputAxis(Key negativeKey, Key positiveKey)
{
	float negativeValue = IsKeyDown(negativeKey) ? -1.0f : 0.0f;
	float positiveValue = IsKeyDown(positiveKey) ? 1.0f : 0.0f;
	return negativeValue + positiveValue;
}

Vec2 ApplicationHarness::GetInputDirection(Key upKey, Key leftKey, Key downKey, Key rightKey, bool normalise)
{
	Vec2 output(GetInputAxis(leftKey, rightKey), GetInputAxis(downKey, upKey));
	if (normalise && (output.x != 0.0f || output.y != 0.0f))
	{
		output.Normalise();
	}
	return output;
}

void ApplicationHarness::SetPixelDensityScale(float scale)
{
	glLineWidth(app->GetAppInfo().lineWidth * scale);
	ImGui::GetIO().FontGlobalScale = scale;
}

void ApplicationHarness::PopulateCameraTransform(float* matrix) const
{
	//Mildly hacky creation of an orthographic matrix (near/far clip at -1 to 1).
	//Not really worth setting up a Mat4 class just for this.
	memset(matrix, 0, sizeof(float) * 16);

	float cameraHeight = app->GetCameraHeight();
	float cameraWidth = aspectRatio * cameraHeight;
	Vec2 cameraCentre = app->GetCameraPos();
	matrix[0] = 2.0f / cameraWidth;
	matrix[5] = 2.0f / cameraHeight;
	matrix[10] = -1.0f;
	matrix[12] = -2.0f * cameraCentre.x / cameraWidth;
	matrix[13] = -2.0f * cameraCentre.y / cameraHeight;
	matrix[15] = 1.0f;
}