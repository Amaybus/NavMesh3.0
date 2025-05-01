#include "GLFWCallbacks.h"
#include "ApplicationHarness.h"

void WindowResizeCallback(GLFWwindow* window, int width, int height)
{
	glViewport(0, 0, width, height);
}

void MouseButtonCallback(GLFWwindow* window, int button, int action, int mods)
{
	ApplicationHarness* programPointer = (ApplicationHarness*)glfwGetWindowUserPointer(window);	//This is an unsafe coercive cast, but we just have to be careful and live with it.
	if (action == GLFW_PRESS)
	{
		programPointer->OnMouseClick(button);
	}
	else if (action == GLFW_RELEASE)
	{
		//We don't just do 'else' here because 'action' can also be GLFW_REPEAT. Operating systems send this when a button or key is held down, depending on the settings, but we're going to ignore it.
		programPointer->OnMouseRelease(button);
	}
}

void MouseWheelCallback(GLFWwindow* window, double xDelta, double yDelta)
{
	ApplicationHarness* programPointer = (ApplicationHarness*)glfwGetWindowUserPointer(window);
	programPointer->OnMouseScroll(yDelta > 0);
}

void KeyPressCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	ApplicationHarness* programPointer = (ApplicationHarness*)glfwGetWindowUserPointer(window);
	if (action == GLFW_PRESS)
	{
		programPointer->OnKeyPress((Key)key);
	}
	else if (action == GLFW_RELEASE)
	{
		programPointer->OnKeyRelease((Key)key);
	}
}

void PixelDensityChangeCallback(GLFWwindow* window, float xScale, float yScale)
{
	ApplicationHarness* programPointer = (ApplicationHarness*)glfwGetWindowUserPointer(window);
	programPointer->SetPixelDensityScale(sqrtf(xScale * yScale));	//Geometric average because it feels right.
}
