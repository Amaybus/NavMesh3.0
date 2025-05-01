#include "Example.h"
#include "LineRenderer.h"
#include "imgui.h"

Example::Example()
{
	//Use the constructor to set up the application info, because the harness
	//needs this information early so it can set the name of the window when
	//it creates it.
	appInfo.appName = "Example Program";
}

void Example::Initialise()
{
	//You should set up your application here. This function gets called
	//after the window is set up and the rendering context is created, so if
	//you're doing anything weird with rendering, OpenGL functions will be
	//available at this point.
}

void Example::Update(float delta)
{
	//Everything that your program does every frame should go here.
	//This includes rendering done with the line renderer!


}

void Example::OnLeftClick()
{

}
