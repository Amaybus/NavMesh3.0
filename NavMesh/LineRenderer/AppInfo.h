#pragma once
#include "Colour.h"


//Info about how to render the grid specifically. If 'show' is set to false it'll still set things
//up, in case it gets set to true later.
struct GridInfo
{
	bool show = true;
	Colour mainAxesColour = Colour::WHITE.Multiply(0.8f);	//The colour of the cardinal axes lines
	Colour linesColour = Colour::WHITE.Multiply(0.3f);	//The colour of the more faded spacing lines.
	unsigned int extent = 10;	//How many grid lines there are in all four directions (positive and negative).
	float unit = 1.0f;	//How far apart grid lines are.

	bool showBasisLines = true;	//The lines that show the coloured X and Y axis lines.
	Colour positiveXLineColour = Colour::RED;
	Colour positiveYLineColour = Colour::GREEN;
	float basisLineLength = 1.0f;
};

struct CameraControls
{
	bool disable = false;
	float cameraSpeed = 0.8f;
	float zoomFactor = 1.2f;
};


//Every application can override this info to help the 
//application harness know how to set things up.
struct AppInfo
{
	const char* appName = "Line Renderer";
	unsigned int horizontalResolution = 1280;
	unsigned int verticalResolution = 720;
	float lineWidth = 3.0f;
	Colour backgroundColour = Colour::BLACK;
	unsigned int fixedFramerate = 60;
	CameraControls camera;
	GridInfo grid;
};