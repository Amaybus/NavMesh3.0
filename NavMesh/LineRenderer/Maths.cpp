#include "Maths.h"



float Remap(float val, float oldMin, float oldMax, float newMin, float newMax)
{
	return ((val - oldMin) / (oldMax - oldMin)) * (newMax - newMin) + newMin;
}
