#include "Grid.h"


#include "Maths.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <iostream>


void Grid::LoadFromImage(std::string filename)
{
	delete[] data;
	
	int channelCount;
	unsigned char *imageData = stbi_load(filename.c_str(), &width, &height, &channelCount, 4);


	data = new TileType[width * height];

	std::cout << "Width is: " << width << ", height is: " << height << ", channel count is: " << channelCount << "\n";
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			//data[x + y * width] = *((unsigned int*) &imageData[(x + y * width) * 4]);

			int integerIndex = x + y * width;
			int charIndex = integerIndex * sizeof(int);
			imageData[charIndex + 3] = 0;

			data[integerIndex] = *((TileType*)(imageData + charIndex));
		}
	}


	stbi_image_free(imageData);
}

TileType& Grid::At(int xCoord, int yCoord)
{
	return data[xCoord + yCoord * width];
}
TileType& Grid::AtWrap(int xCoord, int yCoord)
{
	xCoord = xCoord % width;
	if (xCoord < 0) xCoord += width;
	yCoord = yCoord % height;
	if (yCoord < 0) yCoord += height;

	return data[xCoord + yCoord * width];
}
TileType& Grid::AtClamp(int xCoord, int yCoord)
{
	xCoord = Clamp(xCoord, 0, width - 1);
	yCoord = Clamp(yCoord, 0, height - 1);

	return data[xCoord + yCoord * width];
}

Grid::~Grid()
{
	delete[] data;
}
