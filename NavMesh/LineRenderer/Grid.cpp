#include "Grid.h"


#include "Maths.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <iostream>


Grid::Grid(float size) : mCellSize(size)
{
}

void Grid::LoadFromImage(std::string filename)
{
	delete[] data;
	
	int channelCount;
	unsigned char *imageData = stbi_load(filename.c_str(), &mWidth, &mHeight, &channelCount, 4);


	data = new TileType[mWidth * mHeight];

	std::cout << "Width is: " << mWidth << ", height is: " << mHeight << ", channel count is: " << channelCount << "\n";
	for (int y = 0; y < mHeight; y++)
	{
		for (int x = 0; x < mWidth; x++)
		{
			//data[x + y * width] = *((unsigned int*) &imageData[(x + y * width) * 4]);

			int integerIndex = x + y * mWidth;
			int charIndex = integerIndex * sizeof(int);
			imageData[charIndex + 3] = 0;

			data[integerIndex] = *((TileType*)(imageData + charIndex));
		}
	}


	stbi_image_free(imageData);
}

TileType& Grid::At(int xCoord, int yCoord)
{
	return data[xCoord + yCoord * mWidth];
}

TileType& Grid::At(float xCoord, float yCoord)
{
	return data[(int)xCoord + (int)yCoord * mWidth];
}

TileType& Grid::AtWrap(int xCoord, int yCoord)
{
	xCoord = xCoord % mWidth;
	if (xCoord < 0) xCoord += mWidth;
	yCoord = yCoord % mHeight;
	if (yCoord < 0) yCoord += mHeight;

	return data[xCoord + yCoord * mWidth];
}

TileType& Grid::AtClamp(int xCoord, int yCoord)
{
	xCoord = Clamp(xCoord, 0, mWidth - 1);
	yCoord = Clamp(yCoord, 0, mHeight - 1);

	return data[xCoord + yCoord * mWidth];
}

Grid::~Grid()
{
	delete[] data;
}
