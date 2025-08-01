#pragma once

#include <string>
#include <memory>

enum class TileType
{
	EMPTY = 0xFFFFFF,
	OBSTACLE = 0x000000,
};

class Grid
{
	TileType* data = nullptr;
	int mWidth;
	int mHeight;
	float mCellSize;
	

public:

	Grid(float size);
	void LoadFromImage(std::string filename);

	TileType& At(int xCoord, int yCoord);
	TileType& At(float xCoord, float yCoord);
	TileType& AtWrap(int xCoord, int yCoord);
	TileType& AtClamp(int xCoord, int yCoord);

	~Grid();
	Grid(Grid& other) = delete;
	Grid& operator=(Grid& other) = delete;

	int GetHeight() const { return mHeight; }
	int GetWidth() const { return mWidth; }
	float GetCellSize() const{ return mCellSize; }

};