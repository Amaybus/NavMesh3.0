#pragma once

#include <string>
#include <memory>

enum class TileType
{
	EMPTY = 0xFFFFFF,
	OBSTACLE = 0x000000,
	POINT = 0x0000FF
};

class Grid
{
	TileType* data = nullptr;
	int width;
	int height;
	

public:

	Grid() = default;
	void LoadFromImage(std::string filename);

	TileType& At(int xCoord, int yCoord);
	TileType& AtWrap(int xCoord, int yCoord);
	TileType& AtClamp(int xCoord, int yCoord);

	~Grid();
	Grid(Grid& other) = delete;
	Grid& operator=(Grid& other) = delete;

	int GetHeight() const { return height; }
	int GetWidth() const { return width; }

};