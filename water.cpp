/*!*****************************************************************************
\file    water.cpp
\author  Seth Glaser
\par     Email: seth.g\@digipen.edu
\par     Course: CS280
\par     Assignment 6
\date    4/11/2018
\brief   This is the solution to the water retension problem.
*******************************************************************************/

#include "water.h"    // waterret
#include <fstream>    // std::ifstream
#include <iostream>   // std::ifstream
#include <vector>     // std::vector
#include <string.h>   // memset
#include <algorithm>  // std::min

// STATIC VARIABLES
static int gridHeight;
static int gridWidth;

// DECLARATIONS
void UpdateRec(const std::vector<int> &walls, std::vector<int> &water, std::vector<int> &update, int position, int height);


// Does the work of UpdateRec
void UpdateWork(const std::vector<int> &walls, std::vector<int> &water, std::vector<int> &update, int position, int height)
{
	// Updating
	update[position] = 1;

	// Adjust the water
	water[position] = std::min((height - walls[position]), (water[position]));

	if (water[position] < 0)
		water[position] = 0;

	// Adjust height, if necessary
	if (walls[position] > height)
		height = walls[position];

	// If the water is below 0
	if (water[position] < 0)
	{
		water[position] = 0;

		// They will be updated later
	}
	else
	{
		// Otherwise
		UpdateRec(walls, water, update, position, height);
	}
}

// Update a node (recursive)
void UpdateRec(const std::vector<int> &walls, std::vector<int> &water, std::vector<int> &update, int position, int height)
{
	// This position has been checked
	update[position] = 1;

	// Check left
	if (position - 1 > 0 && (position % gridWidth) && water[position - 1] != 0)
	{
		// If it hasn't been updated yet
		if(!update[position - 1])
		{
			// Do the update work
			UpdateWork(walls, water, update, position - 1, height);
		}

	}

	// Check up
	if (position - gridWidth > 0 && water[position - gridWidth] != 0)
	{
		// If it hasn't been updated yet
		if (!update[position - gridWidth])
		{
			// Do the update work
			UpdateWork(walls, water, update, position - gridWidth, height);
		}

	}

	// Check right
	if (position + 1 < gridHeight * gridWidth && (position + 1 % gridWidth) && water[position + 1] != 0)
	{
		// If it hasn't been updated yet
		if (!update[position + 1])
		{
			// Do the update work
			UpdateWork(walls, water, update, position + 1, height);
		}

	}

	// Check down
	if (position + gridWidth < gridHeight * gridWidth && water[position + gridWidth] != 0)
	{
		// If it hasn't been updated yet
		if (!update[position + gridWidth])
		{
			// Do the update work
			UpdateWork(walls, water, update, position + gridWidth, height);
		}

	}

}

// Update (set up)
void Update(const std::vector<int> &walls, std::vector<int> &water, std::vector<int> &update, int position, int subtract)
{
	// Adjust the water
	water[position] -= subtract;
	if (water[position] < 0)
		water[position] = 0;

	// Make the height to adhere to
	int height = walls[position] + water[position];

	// Update nearby tiles
	UpdateRec(walls, water, update, position, height);

	// Reset the updates
	memset(update.data(), 0, sizeof(int) * update.size());
}

// Scan in a direction
// Returns new runoff
int Scan(const std::vector<int> &walls, std::vector<int> &water, std::vector<int> &update, int position, int runoff)
{
	// Adding water
	if(walls[position] + water[position] < runoff)
	{
		// If there's already water, don't add
		if(!water[position])
		{
			// Add the difference missing
			water[position] += runoff - (walls[position] + water[position]);

			// No change to runoff
			return runoff;
		}
	}

	// If it holds no water, it's a wall
	if (water[position] == 0)
	{
		// Make new runoff
		return walls[position];
	}

	// Removing water
	if(walls[position] + water[position] > runoff)
	{
		// It holds water, adjust and update
		// Smallest between runoff difference and all water avaliable.
		int subtract = std::min((walls[position] + water[position] - runoff), (water[position]));

		// Update the tiles
		Update(walls, water, update, position, subtract);

		// Set the new runoff
		return walls[position] + water[position];
	}

	// No change
	return runoff;
}

// Checks all directions to see if it needs to be updated again
void FinalScan(const std::vector<int> &walls, std::vector<int> &water, std::vector<int> &update, int position)
{
	// If left
	if ((position - 1) >= 0 && (position % gridWidth))
	{
		// If left is lower in total
		if((water[position - 1] + walls[position - 1]) < (water[position] + walls[position]))
		{
			// Match it
			int subtract = (water[position] + walls[position]) - (water[position - 1] + walls[position - 1]);

			// Update the tiles
			Update(walls, water, update, position, subtract);
		}
	}
	// On left edge
	else
	{
		// Remove water
		if(water[position])
		{
			// Remove all water
			int subtract = water[position];

			// Update the tiles
			Update(walls, water, update, position, subtract);
		}
	}

	// If up
	if (position - gridWidth >= 0)
	{
		// If up is lower in total
		if((water[position - gridWidth] + walls[position - gridWidth]) < (water[position] + walls[position]))
		{
			// Match it
			int subtract = (water[position - gridWidth] + walls[position - gridWidth]) - (water[position] + walls[position]);

			// Update the tiles
			Update(walls, water, update, position, subtract);
		}
	}
	// On top edge
	else
	{
		// Remove water
		if (water[position])
		{
			// Remove all water
			int subtract = water[position];

			// Update the tiles
			Update(walls, water, update, position, subtract);
		}
	}

	// If right
	if ((position + 1) < (gridHeight * gridWidth) && (position + 1) % gridWidth)
	{
		// If right is lower in total
		if((water[position + 1] + walls[position + 1]) < (water[position] + walls[position]))
		{
			// Match it
			int subtract = (water[position + 1] + walls[position + 1]) - (water[position] + walls[position]);

			// Update the tiles
			Update(walls, water, update, position, subtract);
		}
	}
	// On right edge
	else
	{
		// Remove water
		if (water[position])
		{
			// Remove all water
			int subtract = water[position];

			// Update the tiles
			Update(walls, water, update, position, subtract);
		}
	}

	// If down
	if ((position + gridWidth) < (gridHeight * gridWidth))
	{
		// If down is lower in total
		if((water[position + gridWidth] + walls[position + gridWidth]) < (water[position] + walls[position]))
		{
			// Match it
			int subtract = (water[position + gridWidth] + walls[position + gridWidth]) - (water[position] + walls[position]);

			// Update the tiles
			Update(walls, water, update, position, subtract);
		}
	}
	// On bottom edge
	else
	{
		// Remove water
		if (water[position])
		{
			// Remove all water
			int subtract = water[position];

			// Update the tiles
			Update(walls, water, update, position, subtract);
		}
	}

}

// Public interface to solve the array
long int waterret(char const* filename)
{
	// open the file
	std::ifstream arrayFile(filename);

	// Check if the file is opened
	if(!arrayFile.is_open())
	{
		std::cout << "File not open!" << std::endl;
		return 0;
	}

	// Get the width and height
	arrayFile >> gridHeight;
	arrayFile >> gridWidth;

	// Make an array to hold the walls
	std::vector<int> walls;
	walls.reserve(gridHeight * gridWidth);

	// Fill in the walls
	for(int i = 0; i < gridHeight * gridWidth; ++i)
	{
		// Holds the value of the file input
		int value;
		arrayFile >> value;

		// Add the value
		walls.push_back(value);
	}

	// Create an array of the same size for water
	std::vector<int> water;
	water.reserve(gridHeight * gridWidth);
	for (int i = 0; i < gridHeight * gridWidth; ++i)
		water.push_back(0);

	// Make an array to track update statuses
	std::vector<int> update;
	update.reserve(gridHeight * gridWidth);
	for (int i = 0; i < gridHeight * gridWidth; ++i)
		update.push_back(0);

	// Step through the walls 4 times

	// Scan left->right
	for(int i = 0; i < gridHeight; ++i)
	{
		// Tracks runoff
		int runoff = 0;
		// Tracks width
		int counter = 0;

		// While we are still in this row
		while(counter < gridWidth)
			// Update the water (increase counter)
			runoff = Scan(walls, water, update, ((i * gridWidth) + counter++), runoff);

	}

	// Scan top->bottom
	for (int i = 0; i < gridWidth; ++i)
	{
		// Tracks runoff
		int runoff = 0;
		// Tracks height
		int counter = 0;

		// While we are still in this column
		while (counter < gridHeight)
			// Update the water (increase counter)
			runoff = Scan(walls, water, update, ((counter++ * gridWidth) + i), runoff);

	}

	// Scan right->left
	for (int i = 0; i < gridHeight; ++i)
	{
		// Tracks runoff
		int runoff = 0;
		// Tracks width
		int counter = gridWidth - 1;

		// While we are still in this row
		while (counter >= 0)
			// Update the water (decrease counter)
			runoff = Scan(walls, water, update, ((i * gridWidth) + counter--), runoff);

	}

	// Scan bottom->top
	for (int i = 0; i < gridWidth; ++i)
	{
		// Tracks runoff
		int runoff = 0;
		// Tracks height
		int counter = gridHeight - 1;

		// While we are still in this column
		while (counter >= 0)
			// Update the water (decrease counter)
			runoff = Scan(walls, water, update, ((counter-- * gridWidth) + i), runoff);

	}

	// FINAL CHECK for all water tiles
	int finalPosition = 0;
	for(auto &iter : water)
	{
		// If there's water
		if (iter != 0)
			FinalScan(walls, water, update, finalPosition);
	
		++finalPosition;
	}

	// add up all values in the water array
	long int waterCount = 0;
	for(auto &iter : water)
	{
		waterCount += iter;
	}

	return waterCount;
}
