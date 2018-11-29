#include <pch.h>
#include "Projects/ProjectTwo.h"
#include "P2_Pathfinding.h"

/////////////////////////////
// DEFINES AND STATICS
/////////////////////////////

#define NO_ELEMENT -1    // When an element doesn't exist in an array
#define TILE_WIDTH 2.0f  // Width of a tile
#define NO_PARENT -1.0f  // If a node's parent x and y is this value, this node is on the closed list
#define SHORTIFY 100     // Convert from float to short

std::array<AStarPather::Neighbor, MAX_NODES> AStarPather::neighborList;  // Holds all preprocessed neighbors
std::array<AStarPather::Node, MAX_NODES> AStarPather::theMap;            // Holds all possible nodes
PathRequest AStarPather::currentRequest;                                 // The current request
static short SQRT_TWO = 142;                                             // Square root of two

#pragma region Extra Credit
bool ProjectTwo::implemented_floyd_warshall()
{
    return false;
}

bool ProjectTwo::implemented_goal_bounding()
{
    return false;
}

bool ProjectTwo::implemented_jps_plus()
{
    return false;
}
#pragma endregion

bool AStarPather::initialize()
{
	// Preprocess the map
	Callback cb = std::bind(&AStarPather::CalculateNeighbors, this);
	Messenger::listen_for_message(Messages::MAP_CHANGE, cb);

    return true; // return false if any errors actually occur, to stop engine initialization
}

void AStarPather::shutdown()
{
}

PathResult AStarPather::compute_path(PathRequest &request)
{
	// If first time through, set up all variables
	if (request.newRequest)
	{
		// Set the request
		currentRequest = request;

		// Make start node
		Node start;
		start.parent = terrain->get_grid_position(request.start);
		start.position = terrain->get_grid_position(request.start);
		start.givenCost = 0;
		start.estimateCost = GetEstimate(start.position, terrain->get_grid_position(request.goal));

		// Reset the tails
		openNodes = 1;

		// Clear the list
		memset(theMap.data(), NO_ELEMENT, MAX_NODES * sizeof(Node));

		// Push Start Node onto the Open List
		PushNodeOpen(start);
	}
	
	// Begin the while loop
	while (openNodes)
	{
		// Pop cheapest node off open list
		Node currentNode = PopCheapest();

		// If node is a dud
		if (currentNode.estimateCost == static_cast<short>(NO_PARENT * SHORTIFY))
			return PathResult::IMPOSSIBLE;

		// If node is the Goal Node, then path found
		if (currentNode.position == terrain->get_grid_position(request.goal))
		{
			// Generate the path
			CreatePath(GetPosition(currentNode.position), request);
			
			// If rubberbanding
			if (request.settings.rubberBanding)
				Rubberband(request.path);

			// If smoothing
			if (request.settings.smoothing)
				Smooth(request.path);

			return PathResult::COMPLETE;
		}

		// Find all neighboring nodes
		std::list<Node> neighbors = GetNeighbors(currentNode);

		// For all neighboring child nodes
		for (auto &iter : neighbors)
		{
			// Get the node at this position
			int nodePos = GetPosition(iter.position);

			// If this position has never been seen before
			if (theMap[nodePos] == Node())
			{
				// Put on open list
				PushNodeOpen(iter);

				// Add a new node
				++openNodes;
			}
			// This position exists on the list
			else
			{
				// If the neighbor is cheaper than one in the list
				if (iter.TotalCost() < (GetEstimate(theMap[nodePos].position, Position(terrain->get_grid_position(request.goal))) + theMap[nodePos].givenCost))
				{
					PushNodeOpen(iter);
					// Add a new node
					++openNodes;
				}

			}

		}

		// If single step
		if (request.settings.singleStep)
			return PathResult::PROCESSING;

	}

	// If Open List is empty, return FAIL
	return PathResult::IMPOSSIBLE;

}


/////////////////////////////
// NODES
/////////////////////////////

AStarPather::Node AStarPather::CreateNode(Position nodePosition, Node parent)
{
	// The node to return
	Node returnNode;

	returnNode.parent = parent.position;
	returnNode.position = nodePosition;
	returnNode.givenCost = parent.givenCost + SHORTIFY;
	returnNode.estimateCost = GetEstimate(nodePosition, terrain->get_grid_position(currentRequest.goal));

	return returnNode;
}

AStarPather::Node AStarPather::CreatePosition(Position position)
{
	// The node to return
	Node returnNode;

	returnNode.parent = position;
	returnNode.position = position;
	returnNode.givenCost = 0;
	returnNode.estimateCost = 0;

	return returnNode;
}

int AStarPather::GetPosition(Position position)
{
	// Use a formula to get a spot in the array
	return GetIndex(position);
}

int AStarPather::GetNode(Node current)
{
	// Use the formula
	int position = GetIndex(current.position);

	// Check if it's the same
	
	// If their parent isn't the same
	if (theMap[position].parent != current.parent)
		return NO_ELEMENT;
	// If their position isn't the same
	if (theMap[position].position != current.position)
		return NO_ELEMENT;
	// If their given cost isn't the same
	if (theMap[position].givenCost != current.givenCost)
		return NO_ELEMENT;
	// If their estimate cost isn't the same
	if (theMap[position].estimateCost != current.estimateCost)
		return NO_ELEMENT;

	// It didn't exist
	return position;
}


/////////////////////////////
// ESTIMATES
/////////////////////////////

short AStarPather::GetEstimate(Position begin, Position end)
{
	// Depends on what heuristic we're using
	switch (currentRequest.settings.heuristic)
	{
		case Heuristic::OCTILE:
			return static_cast<short>(Octile(begin, end) * currentRequest.settings.weight);
		case Heuristic::CHEBYSHEV:
			return static_cast<short>(Chebyshev(begin, end) * currentRequest.settings.weight);
		case Heuristic::MANHATTAN:
			return static_cast<short>(Manhattan(begin, end) * currentRequest.settings.weight);
		case Heuristic::EUCLIDEAN:
			return static_cast<short>(Euclidean(begin, end) * currentRequest.settings.weight);
		case Heuristic::NUM_ENTRIES:
			return 0;
		default:
			return 0;
	}

	// Fail case
	return 0;

}

short AStarPather::Octile(Position begin, Position end)
{
	// Variable because we do this calculation twice
	int minimum = std::min(abs(begin.y - end.y), abs(begin.x - end.x));

	// Min(xDiff, yDiff) * sqrt(2) + Max(xDiff, yDiff) - Min(xDiff, yDiff)
	return static_cast<short>((minimum * SQRT_TWO) + (std::max(abs(begin.y - end.y), abs(begin.x - end.x)) - minimum) * SHORTIFY);
}

short AStarPather::Chebyshev(Position begin, Position end)
{
	// Max(xDiff, yDiff)
	return static_cast<short>(std::max(static_cast<float>(abs(begin.y - end.y)), static_cast<float>(abs(begin.x - end.x))) * SHORTIFY);
}

short AStarPather::Manhattan(Position begin, Position end)
{
	// xDiff + yDiff
	return static_cast<short>(static_cast<float>(abs(begin.y - end.y)) + static_cast<float>(abs(begin.x - end.x)) * SHORTIFY);
}

short AStarPather::Euclidean(Position begin, Position end)
{
	// sqrt(xDiff^2 + yDiff^2)
	return static_cast<short>(sqrt((static_cast<float>(abs(begin.y - end.y))) + static_cast<float>((abs(begin.x - end.x)))) * SHORTIFY);
}


/////////////////////////////
// ALGORITHM
/////////////////////////////

void AStarPather::CalculateNeighbors()
{
	// Clear the array
	memset(neighborList.data(), 0, MAX_NODES * sizeof(Neighbor));
	
	// Holds the width and height
	int width = terrain->get_map_width();
	int height = terrain->get_map_height();
	int totalTiles = width * height;
	
	// For every point
	for (int i = 0; i < totalTiles; ++i)
	{
		// If it's a wall, skip
		if (terrain->is_wall((i / width), (i % width)))
			continue;
	
		// Rows
		for (int j = -1; j <= 1; ++j)
		{
			// Columns
			for (int k = -1; k <= 1; ++k)
			{
				// If it's the center tile
				if (j == 0 && k == 0)
					continue;
	
				// Make a gridpos of the current pos
				GridPos neighborTile;
				neighborTile.row = (i / width) + j;
				neighborTile.col = (i % width) + k;
	
				// If the terrain isn't valid
				if (!terrain->is_valid_grid_position(neighborTile))
					continue;
	
				// If the terrain is a wall
				if (terrain->is_wall(neighborTile))
					continue;
	
				// If the surroundings are walls
				bool diagonal = false;
	
				// If this cuts a diagonal
				if (j != 0 && k != 0)
				{
					// if either parent + j and parent + i are walls
					if (terrain->is_wall((i / width) + j, (i % width))
						|| terrain->is_wall((i / width), (i % width) + k))
						continue;
	
					// It is cutting a diagonal
					diagonal = true;
				}
	
				// Set the variable
				if (j == -1)
				{
					if (k == -1)
					{
						neighborList[i].bottomLeft = true;
					}
					else if (k == 0)
					{
						neighborList[i].bottom = true;
					}
					else if (k == 1)
					{
						neighborList[i].bottomRight = true;
					}
				}
				else if (j == 0)
				{
					if (k == -1)
					{
						neighborList[i].left = true;
					}
					else if (k == 1)
					{
						neighborList[i].right = true;
					}
				}
				else
				{
					if (k == -1)
					{
						neighborList[i].topLeft = true;
					}
					else if (k == 0)
					{
						neighborList[i].top = true;
					}
					else if (k == 1)
					{
						neighborList[i].topRight = true;
					}
				}
	
			}
		}
	
	}
	
}

std::list<AStarPather::Node> AStarPather::GetNeighbors(Node current)
{
	// Holds the return list
	std::list<Node> returnList;
	
	// Look at precalculated neighbors
	
	// Adjacent neighbors
	if (neighborList[GetIndex(current.position)].bottom)
		returnList.push_back(CreateNode(Position(current.position.x, current.position.y - 1), current));
	if (neighborList[GetIndex(current.position)].left)
		returnList.push_back(CreateNode(Position(current.position.x - 1, current.position.y), current));
	if (neighborList[GetIndex(current.position)].right)
		returnList.push_back(CreateNode(Position(current.position.x + 1, current.position.y), current));
	if (neighborList[GetIndex(current.position)].top)
		returnList.push_back(CreateNode(Position(current.position.x, current.position.y + 1), current));
	
	// Diagonal neighbors
	if (neighborList[GetIndex(current.position)].bottomRight)
	{
		Node diagonal = CreateNode(Position(current.position.x + 1, current.position.y - 1), current);
		diagonal.givenCost = static_cast<short>(current.givenCost + SQRT_TWO);
		returnList.push_back(diagonal);
	}
	if (neighborList[GetIndex(current.position)].bottomLeft)
	{
		Node diagonal = CreateNode(Position(current.position.x - 1, current.position.y - 1), current);
		diagonal.givenCost = static_cast<short>(current.givenCost + SQRT_TWO);
		returnList.push_back(diagonal);
	}
	if (neighborList[GetIndex(current.position)].topLeft)
	{
		Node diagonal = CreateNode(Position(current.position.x - 1, current.position.y + 1), current);
		diagonal.givenCost = static_cast<short>(current.givenCost + SQRT_TWO);
		returnList.push_back(diagonal);
	}
	if (neighborList[GetIndex(current.position)].topRight)
	{
		Node diagonal = CreateNode(Position(current.position.x + 1, current.position.y + 1), current);
		diagonal.givenCost = static_cast<short>(current.givenCost + SQRT_TWO);
		returnList.push_back(diagonal);
	}
	
	return returnList;
}

void AStarPather::CreatePath(int goalNode, PathRequest & request)
{
	// If the current node is the start
	if (theMap[goalNode].position == terrain->get_grid_position(request.start))
	{
		// Add to the list and return
		request.path.push_back(terrain->get_world_position(theMap[goalNode].position.y, theMap[goalNode].position.x));
		return;
	}

	// CreatePath with next node
	CreatePath(GetPosition(theMap[goalNode].parent), request);
	
	// Add to the list and return
	request.path.push_back(terrain->get_world_position(theMap[goalNode].position.y, theMap[goalNode].position.x));
	return;
}

void AStarPather::Rubberband(WaypointList &path)
{
	// Check if there are enough nodes
	if (path.size() < 3)
		return;

	// Flip the list
	path.reverse();

	// 3 points of iters
	auto iter1 = path.begin();    // closest to the goal
	auto iter2 = iter1; ++iter2;  // middle node
	auto iter3 = iter2; ++iter3;  // closest to the start

	// For every point starting at the goal
	while (iter3 != path.end())
	{
		// Holds the row and col difference
		bool brokeEarly = false;
		GridPos iterPos = terrain->get_grid_position(iter1._Ptr->_Myval);
		GridPos middlePos = terrain->get_grid_position(iter2._Ptr->_Myval);
		GridPos lastPos = terrain->get_grid_position(iter3._Ptr->_Myval);
		int rowDif = abs(static_cast<int>(iterPos.row - lastPos.row));
		int colDif = abs(static_cast<int>(iterPos.col - lastPos.col));
	
		// Holds the bottom left corner of the area
		GridPos position;
		position.row = abs(std::min(iterPos.row, lastPos.row));
		position.col = abs(std::min(iterPos.col, lastPos.col));

		// For every contained space
		for (int i = 0; i <= rowDif; ++i)
		{
			for (int j = 0; j <= colDif; ++j)
			{
				// If it is a wall
				if(terrain->is_wall(position.row + i, position.col + j))
				{
					brokeEarly = true;
					break;
				}

			}
	
			if (brokeEarly)
				break;
		}
	
		// No rubberband, check next 3
		if (brokeEarly)
		{
			++iter1;
			++iter2;
			++iter3;
			continue;
		}
	
		// Remove the middle node
		path.remove(*iter2);

		// The first node needs to be checked again
		iter2 = iter3;
		++iter3;
	}

	// Flip the list back
	path.reverse();

}

void AStarPather::Smooth(WaypointList & path)
{
	// Can't smooth with less than 3 points
	if (path.size() < 3)
		return;

	// Go through the entire list
	auto first1 = path.begin();
	auto first2 = first1; ++first2;

	// For a pair of points
	while (first2 != path.end())
	{
		// The difference between the points
		float zDiff = abs(first1._Ptr->_Myval.z - first2._Ptr->_Myval.z);
		float xDiff = abs(first1._Ptr->_Myval.x - first2._Ptr->_Myval.x);

		// If their distance is > 1.5 a tile
		if ((pow((zDiff), 2) + pow((xDiff), 2))
			> pow((1.5 * TILE_WIDTH), 2))
		{
			// Make the new point
			Vec3 newPoint(0,0,0);
			newPoint.z = std::min(first1._Ptr->_Myval.z, first2._Ptr->_Myval.z) + (zDiff / 2);
			newPoint.x = std::min(first1._Ptr->_Myval.x, first2._Ptr->_Myval.x) + (xDiff / 2);

			// Put in a new point
			path.insert(first2, newPoint);

			// Restart at first point of pair
			first2 = first1; ++first2;
			continue;
		}

		// Move to the next pair
		++first1;
		++first2;
	}

	// Go through the entire list
	auto second1 = path.begin();
	auto second2 = path.begin();
	auto second3 = second1; ++second3;
	auto second4 = second1; ++second4; ++second4;

	// While there are still nodes
	while (second4 != path.end())
	{
		// 3 times
		for (int i = 1; i <= 3; ++i)
		{
			// Get the result of the current spline
			Vec3 result = Vec3::CatmullRom(second1._Ptr->_Myval, second2._Ptr->_Myval, second3._Ptr->_Myval, second4._Ptr->_Myval, i / 4.0f);

			// Push it to the list
			path.insert(second3, result);
		}

		// Increment based on other iters
		second1 = second2;
		second2 = second3;
		second3 = second4;
		++second4;
	}

	// If 4th iter can't advance
	--second4;
	// 3 times
	for (int i = 1; i <= 3; ++i)
	{
		// Get the result of the current spline
		Vec3 result = Vec3::CatmullRom(second1._Ptr->_Myval, second2._Ptr->_Myval, second3._Ptr->_Myval, second4._Ptr->_Myval, i / 4.0f);

		// Push it to the list
		path.insert(second3, result);
	}

}


/////////////////////////////
// DEBUG
/////////////////////////////

void AStarPather::ColorOpenNode(Node current)
{
	// Make the position blue
	terrain->set_color(current.position.y, current.position.x, Colors::Blue);
}

void AStarPather::ColorClosedNode(Node current)
{
	// Make the position yellow
	terrain->set_color(current.position.y, current.position.x, Colors::Yellow);
}

int AStarPather::GetIndex(Position current)
{
	// Use a formula to get a spot in the array
	return (current.y * (terrain->get_map_width())) + current.x;
}

/////////////////////////////
// ARRAYS
/////////////////////////////

void AStarPather::PushNodeOpen(Node current)
{
	// The position of the node in the list
	int listPos = GetPosition(current.position);

	// Update this node with the new details
	theMap[listPos] = current

	// If it should be colored
	if (currentRequest.settings.debugColoring)
		ColorOpenNode(current);

}

void AStarPather::PushNodeClosed(Node current)
{
	// The position of the node in the list
	int listPos = GetPosition(current.position);

	// Update this node with the new details
	theMap[listPos] = current;

	// Set to the closed list and update the count
	theMap[listPos].estimateCost = static_cast<short>(NO_PARENT * SHORTIFY);

}

AStarPather::Node AStarPather::PopCheapest()
{
	// Store the position of the cheapest
	int cheapest = NO_ELEMENT;
	
	// For every node in the list
	for (int i = 0; i < theMap.size(); ++i)
	{
		// If it's on the closed list, don't count it
		if (theMap[i].estimateCost == NO_ELEMENT || theMap[i].estimateCost == NO_PARENT * SHORTIFY)
			continue;
	
		// If this is the first open node
		if (cheapest == NO_ELEMENT && theMap[i].estimateCost != NO_PARENT * SHORTIFY)
		{
			cheapest = i;
			continue;
		}
		// If the current node is cheaper than the old cheapest
		else if (theMap[i].TotalCost() < theMap[cheapest].TotalCost())
		{
			cheapest = i;
			continue;
		}
	
	}
	
	// If there was no element
	if (cheapest == NO_ELEMENT)
	{
		// Return a bad element
		return Node();
	}
	
	// Store the node
	Node cheapNode = theMap[cheapest];
	
	// Move to the closed list
	theMap[cheapest].estimateCost = static_cast<short>(NO_PARENT * SHORTIFY);
	
	// If it should be colored
	if (currentRequest.settings.debugColoring)
		ColorClosedNode(theMap[cheapest]);
	
	// We're done with the currentNode
	--openNodes;
	
	return cheapNode;
}


/////////////////////////////
// NODES
/////////////////////////////

short AStarPather::Node::TotalCost() const
{
	return givenCost + estimateCost;
}

AStarPather::Node::Node() : position(-1, -1), parent(-1, -1), givenCost(static_cast<short>(NO_PARENT * SHORTIFY)),
                                                              estimateCost(static_cast<short>(NO_PARENT * SHORTIFY))
{
}

bool AStarPather::Node::operator==(const Node & rhs)
{
	// If their position isn't the same
	if (this->position != rhs.position)
		return false;

	// They are the same
	return true;
}

AStarPather::Position::Position() : x(0), y(0)
{
}

AStarPather::Position::Position(int X, int Y) : x(X), y(Y)
{
}

AStarPather::Position::Position(GridPos position) : x(position.col), y(position.row)
{
}

bool AStarPather::Position::operator==(const Position & rhs)
{
	if (x != rhs.x)
		return false;
	if (y != rhs.y)
		return false;

	return true;
}

bool AStarPather::Position::operator!=(const Position & rhs)
{
	if (x == rhs.x)
		return false;
	if (y == rhs.y)
		return false;

	return true;
}
