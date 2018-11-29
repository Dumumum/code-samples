#pragma once
#include "Misc/PathfindingDetails.hpp"

#define MAX_NODES 1600  // Maximum number of nodes in the map

class AStarPather
{
public:

	// FUNCTIONS
	
    bool initialize();
    void shutdown();
    PathResult compute_path(PathRequest &request);

	// Represents a position on the grid
	struct Position
	{
		// VARIABLES
		
		signed char x;
		signed char y;

		// FUNCTIONS
		
		Position();
		Position(int X, int Y);
		Position(GridPos position);

		bool operator==(const Position &rhs);
		bool operator!=(const Position &rhs);
	};

	// The pathfinding data about any given position
	struct Node
	{
		// VARIABLES
		
		Position parent;     // Position that this node came from
		Position position;   // Position of current node
		short givenCost;     // Cost from start node
		short estimateCost;  // Cost determined from method

		// FUNCTIONS
		
		Node();                            // A bad node
		
		short TotalCost() const;           // Function to calculate total cost of node
		bool operator==(const Node &rhs);  // Used for removing a node
	};

	// For every grid position, tracks it's availability to it's neighbors
	struct Neighbor
	{
		// VARIABLES
		
		bool bottomLeft;
		bool bottom;
		bool bottomRight;
		bool right;
		bool left;
		bool topLeft;
		bool top;
		bool topRight;
	};

	// STATICS
	
	static std::array<AStarPather::Neighbor, MAX_NODES> neighborList;     // Holds all preprocessed neighbors
	static std::array<AStarPather::Node, MAX_NODES> AStarPather::theMap;  // Holds all possible nodes
	static PathRequest AStarPather::currentRequest;                       // The current request

private:

	// VARIABLES
	
	int openNodes = 0;  // Holds the size of the open list

	// FUNCTIONS

	// Nodes
	Node CreateNode(Position nodePosition, Node parent);  // Creates a node for the lists given a point and parent
	Node CreatePosition(Position position);               // Use this node to find a node at a position
	int GetPosition(Position position);                   // Gets node based on position
	int GetNode(Node current);                            // Gets the node inside of the list

	// Estimates
	short GetEstimate(Position begin, Position end);  // Calculates the estimate based on the heuristic
	short Octile(Position begin, Position end);       // Octile heuristic
	short Chebyshev(Position begin, Position end);    //Chebyshev heuristic
	short Manhattan(Position begin, Position end);    // Manhattan heuristic
	short Euclidean(Position begin, Position end);    // Euclidean heuristic

	// Algorithm
	void CalculateNeighbors();                            // Preprocesses all neighbors
	std::list<Node> GetNeighbors(Node current);           // Finds all the possible neighbors to the node
	void CreatePath(int goalNode, PathRequest &request);  // Adds the correct nodes to the calculated path
	void Rubberband(WaypointList &path);                  // Add rubberbanding to the path
	void Smooth(WaypointList &path);                      // Add smoothing (splines) to the path

	// Debug
	void ColorOpenNode(Node current);    // Adds the color representation
	void ColorClosedNode(Node current);  // Adds the color representation

	// Arrays
	int GetIndex(Position current);     // Gets the index position of a node's position
	void PushNodeOpen(Node current);    // Puts a node on the open list
	void PushNodeClosed(Node current);  // Puts a node on the closed list
	Node PopCheapest();                 // Gets the cheapest open node and pops it
	
};
