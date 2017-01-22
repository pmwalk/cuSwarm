#ifndef DATA_OPS_H
#define DATA_OPS_H

// System includes
#include <vector>

// External library includes
#include <Eigen/Dense>

// Project includes
#include "kernels.cuh"

using namespace std;

/***********************************
***** STRUCTURES AND TYPEDEFS ******
***********************************/

struct Point {
	float x, y;

	bool operator <(const Point &p) const {
		return x < p.x || (x == p.x && y < p.y);
	}
};

struct Data
{
	float2 centroid;
	float4 bounds;
	float score;
	float heading_avg;
	float heading_var;
	vector<Point> ch;
	float ch_area;
	int explored;
	int targets_explored;
	int targets_seen;
	int goals_reached;
	float connectivity;
};

/**********************
***** GRAPH CLASS *****
**********************/

class Graph {
private:
	// V is the number of vertices in the graph
	// time is used to determine when a node has a back-link to an ancestor
	int V, time;

	// adjList[u] is the adjacency list of vertex u, 0 <= u < V
	vector <int> *adjList;

	// explored[u] is true if u has been explored
	// articulation_point[u] is true is u is an articulation point
	bool *explored, *articulation_point, done;

	// disc_time[u] is the time at which vertex u was explored
	// parent[u] = v if in the dfs tree, there is an edge from v to u
	// low[u] is the time of the earliest explored vertex reachable from u
	// If low[u] < disc_time[u], then there is a back-link from some node in the 
	// subtree rooted at u to some ancestor of u
	int *disc_time, *parent, *low;

	// articulation_points stores the articulation points/cut vertices of graph
	vector <int> articulation_points;

	// Depth first search methods
	void dfsUtil(int u);
	void dfs();

public:
	// create an empty undirected graph having V vertices
	Graph(int V);
	~Graph();

	// add an undirected edge (u, v) to the graph
	// returns false if either u or v is less than 0 or greater than equal to V
	// returns true if the edge was added to the digraph
	bool addEdge(int u, int v);

	// Performs dfs over the graph and returns a vector containing
	// the articulation points
	vector<int> getArticulationPoints();
};

/************************************
**** FORWARD DECLARED FUNCTIONS******
************************************/

// Data processing main function
void processData(float4* positions, float3* velocities, int* explored_grid, int* laplacian, bool* ap, Data* data, Parameters p);

// Convex hull functions
void convexHull(float4* pos, vector<Point>* points, uint n);
float2 convexHullCentroid(vector<float4> points);
float convexHullArea(vector<Point> points);
float cross(const Point &O, const Point &A, const Point &B);

// Eigenvalue functions
float connectivity(uint n, int* laplacian);
void articulationPoints(uint n, int* laplacian, bool* ap);

#endif
