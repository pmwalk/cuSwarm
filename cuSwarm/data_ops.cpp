#include "data_ops.h"

/////////////////////////
///// Graph Methods /////
/////////////////////////

Graph::Graph(int V)
{
	this->V = V;
	time = 0;
	done = false;

	adjList = new vector <int>[V];
	explored = new bool[V];
	articulation_point = new bool[V];
	disc_time = new int[V];
	parent = new int[V];
	low = new int[V];

	memset(explored, false, V * sizeof(bool));
	memset(articulation_point, false, V * sizeof(bool));
	memset(parent, -1, V * sizeof(int));
}

Graph::~Graph()
{
	delete[] adjList;
	delete[] articulation_point;
	delete[] explored;
	delete[] parent;
	delete[] disc_time;
	delete[] low;
}

bool Graph::addEdge(int u, int v)
{
	if (u < 0 || u >= V) return false;
	if (v < 0 || v >= V) return false;
	adjList[u].push_back(v);
	adjList[v].push_back(u);
	return true;
}

vector<int> Graph::getArticulationPoints()
{
	if (done)
		return articulation_points;
	dfs();
	done = true;
	for (int u = 0; u < V; u++)
		if (articulation_point[u])
			articulation_points.push_back(u);
	return articulation_points;
}

void Graph::dfsUtil(int u)
{
	explored[u] = true;
	int num_child = 0;
	disc_time[u] = low[u] = ++time;

	for (vector <int>::iterator v = adjList[u].begin();
		v != adjList[u].end(); v++)	{
		if (!explored[*v])	{
			num_child++;
			parent[*v] = u;
			dfsUtil(*v);
			low[u] = min(low[u], low[*v]);

			// u is an articulation point iff
			// 1. It the the root and has more than 1 child
			// 2. It is not the root and no vertex in the subtree rooted at 
			//    one of its children has a back-link to its ancestor.
			//    A child has a back-link to an ancestor of its parent when 
			//    its low value is less than the discovery time of its parent
			if (parent[u] == -1 && num_child > 1)
				articulation_point[u] = true;
			else if (parent[u] != -1 && low[*v] >= disc_time[u])
				articulation_point[u] = true;
		}
		else if (*v != parent[u])
			low[u] = min(low[u], disc_time[*v]);
	}
}

void Graph::dfs()
{
	for (int u = 0; u < V; u++)
		if (!explored[u])
			dfsUtil(u);
}

////////////////////////////////
///// Main Data Processing /////
////////////////////////////////

void processData(float4* positions, float3* velocities, int* explored_grid, 
	int* laplacian, bool* ap, Data* data, Parameters p)
{
	///// HEADING AVERAGE /////
	float nf = static_cast<float>(p.num_robots);
	// Get the average velocity
	float2 avg_vel = make_float2(0.0f, 0.0f);
	for (uint i = 0; i < p.num_robots; i++) {
		// Get heading for this robot
		float h = atan2(velocities[i].y, velocities[i].x);
		avg_vel.x += cos(h);
		avg_vel.y += sin(h);
	}

	// Finish average
	avg_vel.x /= nf;
	avg_vel.y /= nf;

	float avg_h = atan2(avg_vel.y, avg_vel.x);
	data->heading_avg = avg_h;

	///// HEADING VARIANCE /////
	float var = 0.0f;
	for (uint i = 0; i < p.num_robots; i++) {
		float h = atan2(velocities[i].y, velocities[i].x);
		float diff = abs(avg_h - h);
		while (diff > PI) {
			diff -= static_cast<float>(2.0 * PI);
		}
		var += abs(diff);
	}

	data->heading_var = var;

	///// CENTROID /////
	for (uint i = 0; i < p.num_robots; i++) {
		data->centroid.x += positions[i].x;
		data->centroid.y += positions[i].y;
	}

	data->centroid.x = data->centroid.x / nf;
	data->centroid.y = data->centroid.y / nf;

	///// POSITION BOUNDS and VARIANCE /////
	// Initialize position bounds to minima/maxima
	data->bounds.x = FLT_MAX;
	data->bounds.y = -FLT_MAX;
	data->bounds.z = FLT_MAX;
	data->bounds.w = -FLT_MAX;

	for (uint i = 0; i < p.num_robots; i++) {
		// Check bounds of the swarm
		data->bounds.x = min(data->bounds.x, positions[i].x);
		data->bounds.y = max(data->bounds.y, positions[i].x);
		data->bounds.z = min(data->bounds.z, positions[i].y);
		data->bounds.w = max(data->bounds.w, positions[i].y);
	}

	///// CONVEX HULL /////
	// Coordinates of convex hull vertices (stored in data object)
	data->ch.clear();
	// Compute convex hull of swarm
	convexHull(positions, &(data->ch), p.num_robots);
	// Get the area of the convex hull
	data->ch_area = convexHullArea(data->ch);

	///// EXPLORED AREA /////
	int explored = 0;
	for (uint i = 0; i < p.world_size * p.world_size; i++) {
		explored += abs(explored_grid[i]);
	}
	data->explored = explored;
	// Score is the explored area plus a bonus for each target fully explored
	data->score = static_cast<float>(data->explored);

	getLaplacian(p.num_robots, laplacian);
	if (p.get_ap) {
		articulationPoints(p.num_robots, laplacian, ap);
	}
	(p.get_connectivity) ?
		data->connectivity = connectivity(p.num_robots, laplacian) :
		data->connectivity = 0.0f;
}

/*********************************
***** CONVEX HULL FUNCTIONS ******
*********************************/

// Returns a list of points on the convex hull in counter-clockwise order.
// Note: the last point in the returned list is the same as the first one.
void convexHull(float4* pos, vector<Point>* points, uint num)
{
	int n = static_cast<int>(num);
	int k = 0;
	vector<Point> P;
	for (int i = 0; i < n; i++) {
		Point p_temp;
		p_temp.x = pos[i].x;
		p_temp.y = pos[i].y;
		P.push_back(p_temp);
	}

	// Convex hull vector
	vector<Point> ch(2 * n);

	// Sort points lexicographically
	sort(P.begin(), P.end());

	// Build lower hull
	for (int i = 0; i < n; i++) {
		while (k >= 2 && cross(ch[k - 2], ch[k - 1], P[i]) <= 0) k--;
		ch[k++] = P[i];
	}

	// Build upper hull
	for (int i = n - 2, t = k + 1; i >= 0; i--) {
		while (k >= t && cross(ch[k - 2], ch[k - 1], P[i]) <= 0) k--;
		ch[k++] = P[i];
	}

	// Resize the convex hull point array
	ch.resize(k);

	// Copy ch to points list supplied in arguments
	for (uint i = 0; i < ch.size(); i++) {
		Point temp_point;
		temp_point.x = ch[i].x;
		temp_point.y = ch[i].y;
		points->push_back(temp_point);
	}
}

// Compute the centroid (x, y) of the convex hull given by points
float2 convexHullCentroid(vector<float4> points)
{
	float min_x = FLT_MAX, max_x = -FLT_MAX;
	float min_y = FLT_MAX, max_y = -FLT_MAX;
	for (unsigned int i = 0; i < points.size(); i++) {
		if (min_x > points[i].x) {
			min_x = points[i].x;
		}
		else if (max_x < points[i].x) {
			max_x = points[i].x;
		}
		if (min_y > points[i].y) {
			min_y = points[i].y;
		}
		else if (max_y < points[i].y) {
			max_y = points[i].y;
		}
	}
	float2 to_return = make_float2((min_x + max_x) / 2.0f, 
		(min_y + max_y) / 2.0f);
	return to_return;
}

// Compute the area of the convex hull
float convexHullArea(vector<Point> points)
{
	float area = 0.0f;
	for (int a = 0; static_cast<uint>(a) < points.size(); a++) {
		int b = ((a + 1) % points.size());
		float temp = (points[a].x * points[b].y) - (points[b].x * points[a].y);
		area += temp;
	}
	return area;
}

// 2D cross product of OA and OB vectors, i.e. z-component of their 3D cross 
// product.
// Returns a positive value, if OAB makes a counter-clockwise turn, negative for 
// clockwise turn, and zero if the points are collinear.
float cross(const Point &O, const Point &A, const Point &B)
{
	return (float)(A.x - O.x) * (B.y - O.y) - (float)(A.y - O.y) * (B.x - O.x);
}

// Convert the float4 array to a vector of points
vector<Point> float4toPointArray(float4* points, uint n) {
	vector<Point> vector_points;
	for (uint i = 0; i < n; i++) {
		Point p;
		p.x = points[i].x;
		p.y = points[i].y;
		vector_points.push_back(p);
	}
	// Return the new vector of points
	return vector_points;
}

float connectivity(uint n, int* laplacian)
{
	// Laplacian matrix in Eigen form
	Eigen::MatrixXf A(n, n);
	// Populate Eigen matrix
	for (uint i = 0; i < n; i++) {
		for (uint j = 0; j < n; j++) {
			// Get connectivity based on robot range
			A(i, j) = static_cast<float>(laplacian[(i * n) + j]);
		}
	}

	// Get eigenvalues of laplacian
	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> es(A, true);

	// Return second-smallest eigenvalue
	return es.eigenvalues()[1];
}

void articulationPoints(uint n, int* laplacian, bool* ap)
{
	// Clear the articulation point vector
	for (uint i = 0; i < n; i++) {
		ap[i] = false;
	}

	// Graph object
	Graph g(n);

	// Create graph
	for (uint i = 0; i < n; i++) {
		for (uint j = i + 1; j < n; j++) {
			bool connected;
			// Compute articulation points based on robot range
			connected = laplacian[(i * n) + j] == -1;
			if (connected) {
				g.addEdge(i, j);
			}
		}
	}

	// Get articulation points of graph
	vector<int> points = g.getArticulationPoints();
	for (uint i = 0; i < points.size(); i++) {
		ap[points[i]] = true;
	}
}
