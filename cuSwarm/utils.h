#ifndef UTILS_H
#define UTILS_H

#include <math.h>

/*******************
***** DEFINES ******
*******************/

#ifndef PI
#define PI 3.14159265358979323846f
#endif
#ifndef DEG2RAD
#define DEG2RAD 0.01745329251f
#endif
#ifndef BLOCK_SIZE
#define BLOCK_SIZE 256
#endif
#ifndef NORTH
#define NORTH 1.57079632679f
#endif
#ifndef EAST
#define EAST 0.0f
#endif
#ifndef SOUTH
#define SOUTH -1.57079632679f
#endif
#ifndef WEST
#define WEST 3.14159265358f
#endif

/***********************************
***** STRUCTURES AND TYPEDEFS ******
***********************************/

typedef unsigned int uint;
typedef unsigned long ulong;

struct Parameters
{
	float align_weight;
	float ang_bound;
	uint behavior;
	float cohere_weight;
	bool confirm_quit;
	float current;
	uint hops;
	uint leader_selection;
	bool log_data;
	float range_r;
	float range_f;
	float range_d;
	float range_o;
	float range_l;
	float range;
	uint max_explore;
	uint max_obstacle_size;
	float noise;
	uint num_obstacles;
	uint num_robots;
	uint point_size;
	float repel_weight;
	bool get_connectivity;
	bool get_ap;
	bool show_ap;
	bool highlight_leaders;
	bool show_connections;
	bool show_convex_hull;
	bool show_explored;
	bool show_leaders;
	bool show_range;
	bool show_range_leaders;
	bool show_non_leaders;
	float start_size;
	uint step_limit;
	bool training;
	uint targets;
	float vel_bound;
	uint window_height;
	uint window_width;
	uint world_size;
};

/**************************************
***** FORWARD DECLARED FUNCTIONS ******
**************************************/

float eucl2(float x1, float y1, float x2, float y2);

#endif
