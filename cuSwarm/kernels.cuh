#ifndef KERNELS_H
#define KERNELS_H

/********************
***** INCLUDES ******
********************/

// System includes
#include <algorithm>
#include <iostream>

// Cuda includes
#include <cuda.h>
#include <cuda_runtime.h>
#include <curand.h>
#include <curand_kernel.h>
#include <vector_types.h>
#include "device_launch_parameters.h"

// OpenGL includes
#include <gl/glew.h>
#include <gl/freeglut.h>
#include <cuda_gl_interop.h>

// Project includes
#include "utils.h"

/***********************************
***** STRUCTURES AND TYPEDEFS ******
***********************************/

union Color
{
	float c;
	uchar4 components;
};

/**************************************
***** FORWARD DECLARED FUNCTIONS ******
**************************************/

// CUDA memory functions
void cudaAllocate(Parameters p);
void cuFree();

// Kernel launches
void launchInitKernel(Parameters p, struct cudaGraphicsResource **vbo_resource);
void launchMainKernel(float3 gh, float2 gp, uint sn, int* leaders, bool* ap, 
	Parameters p, 
struct cudaGraphicsResource **vbo_resource);
void launchInitKernel(Parameters p);
void launchMainKernel(float3 gh, float2 gp, uint sn, int* leaders, bool* ap, 
	Parameters p);

// CUDA host<->device copy functions
void getData(uint n, float4* positions, float3* velocities, int* modes);
void getData(uint n, float4* positions, float3* velocities, int* modes, 
	int* nearest_leader, uint* leader_countdown);
void getLaplacian(uint n, int* laplacian);
void setData(uint n, float4* positions, float3* velocities, int* modes);
void setData(uint n, float4* positions, float3* velocities, int* modes,
	int* nearest_leader, uint* leader_countdown);
void setOccupancy(Parameters p, bool* occupancy);

/*********************************************
***** FORWARD DECLARED DEVICE FUNCTIONS ******
*********************************************/

__global__ void init_kernel(float4* pos, float3* vel, int* mode, 
	curandState* rand_state, ulong seed, float2* flow_pos, float2* flow_dir, 
	int* nearest_leader, uint* leader_countdown, Parameters p);

__global__ void side_kernel(float4* pos, int* mode, int* leaders, 
	curandState* rand_state, Parameters p, int* nearest_leader, 
	uint* leader_countdown, int* laplacian, uint sn);

__global__ void main_kernel(float4* pos, float3* vel, int* mode, 
	float3 goal_heading, float2 goal_point, curandState* rand_state, bool* ap, 
	float2* flow_pos, float2* flor_dir, bool* occupancy, Parameters p, uint sn);

__device__ void rendezvous(float3 dist3, float2* min_bounds, float2* max_bounds, 
	float2* repel, bool is_ap, Parameters p);

__device__ void flock(int myMode, float3 nVel, int nMode, float3 dist3, 
	float2* repel, float2* align, float2* cohere, bool is_ap, Parameters p);

__device__ void disperse(float3 dist3, float2* repel, float2* cohere, bool is_ap, 
	Parameters p);

__device__ void rendezvousToPoint(float3 dist3, float2* repel, Parameters p);

__device__ void obstacleAvoidance(float4 myPos, float2* avoid, 
	float* dist_to_obstacle, bool* occupancy, Parameters p);

__device__ bool checkOccupancy(float x, float y, bool* occupancy, Parameters p);

__device__ void setColor(uchar4* color, int mode, bool is_ap, uint i, 
	Parameters p);

__device__ float euclidean(float2 vector);

__device__ void rescale(float2* vel, float value, bool is_value_limit);

__device__ void normalizeAngle(float* angle);

__device__ void capAngularVelocity(float2 old, float2* goal, float max);

#endif
