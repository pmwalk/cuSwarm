#include "kernels.cuh"

/*********************
***** VARIABLES ******
*********************/

// Device pointers for simulation agent (d_positions, etc) and planning agent 
// (d_positions2, etc) data arrays
float4* d_positions;
float3* d_velocities;
int* d_modes;
int* d_leaders;
int* d_nearest_leader;
uint* d_leader_countdown;
int* d_laplacian;
bool* d_ap;

// Device pointers for environment variables
bool* d_occupancy;
float2* d_flow_pos;
float2* d_flow_dir;

// CUDA random states
curandState* d_rand_states;

// Kernel launch parameters
uint grid_dim;
dim3 block;
dim3 grid;

// CUDA streams
cudaStream_t streams[4];

/****************************
***** HELPER FUNCTIONS ******
****************************/

void cudaAllocate(Parameters p)
{
	// Allocate space on device for simulation arrays
	cudaMalloc(&d_positions, p.num_robots * sizeof(float4));
	cudaMalloc(&d_velocities, p.num_robots * sizeof(float3));
	cudaMalloc(&d_modes, p.num_robots * sizeof(int));
	cudaMalloc(&d_leaders, p.num_robots * sizeof(int));
	cudaMalloc(&d_nearest_leader, p.num_robots * sizeof(int));
	cudaMalloc(&d_leader_countdown, p.num_robots * sizeof(uint));
	cudaMalloc(&d_laplacian, p.num_robots * p.num_robots * sizeof(int));
	cudaMalloc(&d_ap, p.num_robots * sizeof(bool));
	// Allocate space on device for environment arrays
	cudaMalloc(&d_occupancy, p.world_size * 10 * p.world_size * 10 *
		sizeof(bool));
	cudaMalloc(&d_flow_pos, 256 * sizeof(float2));
	cudaMalloc(&d_flow_dir, 256 * sizeof(float2));
	// Allocate space on device for random state variables
	cudaMalloc(&d_rand_states, p.num_robots * sizeof(curandState));

	// Set kernel launch parameters
	grid_dim = static_cast<uint>(ceilf(static_cast<float>(p.num_robots) /
		BLOCK_SIZE));
	block = dim3(min(p.num_robots, BLOCK_SIZE), 1, 1);
	grid = dim3(grid_dim, 1, 1);

	// Create streams for simultaneous kernel launches
	cudaStreamCreate(&streams[0]);
	cudaStreamCreate(&streams[1]);
	cudaStreamCreate(&streams[2]);
	cudaStreamCreate(&streams[3]);
}

void cuFree()
{
	// Free arrays from simulation device memory
	cudaFree(d_positions);
	cudaFree(d_velocities);
	cudaFree(d_modes);
	cudaFree(d_leaders);
	cudaFree(d_nearest_leader);
	cudaFree(d_leader_countdown);
	cudaFree(d_laplacian);
	cudaFree(d_occupancy);
	cudaFree(d_flow_pos);
	cudaFree(d_flow_dir);
	cudaFree(d_rand_states);

	// Delete streams for simultaneous kernel launches
	cudaStreamDestroy(streams[0]);
	cudaStreamDestroy(streams[1]);
	cudaStreamDestroy(streams[2]);
	cudaStreamDestroy(streams[3]);
}

void launchInitKernel(Parameters p)
{
	// Run initialization kernel to load initial simulation state
	init_kernel << <grid, block >> >(
		d_positions,
		d_velocities,
		d_modes,
		d_rand_states,
		static_cast<ulong>(time(NULL)),
		d_flow_pos,
		d_flow_dir,
		d_nearest_leader,
		d_leader_countdown,
		p);
}

void launchInitKernel(Parameters p, struct cudaGraphicsResource **vbo_resource)
{
	// Map OpenGL buffer object for writing from CUDA
	cudaGraphicsMapResources(1, vbo_resource, 0);
	size_t num_bytes;
	cudaGraphicsResourceGetMappedPointer((void **)&d_positions, &num_bytes,
		*vbo_resource);

	// Run initialization kernel to load initial simulation state
	init_kernel << <grid, block >> >(
		d_positions,
		d_velocities,
		d_modes,
		d_rand_states,
		static_cast<ulong>(time(NULL)),
		d_flow_pos,
		d_flow_dir,
		d_nearest_leader,
		d_leader_countdown,
		p);

	// Unmap OpenGL buffer object
	cudaGraphicsUnmapResources(1, vbo_resource, 0);
}

void launchMainKernel(float3 gh, float2 gp, uint sn, int* leaders, bool* ap,
	Parameters p)
{
	// Copy leader and articulation point data to GPU
	cudaMemcpy(d_leaders, leaders, p.num_robots * sizeof(int),
		cudaMemcpyHostToDevice);
	cudaMemcpy(d_ap, ap, p.num_robots * sizeof(bool), cudaMemcpyHostToDevice);

	// Launch the main and side kernels
	main_kernel << <grid, block, 0, streams[0] >> >(
		d_positions,
		d_velocities,
		d_modes,
		gh,
		gp,
		d_rand_states,
		d_ap,
		d_flow_pos,
		d_flow_dir,
		d_occupancy,
		p,
		sn);

	// Run side kernel for extra computations outside the control loop
	side_kernel << <grid, block, 0, streams[1] >> >(
		d_positions,
		d_modes,
		d_leaders,
		d_rand_states,
		p,
		d_nearest_leader,
		d_leader_countdown,
		d_laplacian,
		sn);

	// Synchronize kernels on device
	cudaDeviceSynchronize();
}

void launchMainKernel(float3 gh, float2 gp, uint sn, int* leaders, bool* ap,
	Parameters p, struct cudaGraphicsResource **vbo_resource)
{
	// Map OpenGL buffer object for writing from CUDA
	cudaGraphicsMapResources(1, vbo_resource, 0);
	size_t num_bytes;
	cudaGraphicsResourceGetMappedPointer((void **)&d_positions, &num_bytes,
		*vbo_resource);

	launchMainKernel(gh, gp, sn, leaders, ap, p);

	// Unmap OpenGL buffer object
	cudaGraphicsUnmapResources(1, vbo_resource, 0);
}

void getData(uint n, float4* positions, float3* velocities, int* modes)
{
	// Copy simulation data from device to host arrays
	cudaMemcpy(positions, d_positions, n * sizeof(float4),
		cudaMemcpyDeviceToHost);
	cudaMemcpy(velocities, d_velocities, n * sizeof(float3),
		cudaMemcpyDeviceToHost);
	cudaMemcpy(modes, d_modes, n * sizeof(int), cudaMemcpyDeviceToHost);
}

void getData(uint n, float4* positions, float3* velocities, int* modes,
	int* nearest_leader, uint* leader_countdown)
{
	// Copy simulation data from device to host arrays
	cudaMemcpy(positions, d_positions, n * sizeof(float4),
		cudaMemcpyDeviceToHost);
	cudaMemcpy(velocities, d_velocities, n * sizeof(float3),
		cudaMemcpyDeviceToHost);
	cudaMemcpy(modes, d_modes, n * sizeof(int), cudaMemcpyDeviceToHost);
	cudaMemcpy(nearest_leader, d_nearest_leader, n * sizeof(int),
		cudaMemcpyDeviceToHost);
	cudaMemcpy(leader_countdown, d_leader_countdown, n * sizeof(uint),
		cudaMemcpyDeviceToHost);
}

void getLaplacian(uint n, int* laplacian)
{
	cudaMemcpy(laplacian, d_laplacian, n * n * sizeof(int),
		cudaMemcpyDeviceToHost);
}

void setData(uint n, float4* positions, float3* velocities, int* modes)
{
	// Copy simulation data from host to device arrays
	cudaMemcpy(d_positions, positions, n * sizeof(float4),
		cudaMemcpyHostToDevice);
	cudaMemcpy(d_velocities, velocities, n * sizeof(float3),
		cudaMemcpyHostToDevice);
	cudaMemcpy(d_modes, modes, n * sizeof(int), cudaMemcpyHostToDevice);
}

void setData(uint n, float4* positions, float3* velocities, int* modes,
	int* nearest_leader, uint* leader_countdown)
{
	// Copy simulation data from host to device arrays
	cudaMemcpy(d_positions, positions, n * sizeof(float4),
		cudaMemcpyHostToDevice);
	cudaMemcpy(d_velocities, velocities, n * sizeof(float3),
		cudaMemcpyHostToDevice);
	cudaMemcpy(d_modes, modes, n * sizeof(int), cudaMemcpyHostToDevice);
	cudaMemcpy(d_nearest_leader, nearest_leader, n * sizeof(int),
		cudaMemcpyHostToDevice);
	cudaMemcpy(d_leader_countdown, leader_countdown, n * sizeof(uint),
		cudaMemcpyHostToDevice);
}

void setOccupancy(Parameters p, bool* occupancy)
{
	// Copy occupancy data from host to device array
	cudaMemcpy(d_occupancy, occupancy, p.world_size * 10 * p.world_size * 10 *
		sizeof(bool), cudaMemcpyHostToDevice);
}

/**************************
***** CUDA FUNCTIONS ******
**************************/

__global__ void init_kernel(float4* pos, float3* vel, int* mode,
	curandState* rand_state, ulong seed, float2* flow_pos, float2* flow_dir,
	int* nearest_leader, uint* leader_countdown, Parameters p)
{
	// Index of this robot
	uint i = blockIdx.x * blockDim.x + threadIdx.x;

	// Frequently-used parameters
	float ws = static_cast<float>(p.world_size);

	// Seed the RNG
	curand_init(seed, i, 0, &rand_state[i]);
	curandState local_state = rand_state[i];

	// Initialize mode
	mode[i] = p.hops + 1;

	// Initialize nearest_leader and leader_countdown arrays for RCC leader 
	// selection
	nearest_leader[i] = -1;
	leader_countdown[i] = i;

	// Randomly place this robot within the starting circle
	float theta = curand_uniform(&local_state) * 2.0f * PI;
	float unit_r = curand_uniform(&local_state);
	float sqrt_unit_r = sqrtf(unit_r);
	float x_pos = p.start_size * sqrt_unit_r * cosf(theta);
	float y_pos = p.start_size * sqrt_unit_r * sinf(theta);

	// Set the initial color
	Color color;
	setColor(&(color.components), 1, false, i, p);

	// Set speed manually from params.txt
	float speed = p.vel_bound / 60.0f;
	// Set initial position, color and velocity
	pos[i] = make_float4(x_pos, y_pos, 0.0f, color.c);
	vel[i] = make_float3(0.0f, 0.0f, speed);

	// Assign a random flow to 256 points in the environment
	if (i < 256) {
		// First, set flow position
		float x_flow_pos = (curand_uniform(&local_state) * ws) - (ws / 2.0f);
		float y_flow_pos = (curand_uniform(&local_state) * ws) - (ws / 2.0f);
		flow_pos[i] = make_float2(x_flow_pos, y_flow_pos);

		// Second, set flow direction
		float x_flow_dir = (curand_uniform(&local_state) * 2.0f) - 1.0f;
		float y_flow_dir = (curand_uniform(&local_state) * 2.0f) - 1.0f;
		flow_dir[i] = make_float2(x_flow_dir, y_flow_dir);
	}
}

__global__ void side_kernel(float4* pos, int* mode, int* leaders,
	curandState* rand_state, Parameters p, int* nearest_leader,
	uint* leader_countdown, int* laplacian, uint sn)
{
	// Index of this robot
	uint i = blockIdx.x * blockDim.x + threadIdx.x;

	curandState local_state = rand_state[i];

	// Perform either RCC or CH leader assignment, depending on parameter
	if (p.leader_selection == 0) {

		// Holds the new mode and nearest leader during computation
		int my_mode = mode[i];
		int my_nearest_leader = nearest_leader[i];
		int my_leader_countdown = leader_countdown[i];

		// If the leader countdown for this robot has expired, switch leader 
		// state and reset the timer;
		if (my_leader_countdown == 0) {
			// Switch to a leader if not already; else switch to non-leader
			if (my_mode == 0) {
				my_mode = p.hops + 1;
				my_nearest_leader = -1;
				// Assign as non-leader for 1 second +/- (0-1/2) second
				leader_countdown[i] = 60 +
					static_cast<int>(curand_uniform(&local_state) * 30.0f);
			}
			if (my_mode > 0) {
				my_mode = 0;
				my_nearest_leader = i;
				// Assign as a leader for 2 seconds +/- (0-1) second
				leader_countdown[i] = 120 +
					static_cast<int>(curand_uniform(&local_state) * 60.0f);
			}
		}

		// Find the neighbor with the lowest mode (i.e. lowest hops to leader)
		int closest_n_mode = p.hops + 1;
		int closest_n_idx = INT_MAX;
		for (int n = 0; n < p.num_robots; n++) {
			// Get the distance between the robots
			float4 me = pos[i];
			float4 them = pos[n];
			float2 dist = make_float2(me.x - them.x, me.y - them.y);
			// Determine if these robots are within range
			bool within_range = euclidean(dist) < (p.range_l);
			// Exclude cases where neighbor and current robot are the same, 
			// or where the robot is out of range
			if (i != n && within_range) {
				closest_n_mode = min(closest_n_mode, mode[n]);
				closest_n_idx = min(closest_n_idx, n);
			}
		}

		// Update mode if not a leader
		if (my_mode != 0) {
			// Set new mode for this robot
			my_mode = closest_n_mode + 1;
			if (closest_n_mode < p.hops) {
				// Reset leader timer to 1 second +/- (0-1/2) seconds
				leader_countdown[i] = 60 +
					static_cast<int>(curand_uniform(&local_state) *
					30.0f);
			}
		}

		// Synchronize threads before updating mode and neighbor arrays
		__syncthreads();

		// Update this robot's mode and nearest leader
		mode[i] = my_mode;
		nearest_leader[i] = my_nearest_leader;

		// Decrease countdown timer
		leader_countdown[i]--;
	}
	else if (p.leader_selection == 1) {
		// Look at the index of this robot in the leader list to determine if 
		// this robot should be a leader
		if (leaders[i] == 0) {
			mode[i] = 0;
		}
		else {
			mode[i] = 1;
		}
	}

	// Degree for laplacian
	uint degree = 0;
	// Iterate through all other robots
	for (uint j = 0; j < p.num_robots; j++) {
		if (i != j) {
			// Get the distance between the robots
			float4 me = pos[i];
			float4 them = pos[j];
			float2 dist_xy = make_float2(me.x - them.x, me.y - them.y);
			float dist = euclidean(dist_xy);
			// Set the non-diagonal values based on whether robots are connected
			// at the four different ranges, -1 means connected, 0 disconnected
			if (dist < p.range) {
				degree++;
				laplacian[(i * p.num_robots) + j] = -1;
				laplacian[(j * p.num_robots) + i] = -1;
			}
			else {
				laplacian[(i * p.num_robots) + j] = 0;
				laplacian[(j * p.num_robots) + i] = 0;
			}
		}
	}
	// Set the diagonal of the laplacian to the degree of corresponding robot
	laplacian[(i * p.num_robots) + i] = degree;
}

__global__ void main_kernel(float4* pos, float3* vel, int* mode,
	float3 goal_heading, float2 goal_point, curandState* rand_state, bool* ap,
	float2* flow_pos, float2* flow_dir, bool* occupancy, Parameters p, uint sn)
{
	// Index of this robot
	uint i = blockIdx.x * blockDim.x + threadIdx.x;

	__shared__ float4 s_pos[BLOCK_SIZE];
	__shared__ float3 s_vel[BLOCK_SIZE];
	__shared__ int s_mode[BLOCK_SIZE];
	__shared__ bool s_ap[BLOCK_SIZE];

	__syncthreads();

	// Variables for this robot's data
	float4 myPos = pos[i];
	int myMode = mode[i];
	float mySpeed = p.vel_bound / 60.0f;
	float dist_to_obstacle = p.range_o;
	curandState local_state = rand_state[i];

	// Computation variable initializations
	float2 min_bounds = make_float2(0.0f, 0.0f);
	float2 max_bounds = make_float2(0.0f, 0.0f);
	float2 repel = make_float2(0.0f, 0.0f);
	float2 align = make_float2(0.0f, 0.0f);
	float2 cohere = make_float2(0.0f, 0.0f);
	float2 avoid = make_float2(0.0f, 0.0f);
	float2 flow = make_float2(0.0f, 0.0f);
	float2 goal = make_float2(0.0f, 0.0f);

	// If we are flocking and a leader, set the alignment vector towards 
	// the goal point
	if (p.behavior == 1 && myMode == 0) {
		align.x = goal_heading.x;
		align.y = goal_heading.y;
	}

	// Iterate through blocks to use shared memory within a block
	for (uint tile = 0; tile < gridDim.x; tile++) {

		// Assign shared memory for this block
		uint n = tile * blockDim.x + threadIdx.x;
		s_pos[threadIdx.x] = pos[n];
		s_vel[threadIdx.x] = vel[n];
		s_mode[threadIdx.x] = mode[n];
		s_ap[threadIdx.x] = ap[n];

		// Synchronize threads after shared memory is assigned
		__syncthreads();

		// Iterate through all threads in this block
		for (uint ti = 0; ti < blockDim.x; ti++) {
			// Do not perform an interaction between this robot and itself
			if (i != tile * blockDim.x + ti) {

				// Calculate the distance between the two robots on all axes
				float dist_x = s_pos[ti].x - myPos.x;
				float dist_y = s_pos[ti].y - myPos.y;
				// Calculate the Euclidean distance between the two robots
				float dist = euclidean(make_float2(dist_x, dist_y));

				// Perform interaction for neighbors within range
				if (dist <= p.range) {

					// Create collecte distance variable (for readability)
					float3 dist3 = make_float3(dist_x, dist_y, dist);

					// Perform the interaction for this robot pair based on 
					// the current behavior
					switch (p.behavior) {
					case 0:
						rendezvous(dist3, &min_bounds, &max_bounds, &repel,
							s_ap[ti], p);
						break;
					case 1:
						flock(myMode, s_vel[ti], s_mode[ti], dist3, &repel,
							&align, &cohere, s_ap[ti], p);
						break;
					case 2:
						disperse(dist3, &repel, &cohere, s_ap[ti], p);
						break;
					case 3:
						rendezvousToPoint(dist3, &repel, p);
						break;
					}
				}
			}
		}
	}

	// Perform obstacle avoidance computation for this robot
	obstacleAvoidance(myPos, &avoid, &dist_to_obstacle, occupancy, p);

	// Finish necessary summary computations for each behavior
	switch (p.behavior) {
	case 0:
		// Finish computation of parallel circumcenter algorithm
		cohere.x = ((min_bounds.x + max_bounds.x) / 2.0f);
		cohere.y = ((min_bounds.y + max_bounds.y) / 2.0f);
		break;
	case 1:
		break;
	case 2:
		break;
	case 3:
		// Set align vector to point toward goal point
		float align_angle = atan2f(goal_point.y - myPos.y,
			goal_point.x - myPos.x);
		align.x = cosf(align_angle);
		align.y = sinf(align_angle);
		break;
	}

	// If velocity is affected by random flows, calculate flow effect here
	if (p.current > 0.0f) {
		for (uint fi = 0; fi < 256; fi++) {
			float2 dist_v = make_float2(myPos.x - flow_pos[fi].x,
				myPos.y - flow_pos[fi].y);
			float dist = euclidean(dist_v);
			flow.x += flow_dir[fi].x * (1.0f / dist);
			flow.y += flow_dir[fi].y * (1.0f / dist);
		}
	}

	// Scale all component vectors to their weights and compute goal vector
	rescale(&repel, p.repel_weight, false);
	rescale(&align, p.align_weight, false);
	rescale(&cohere, p.cohere_weight, false);
	rescale(&avoid, 4.0f * powf((p.range_o - dist_to_obstacle), 4.0f), false);
	// Add random currents, if applicable
	if (p.current > 0.0f) {
		rescale(&flow, p.current, false);
	}

	// Combine behavior components to make the new goal vector
	goal.x = repel.x + align.x + cohere.x + avoid.x + flow.x;
	goal.y = repel.y + align.y + cohere.y + avoid.y + flow.y;

	// Cap the angular velocity
	capAngularVelocity(make_float2(vel[i].x, vel[i].y),
		&goal, p.ang_bound / 60.0f);
	// Rescale the goal to the calculated speed
	rescale(&goal, mySpeed, true);

	// Synchronize threads before updating robot state variables
	__syncthreads();

	// Set the color based on current mode
	Color color;
	setColor(&(color.components), myMode, ap[i], i, p);
	// Update velocity
	vel[i] = make_float3(goal.x, goal.y, mySpeed);

	// Update position
	pos[i] = make_float4(myPos.x + vel[i].x, myPos.y + vel[i].y, 0.0f, color.c);

	// Update random state for CUDA RNG
	rand_state[i] = local_state;
}

__device__ void rendezvous(float3 dist3, float2* min_bounds, float2* max_bounds,
	float2* repel, bool is_ap, Parameters p)
{
	if (dist3.z <= p.range_r) {
		// REPEL
		// Repel from robots within repel range
		float weight = powf(p.range_r - dist3.z, 2.0f);
		repel->x -= weight * dist3.x;
		repel->y -= weight * dist3.y;
	}
	if (dist3.z <= p.range && dist3.z > p.range_r)
	{
		// COHERE
		// Robots cohere to the center of the rectangle that bounds neighbors
		// Do not cohere to neighbors within repel range
		min_bounds->x = fminf(min_bounds->x, dist3.x);
		min_bounds->y = fminf(min_bounds->y, dist3.y);
		max_bounds->x = fmaxf(max_bounds->x, dist3.x);
		max_bounds->y = fmaxf(max_bounds->y, dist3.y);
	}
}

__device__ void flock(int myMode, float3 nVel, int nMode, float3 dist3,
	float2* repel, float2* align, float2* cohere, bool is_ap, Parameters p)
{
	// Main flocking section
	if (dist3.z <= p.range) {
		// REPEL
		// Robots repel from neighbors within flocking repel range
		float weight = powf(p.range - dist3.z, 4.0f);
		repel->x -= weight * dist3.x;
		repel->y -= weight * dist3.y;
	}
	if (myMode != 0) {
		// ALIGN
		float weight;
		(nMode == 0) ? weight = 1000.0f : weight = 1.0f;
		align->x += weight * nVel.x;
		align->y += weight * nVel.y;
	}
	if (dist3.z < p.range) {
		// COHERE
		// Do not cohere to neighbors within repel range
		float weight = powf(dist3.z, 4.0f);
		//(is_ap) ? weight = 1000.0f : weight *= 1.0f;
		cohere->x += weight * dist3.x;
		cohere->y += weight * dist3.y;
	}
}

__device__ void disperse(float3 dist3, float2* repel, float2* cohere, bool is_ap,
	Parameters p)
{
	// Determine whether we should repel or cohere based on the 
	// distance to the neighbor
	if (dist3.z <= p.range_d) {
		// REPEL
		// Robots repel from neighbors within disperse repel range
		float weight = powf(p.range_d - dist3.z, 2.0f);
		repel->x -= weight * dist3.x;
		repel->y -= weight * dist3.y;
	}
	if (dist3.z <= p.range && dist3.z > p.range_d) {
		// COHERE
		// Do not cohere to robots within disperse repel range
		float weight = powf(dist3.z - p.range_d, 3.0f);
		cohere->x += weight * dist3.x;
		cohere->y += weight * dist3.y;
	}
}

__device__ void rendezvousToPoint(float3 dist3, float2* repel, Parameters p)
{
	if (dist3.z <= p.range_r) {
		// REPEL
		// Repel from robots within repel range
		float weight = powf(p.range_r - dist3.z, 2.0f);
		repel->x -= weight * dist3.x;
		repel->y -= weight * dist3.y;
	}
}

__device__ void obstacleAvoidance(float4 myPos, float2* avoid,
	float* dist_to_obstacle, bool* occupancy, Parameters p)
{
	*(dist_to_obstacle) = FLT_MAX;
	int count = 0;
	for (float i = 0; i < 2.0f * PI; i += PI / 45.0f) {
		float cos = cosf(i);
		float sin = sinf(i);
		// Ray trace along this angle up to the robot's avoidance range
		for (float r = 0.0f; r < p.range_o; r += 1.0f) {
			count++;
			float x_check = myPos.x + r * cos;
			float y_check = myPos.y + r * sin;
			// If this point contains an obstacle, add the corresponding vector 
			// component to the obstacle vector
			if (checkOccupancy(x_check, y_check, occupancy, p)) {
				// Get weight for obstacle repulsion force
				float weight = powf(1.0f - (r / p.range_o), 2.0f);
				// Update the distance to the closest obstacle
				if (r < *dist_to_obstacle) {
					*dist_to_obstacle = r;
				}
				// Update the obstacle avoidance vector
				avoid->x += weight * -r * cos;
				avoid->y += weight * -r * sin;
				break;
			}
		}
	}
}

__device__ bool checkOccupancy(float x, float y, bool* occupancy, Parameters p)
{
	float ws_2 = static_cast<float>(p.world_size) / 2.0f;
	float ws_10 = static_cast<float>(p.world_size) * 10.0f;
	// Return false if the coordinates to check are outside the world boundaries; 
	// else, return the occupancy grid value for these coordinates
	if (x < -ws_2 || x > ws_2 || y < -ws_2 || y > ws_2) {
		return true;
	}
	else {
		// Get the 1d index for the occupancy array from the x and y coordinates
		uint x_component = static_cast<uint>((x + ws_2) * 10.0f);
		uint y_component = static_cast<uint>(floorf(y + ws_2) * ws_10 * 10.0f);
		uint index = x_component + y_component;
		return occupancy[index];
	}
}

__device__ void setColor(uchar4* color, int mode, bool is_ap, uint i,
	Parameters p)
{
	if (p.show_leaders_only && mode == 0 || !p.show_leaders_only) {
		if (p.show_mode) {
			switch (mode) {
			case 0:
				*color = make_uchar4(255, 0, 0, 255);
				break;
			case 1:
				*color = make_uchar4(0, 200, 0, 255);
				break;
			case 2:
				*color = make_uchar4(100, 100, 255, 255);
				break;
			case 3:
				*color = make_uchar4(200, 200, 50, 255);
				break;
			case 4:
				*color = make_uchar4(50, 200, 200, 255);
				break;
			default:
				*color = make_uchar4(100, 100, 100, 255);
				break;
			}
		}
		else {
			*color = make_uchar4(255, 255, 255, 255);
		}
	}
	else {
		*color = make_uchar4(0, 0, 0, 0);
	}
}

__device__ float euclidean(float2 vector)
{
	return sqrtf(powf(vector.x, 2.0f) + powf(vector.y, 2.0f));
}

__device__ void rescale(float2* vel, float value, bool is_value_limit)
{
	// Determine the scalar of the vector
	float scalar = euclidean(*vel);

	// Normalize the vector if the value given is the limit and the scalar is  
	// above that, or if the value given is not a limit
	if (scalar != 0.0f &&
		((is_value_limit && scalar > value) || !is_value_limit)) {
		float factor = value / scalar;
		vel->x *= factor;
		vel->y *= factor;
	}
}

__device__ void normalizeAngle(float* angle)
{
	while (*angle > PI) {
		*angle -= 2.0f * PI;
	}
	while (*angle <= -PI) {
		*angle += 2.0f * PI;
	}
}

__device__ void capAngularVelocity(float2 old, float2* goal, float max)
{
	// Get the magnitude of each vector
	float norm_old = euclidean(old);
	float norm_goal = euclidean(*goal);

	float old_angle = atan2f(old.y, old.x);
	float goal_angle = atan2f(goal->y, goal->x);

	// Get the angle from the old to new goal vector
	float angle = goal_angle - old_angle;
	normalizeAngle(&angle);

	// If the angle is greater than the maximum angular velocity, cap the 
	// angular velocity to this maximum
	if (fabsf(angle) > max && norm_old != 0.0f && norm_goal != 0.0f) {
		(angle < 0.0f) ? old_angle -= max : old_angle += max;
		goal->x = norm_goal * cosf(old_angle);
		goal->y = norm_goal * sinf(old_angle);
	}
}
