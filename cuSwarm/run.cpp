#include "run.h"

/*************************************
***** OpenGL Callback Functions ******
*************************************/

void drawInterface(float window_width, float window_height)
{
	// Draw edge only polygons (wire frame)
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
	glLineWidth(2.0f);

	// Draw world boundaries
	float world_size_2 = p.world_size / 2.0f;
	glBegin(GL_POLYGON);
	glVertex3f(-world_size_2, -world_size_2, 0.0f);
	glVertex3f(-world_size_2, world_size_2, 0.0f);
	glVertex3f(world_size_2, world_size_2, 0.0f);
	glVertex3f(world_size_2, -world_size_2, 0.0f);
	glEnd();

	// Draw convex hull
	if (p.show_convex_hull) {
		glColor4f(0.9f, 0.9f, 0.1f, 0.7f);
		glBegin(GL_POLYGON);
		for (uint i = 0; i < data.ch.size(); i++) {
			glVertex3f(data.ch[i].x, data.ch[i].y, -0.1f);
		}
		glEnd();
	}

	// Draw filled polygons
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	// Draw explored grid cells on the GUI
	if (p.show_explored) {
		for (uint i = 0; i < p.world_size * p.world_size; i++) {
			// Get the world coordinates for this iteration
			float world_x = -world_size_2 +
				static_cast<float>(floor(i / p.world_size));
			float world_y = -world_size_2 + static_cast<float>(i % p.world_size);

			// Now draw the grid cell if explored
			float explored_color = 0.0f;
			if (explored_grid[i] != 0) {
				explored_color = fabsf(static_cast<float>(explored_grid[i]) /
					p.max_explore);
				// Color is based on obstacle/free space
				if (explored_grid[i] > 0) {		// Free space
					glColor4f(0.1f * explored_color, 0.3f * explored_color,
						0.6f * explored_color, 0.5f);
				}
				else {							// Obstacle
					// Lower bar for showing an obstacle cell as fully explored
					explored_color = min(1.0f, explored_color * 4.0f);
					glColor4f(0.6f * explored_color, 0.2f * explored_color,
						0.2f * explored_color, 1.0f);
				}
				glBegin(GL_POLYGON);
				glVertex3f(world_x, world_y, -0.2f);
				glVertex3f(world_x + 1.0f, world_y, -0.2f);
				glVertex3f(world_x + 1.0f, world_y + 1.0f, -0.2f);
				glVertex3f(world_x, world_y + 1.0f, -0.2f);
				glEnd();
			}
		}
	}

	// Draw targets on the GUI
	for (uint i = 0; i < p.targets; i++) {

		// Get the explored grid index that this target corresponds to
		uint exp_ind = static_cast<uint>(((targets[i].x + ws_2) * p.world_size) + 
			(targets[i].y + ws_2));

		// Set the target color based on explored value
		float target_color;
		// Target must be seen at least once to show
		(explored_grid[exp_ind] == 0) ? target_color = 0.0f : 
			target_color = fabsf(0.25f + (0.75f * 
			(static_cast<float>(explored_grid[exp_ind]) / p.max_explore)));

		// Change target color based on whether fully explored
		if (target_color < 1.0f) {
			// Purple (not fully explored)
			glColor4f(0.6f * target_color, 0.0f * target_color,
				0.6f * target_color, 1.0f);
			// If first time target is seen, indicate so in target data field, 
			// and add to targets_seen count
			if (target_color > 0.0f && targets[i].z == 0) {
				targets[i].z = 1;
				data.targets_seen++;
			}
		}
		else {
			// Green (fully explored)
			glColor4f(0.2f * target_color, 0.8f * target_color,
				0.2f * target_color, 1.0f);
			// If first time reaching fully explored status, indicate so in data
			// field and add to targets_explored count
			if (targets[i].z == 1) {
				targets[i].z = 2;
				data.targets_explored++;
			}
		}

		// Draw target
		float x = static_cast<float>(targets[i].x);
		float y = static_cast<float>(targets[i].y);
		glBegin(GL_POLYGON);
		glVertex3f(x, y, -0.1f);
		glVertex3f(x + 1.0f, y, -0.1f);
		glVertex3f(x + 1.0f, y + 1.0f, -0.1f);
		glVertex3f(x, y + 1.0f, -0.1f);
		glEnd();
	}

	// Set color to white for next GUI elements
	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);

	glResetModelAndProjection();

	// Draw progress bar for time remaining
	if (p.step_limit > 0) {
		glLineWidth(1.0f);
		float bar_y_end = -0.94f + (1.6f * (static_cast<float>(step_num) /
			static_cast<float>(p.step_limit)));
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		// Outline
		glBegin(GL_POLYGON);
		glVertex3f(-0.94f, -0.94f, 0.0f);
		glVertex3f(-0.94f, 0.66f, 0.0f);
		glVertex3f(-0.84f, 0.66f, 0.0f);
		glVertex3f(-0.84f, -0.94f, 0.0f);
		glEnd();
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		// Filling
		glBegin(GL_POLYGON);
		glVertex3f(-0.94f, -0.94f, 0.0f);
		glVertex3f(-0.94f, bar_y_end, 0.0f);
		glVertex3f(-0.84f, bar_y_end, 0.0f);
		glVertex3f(-0.84f, -0.94f, 0.0f);
		glEnd();
	}

	// Set color to cyan for user inputs
	glColor4f(0.0f, 1.0f, 1.0f, 1.0f);
	glLineWidth(2.0f);

	// Convert user-drawn line from screen to world coordinates
	float3 screen_start_point = make_float3(mouse_start_x, mouse_start_y, 
		translate_z0);
	float3 screen_last_point = make_float3(mouse_last_x, mouse_last_y, 
		translate_z0);
	float3 world_start_point = make_float3(0.0f, 0.0f, 0.0f);
	float3 world_last_point = make_float3(0.0f, 0.0f, 0.0f);
	screenToWorld(screen_start_point, &world_start_point);
	screenToWorld(screen_last_point, &world_last_point);
	// Draw the user-given direction
	glBegin(GL_LINES);
	glVertex3f(world_start_point.x, world_start_point.y, 0.0f);
	glVertex3f(world_last_point.x, world_last_point.y, 0.0f);
	glEnd();
}

void drawEllipse(float cx, float cy, float w, float h, float z, bool fill)
{
	// Variables for incremental lines
	float theta = 2.0f * static_cast<float>(PI) / 100.0f;
	float c = cosf(theta);
	float s = sinf(theta);
	float t;
	float x = 1.0f;
	float y = 0.0f;

	// Set to filled or wire frame depending on fill parameter
	(fill) ? glPolygonMode(GL_FRONT_AND_BACK, GL_FILL) : 
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	// Draw the ellipse
	glBegin(GL_POLYGON);
	for (uint i = 0; i < 100; i++) {
		glVertex3f((w * x) + cx, (h * y) + cy, z);

		// Apply the rotation matrix
		t = x;
		x = c * x - s * y;
		y = s * t + c * y;
	}
	glEnd();
}

void drawText(float x, float y, const unsigned char *string, GLfloat r,
	GLfloat g, GLfloat b)
{
	// Change to specified color
	glColor4f(r, g, b, 0.75f);

	// Draw text at the given point
	glRasterPos2f(-134.0f + translate_x0, 70.0f + translate_y0);
	glutBitmapString(GLUT_BITMAP_TIMES_ROMAN_24, string);
}

void keyboard(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 'w': {
		moveUp();
		break;
	}
	case 'a': {
		moveLeft();
		break;
	}
	case 's': {
		moveDown();
		break;
	}
	case 'd': {
		moveRight();
		break;
	}
	case ' ': {
		// Pause / resume the simulation
		paused = !paused;
		break;
	}
	case '1': {
		// Switch to rendezvous
		p.behavior = 0;
		break;
	}
	case '2': {
		// Switch to flocking
		p.behavior = 1;
		break;
	}
	case '3': {
		// Switch to dispersion
		p.behavior = 2;
		break;
	}
	case 27: { // Escape key
		exitSimulation();
		break;
	}
	}
}

// Handles directional arrow and function key presses
void keyboardSpecial(int key, int x, int y)
{
	switch (key) {
	case GLUT_KEY_UP: {
		moveUp();
		break;
	}
	case GLUT_KEY_LEFT: {
		moveLeft();
		break;
	}
	case GLUT_KEY_DOWN: {
		moveDown();
		break;
	}
	case GLUT_KEY_RIGHT: {
		moveRight();
		break;
	}
	}
}

void mouse(int button, int state, int x, int y)
{
	// If the button is pressed down (ignore subsequent buttons before the first
	// is released)
	if (state == GLUT_DOWN && mb == -1) {
		// Get mouse button
		mb = button;

		// Only process presses if unpaused
		if (!paused) {	
			// Primary buttons to give commands
			if (mb == 0) {
				mouse_start_x = static_cast<float>(x);
				mouse_start_y = static_cast<float>(y);
				mouse_last_x = mouse_start_x;
				mouse_last_y = mouse_start_y;
			}
			else if (mb == 2) {
				p.behavior = 3;
				p.align_weight = 1.0f;
				// Set goal point to swarm's center of mass
				for (uint i = 0; i < p.num_robots; i++) {
					goal_point.x += positions[i].x;
					goal_point.y += positions[i].y;
				}
				goal_point.x /= p.num_robots;
				goal_point.y /= p.num_robots;
			}
		}
		if (mb == 3) {		// Scroll wheel forward
			translate_z0 -= 1.0f;
			translate_z0 = min(translate_z0, 200.0f);
		}
		else if (mb == 4) {		// Scroll wheel backward
			translate_z0 += 1.0f;
			translate_z0 = max(translate_z0, 10.0f);
		}
	}
	else if (state == GLUT_UP && mb == button) {	// If the button is released
		if (mb == 0) { // Primary or seconday mouse button
			// If the simulation is paused, unpause it; 
			// else log the new user goal heading and log the information; 
			if (paused) {
				paused = false;
			}
			else if (!paused) {
				// Set behavior back to flocking
				p.behavior = 1;

				// Get the goal direction in radians
				goal_heading = atan2f(static_cast<float>(y)-mouse_start_y,
					static_cast<float>(x)-mouse_start_x);
				// Transform this into a 2D unit vector (float3, but z not used)
				goal_vector = make_float3(cosf(goal_heading),
					-sinf(goal_heading), 0.0f);

				// Clear the user-drawn line data points
				mouse_start_x = 0;
				mouse_start_y = 0;
				mouse_last_x = 0;
				mouse_last_y = 0;
			}
		}
		else if (mb == 5) { // Scroll wheel forward
			translate_z0 -= 1.0f;
			translate_z0 = min(translate_z0, 200.0f);
		}
		else if (mb == 6) { // Scroll wheel backward
			translate_z0 += 1.0f;
			translate_z0 = max(translate_z0, 10.0f);
		}

		// If mouse_up event cause by scrolling while left mouse is down, reset 
		// mb to 0 (left click); else, reset to null
		if (mb == 5 || mb == 6) {
			mb = 0;
		}
		else {
			mb = -1;
		}
	}
}

void motion(int x, int y)
{
	// Draw the user heading line if the primary button is down and simulation 
	// is not paused
	if ((mb == 0) && !paused) {
		mouse_last_x = static_cast<float>(x);
		mouse_last_y = static_cast<float>(y);
	}

	// Assign current mouse coordinates
	mouse_x = static_cast<float>(x);
	mouse_y = static_cast<float>(y);
}

void moveUp()
{
	translate_y0 += 1.0f;
}

void moveLeft()
{
	translate_x0 -= 1.0f;
}

void moveRight()
{
	translate_x0 += 1.0f;
}

void moveDown()
{
	translate_y0 -= 1.0f;
}

void glResetModelAndProjection()
{
	// Reinitialize OpenGL modelview and projection matricies
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

/****************************
***** OPENGL FUNCTIONS ******
****************************/

void initGL(int argc, char **argv)
{
	glutInit(&argc, argv);

	if (p.window_width == 0 || p.window_height == 0) {
		p.window_width = glutGet(GLUT_SCREEN_WIDTH);
		p.window_height = glutGet(GLUT_SCREEN_HEIGHT);
	}

	// Setup window
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
	glutInitWindowSize(p.window_width, p.window_height);
	glutCreateWindow("CUDA Swarm Simulation");

	// If in Windows, start with the window maximized
#ifdef _WIN32
	// Get the handle for the current window
	HWND win_handle = FindWindow(0, "CUDA Swarm Simulation");
	if (!win_handle)
	{
		std::printf("!!! Failed FindWindow\n");
	}
	// Maximize the window for full screen simulation operation
	SetWindowLong(win_handle, GWL_STYLE, (GetWindowLong(win_handle, GWL_STYLE) | 
		WS_MAXIMIZE));
	ShowWindowAsync(win_handle, SW_SHOWMAXIMIZED);
#endif

	// Register OpenGL callbacks
	glutDisplayFunc(display);				// OpenGL display callback (looped)
	glutKeyboardFunc(keyboard);				// OpenGL keyboard callback
	glutSpecialFunc(keyboardSpecial);		// OpenGL keyboard special callback
	glutMouseFunc(mouse);					// OpenGL mouse callback
	glutMotionFunc(motion);					// OpenGL mouse motion callback
	glutPassiveMotionFunc(motion);			// OpenGL pass. mouse motion callback
	glutTimerFunc(1000, calculateFPS, 0);	// Recalculate FPS every 1/2 second

	// GLEW initialization
	glewInit();
	if (!glewIsSupported("GL_VERSION_2_0")) {
		fprintf(stderr, 
			"ERROR: Support for necessary OpenGL extensions missing.");
		fflush(stderr);
		exit(0);
	}
}

void createVBO(GLuint* vbo, struct cudaGraphicsResource **vbo_res,
	unsigned int vbo_res_flags)
{
	// Create vertex buffer object
	glGenBuffers(1, vbo);
	glBindBuffer(GL_ARRAY_BUFFER, *vbo);

	// Initialize VBO
	uint size = p.num_robots * 4 * sizeof(float);
	glBufferData(GL_ARRAY_BUFFER, size, 0, GL_DYNAMIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	// Register VBO with CUDA
	cudaGraphicsGLRegisterBuffer(vbo_res, *vbo, vbo_res_flags);
}

void deleteVBO(GLuint *vbo, struct cudaGraphicsResource *vbo_res)
{
	// unregister this buffer object with CUDA
	cudaGraphicsUnregisterResource(vbo_res);

	// Delete VBO
	glBindBuffer(1, *vbo);
	glDeleteBuffers(1, vbo);
	*vbo = 0;
}

static void display(void)
{
	// Quit if not automated and last step reached; else take a sulation step
	if (step_num > p.step_limit) {
		exitSimulation();
	}
	else {
		step(0);
	}

	// Get the window dimensions
	int window_width = glutGet(GLUT_WINDOW_WIDTH);
	int window_height = glutGet(GLUT_WINDOW_HEIGHT);

	// Clear the display
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glViewport(0, 0, window_width, window_height);

	// Set the environment background
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

	// Draw interface elements
	drawInterface(static_cast<float>(window_width),
		static_cast<float>(window_height));

	// Projection
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(75.0, (GLfloat)window_width / (GLfloat)window_height,
		0.001, 500.0);

	// Change point size based on distance from camera
	glPointSize(static_cast<float>(p.point_size));
	glEnable(GL_POINT_SMOOTH);
	float quadratic[] = {0.05f, 0.0f, 0.001f};
	glPointParameterfvARB(GL_POINT_DISTANCE_ATTENUATION_ARB, quadratic);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// Cap the translation values to +/- world size + 10
	translate_x0 = min(translate_x0, ws_2 + 10.0f);
	translate_x0 = max(translate_x0, -ws_2 - 10.0f);
	translate_y0 = min(translate_y0, ws_2 + 10.0f);
	translate_y0 = max(translate_y0, -ws_2 - 10.0f);
	translate_z0 = min(translate_z0, ws_2 * 2.0f);
	translate_z0 = max(translate_z0, 1.0f);

	// Apply view transforms
	glRotatef(rotate_x0, 1.0, 0.0, 0.0);
	glRotatef(rotate_y0, 0.0, 1.0, 0.0);
	glTranslatef(-translate_x0, -translate_y0, -translate_z0);

	// Closer things cover far things
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);
	glDepthMask(GL_TRUE);

	// Transparency
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	// Draw agents from vbo
	glBindBuffer(GL_ARRAY_BUFFER, vbo_swarm);
	glVertexPointer(3, GL_FLOAT, 16, 0);
	glColorPointer(4, GL_UNSIGNED_BYTE, 16, (GLvoid*)12);
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);
	glDrawArrays(GL_POINTS, 0, p.num_robots);
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);

	// Draw filled polygons
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	// Draw robot data
	for (uint i = 0; i < p.num_robots; i++) {

		// Orientation lines
		if (modes[i] >= 0) {
			glColor4f(1.0f, 1.0f, 1.0f, 0.5f);
			glLineWidth(2.0f);
			glBegin(GL_LINES);
			glVertex3f(positions[i].x, positions[i].y, 0.0f);
			glVertex3f(positions[i].x + ((100.0f * velocities[i].x) / 
				p.vel_bound), positions[i].y + ((100.0f * velocities[i].y) / 
				p.vel_bound), 0.1f);
			glEnd();
		}

		glColor4f(1.0f, 1.0f, 1.0f, 0.1f);
		glLineWidth(1.0f);

		// Communication connections
		if (p.show_connections) {
			for (uint j = i + 1; j < p.num_robots; j++) {
				if (laplacian[(i * p.num_robots) + j] == -1) {
					if (modes[i] >= 0 && modes[j] >= 0) {
						glBegin(GL_LINES);
						glVertex3f(positions[i].x, positions[i].y, 0.0f);
						glVertex3f(positions[j].x, positions[j].y, 0.0f);
						glEnd();
					}
				}
			}
		}

		// Show communication range if specified in parameters
		if (p.show_range || (p.show_range_leaders && modes[i] == 0)) {
			glColor4f(1.0f, 1.0f, 1.0f, 0.2f);
			drawEllipse(positions[i].x, positions[i].y, p.range, p.range, -0.1f, 
				false);
			glColor4f(0.5f, 0.5f, 0.5f, 0.1f);
			drawEllipse(positions[i].x, positions[i].y, p.range, p.range, -0.2f, 
				true);
		}
	}

	// Refresh display
	glutSwapBuffers();
	glutPostRedisplay();

	// Increment frames shown
	frames++;
}


void screenToWorld(float3 screen, float3 *world)
{
	// Initialize variables
	GLint viewport[4];
	GLdouble modelview[16];
	GLdouble projection[16];
	// Screen and world coordinates
	GLdouble x_s, y_s, z_s;
	GLdouble x_w, y_w, z_w;

	// Get view matrix data and viewport bounds
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
	glGetDoublev(GL_PROJECTION_MATRIX, projection);
	glGetIntegerv(GL_VIEWPORT, viewport);

	// Get screen coordinates
	x_s = static_cast<double>(screen.x);
	y_s = viewport[3] - static_cast<double>(screen.y) - 10.0;
	z_s = static_cast<double>(screen.z);

	// Get world coordinates from screen coordinates
	gluUnProject(x_s, y_s, z_s, modelview, projection, viewport, 
		&x_w, &y_w, &z_w);
	world->x = static_cast<float>(x_w);
	world->y = static_cast<float>(y_w);
	world->z = static_cast<float>(z_w);
}

void worldToScreen(float3 world, float3 *screen)
{
	// Transform matricies
	GLint viewport[4];
	GLdouble modelview[16];
	GLdouble projection[16];
	// Screen and world coordinates
	GLdouble x_s, y_s, z_s;
	GLdouble x_w, y_w, z_w;

	// Get view matrix data and viewport bounds
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
	glGetDoublev(GL_PROJECTION_MATRIX, projection);
	glGetIntegerv(GL_VIEWPORT, viewport);

	// Get world coordinates
	x_w = static_cast<double>(world.x);
	y_w = static_cast<double>(world.y);
	z_w = static_cast<double>(world.z);

	gluProject(x_w, y_w, z_w, modelview, projection, viewport, &x_s, &y_s, &z_s);
	screen->x = static_cast<float>(x_s);
	screen->y = static_cast<float>(y_s);
	screen->z = 0.0f; //static_cast<float>(z_s);
}

void resetCamera()
{
	// Reset the camera to the start position
	translate_x0 = 0.0f;
	translate_y0 = 0.0f;
	translate_z0 = 100.0f;
	rotate_x0 = 0.0f;
	rotate_y0 = 0.0f;
}

/****************************
***** HELPER FUNCTIONS ******
****************************/

void calculateFPS(int value)
{
	// Convert frames per second into string for window header
	char fps_text[256];
	sprintf_s(fps_text, "CUDA Swarm Simulation (%d FPS)", frames);
	glutSetWindowTitle(fps_text);

	// Reset the frames counter
	last_frames = frames;
	frames = 0;

	// Reset timer every second for this function to recalculate FPS
	glutTimerFunc(1000, calculateFPS, 0);
}

void loadParametersFromFile(std::string filename)
{
	std::fstream file(filename);
	std::string str;
	unsigned int line = 1;
	size_t comment;

	// Get the parameters from specified file
	while (std::getline(file, str)) {
		// Remove any comment from the line
		if (comment = str.find('#', 0)) {
			str = str.substr(0, comment);
		}

		// String working variables
		std::istringstream iss(str);
		std::vector<std::string> tokens;

		// Place the parameter and value into the token array
		copy(std::istream_iterator<std::string>(iss),
			std::istream_iterator<std::string>(), back_inserter(tokens));

		// Ensure valid line before processing parameter
		if (tokens.size() == 2) {
			processParam(tokens);
		}

		// Move to the next line in the parameters file
		line++;
	}
}

void processParam(std::vector<std::string> tokens)
{
	// Assign parameters according to the parameter name
	if (tokens[0] == "align_weight")
		p.align_weight = std::stof(tokens[1]);
	else if (tokens[0] == "ang_bound")
		p.ang_bound = std::stof(tokens[1]);
	else if (tokens[0] == "behavior")
		p.behavior = std::stoul(tokens[1]);
	else if (tokens[0] == "cohere_weight")
		p.cohere_weight = std::stof(tokens[1]);
	else if (tokens[0] == "confirm_quit")
		p.confirm_quit = (std::stoul(tokens[1]) != 0);
	else if (tokens[0] == "current")
		p.current = std::stof(tokens[1]);
	else if (tokens[0] == "hops")
		p.hops = std::stoul(tokens[1]);
	else if (tokens[0] == "leader_selection")
		p.leader_selection = std::stoul(tokens[1]);
	else if (tokens[0] == "log_data")
		p.log_data = (std::stoul(tokens[1]) != 0);
	else if (tokens[0] == "range_r")
		p.range_r = std::stof(tokens[1]);
	else if (tokens[0] == "range_f")
		p.range_f = std::stof(tokens[1]);
	else if (tokens[0] == "range_d")
		p.range_d = std::stof(tokens[1]);
	else if (tokens[0] == "range_o")
		p.range_o = std::stof(tokens[1]);
	else if (tokens[0] == "range_l")
		p.range_l = std::stof(tokens[1]);
	else if (tokens[0] == "range")
		p.range = std::stof(tokens[1]);
	else if (tokens[0] == "max_explore")
		p.max_explore = std::stoul(tokens[1]);
	else if (tokens[0] == "max_obstacle_size")
		p.max_obstacle_size = std::stoul(tokens[1]);
	else if (tokens[0] == "num_obstacles")
		p.num_obstacles = std::stoul(tokens[1]);
	else if (tokens[0] == "num_robots")
		p.num_robots = std::stoul(tokens[1]);
	else if (tokens[0] == "point_size")
		p.point_size = std::stoul(tokens[1]);
	else if (tokens[0] == "repel_weight")
		p.repel_weight = std::stof(tokens[1]);
	else if (tokens[0] == "get_connectivity")
		p.get_connectivity = (std::stoul(tokens[1]) != 0);
	else if (tokens[0] == "get_ap")
		p.get_ap = (std::stoul(tokens[1]) != 0);
	else if (tokens[0] == "show_ap")
		p.show_ap = (std::stoul(tokens[1]) != 0);
	else if (tokens[0] == "show_connections")
		p.show_connections = (std::stoul(tokens[1]) != 0);
	else if (tokens[0] == "show_convex_hull")
		p.show_convex_hull = (std::stoul(tokens[1]) != 0);
	else if (tokens[0] == "show_explored")
		p.show_explored = (std::stoul(tokens[1]) != 0);
	else if (tokens[0] == "show_mode")
		p.show_mode = (std::stoul(tokens[1]) != 0);
	else if (tokens[0] == "show_leaders_only")
		p.show_leaders_only = (std::stoul(tokens[1]) != 0);
	else if (tokens[0] == "show_range")
		p.show_range = (std::stoul(tokens[1]) != 0);
	else if (tokens[0] == "show_range_leaders")
		p.show_range_leaders = (std::stoul(tokens[1]) != 0);
	else if (tokens[0] == "start_size")
		p.start_size = std::stof(tokens[1]);
	else if (tokens[0] == "step_limit")
		p.step_limit = std::stoul(tokens[1]);
	else if (tokens[0] == "targets")
		p.targets = std::stoul(tokens[1]);
	else if (tokens[0] == "training")
		p.training = (std::stoul(tokens[1]) != 0);
	else if (tokens[0] == "vel_bound")
		p.vel_bound = std::stof(tokens[1]);
	else if (tokens[0] == "window_height")
		p.window_height = std::stoul(tokens[1]);
	else if (tokens[0] == "window_width")
		p.window_width = std::stoul(tokens[1]);
	else if (tokens[0] == "world_size")
		p.world_size = std::stoul(tokens[1]);
}

void generateWorld()
{
	// Create the specified number of obstacles in the parameters file
	for (uint i = 0; i < p.num_obstacles; i++) {
		bool obstacle_accepted = false;

		// Generate obstacles, discarding ones that don't fit the criteria
		while (!obstacle_accepted) {
			float4 obstacle = make_float4(0.0f, 0.0f, 0.0f, 0.0f);

			// Create width and height for the obstacle
			obstacle.z = static_cast<float>(rand() % p.max_obstacle_size);
			obstacle.w = static_cast<float>(rand() % p.max_obstacle_size);
			// Create x, y position for top left corner of rectangular obstacle
			obstacle.x = rand() % (p.world_size - static_cast<uint>(obstacle.z)) -
				(p.world_size / 2.0f);
			obstacle.y = rand() % (p.world_size - static_cast<uint>(obstacle.w)) -
				(p.world_size / 2.0f);

			// Ensure obstacle does not cover the start or goal areas and that 
			// it is not too thin
			if ((obstacle.x < -(p.start_size + 1.0f) - obstacle.z || 
				obstacle.x > (p.start_size + 1.0f) ||
				obstacle.y < -(p.start_size + 1.0f) - obstacle.w || 
				obstacle.y > (p.start_size + 1.0f)) &&
				(obstacle.z > 3.0f && obstacle.w > 3.0f)) {
				// Signal the obstacle fits criteria
				obstacle_accepted = true;
				// Add this to the list of obstacles
				obstacles[i] = obstacle;
			}
		}
	}

	// Create the specified number of targets in the parameters file
	for (uint i = 0; i < p.targets; i++) {
		bool target_accepted = false;

		// Generate targets, discarding ones that are within obstacles
		while (!target_accepted) {
			int3 target = make_int3(0, 0, 0);
			float x = rand() % (p.world_size) - (p.world_size / 2.0f);
			float y = rand() % (p.world_size) - (p.world_size / 2.0f);
			target.x = static_cast<int>(x);
			target.y = static_cast<int>(y);

			// Ensure this target is not within an obstacle
			if (!checkCollision(x, y)) {
				// Signal this obstacle fits the criteria
				target_accepted = true;
				// Add this to the list of targets
				targets[i] = target;
			}
		}
	}
}

void calculateOccupancyGrid()
{
	// Iterate through each cell of the occupancy grid to calculate its value
	float ws_10 = static_cast<float>(p.world_size) * 10.0f;
	uint len = static_cast<uint>(ws_10 * ws_10);
	for (uint i = 0; i < len; i++) {
		// Get the x and y coordinates for this cell
		float x = ((i % static_cast<int>(ws_10)) / 10.0f) - ws_2;
		float y = (floorf(static_cast<float>(i) / ws_10) / 10.0f) - ws_2;
		occupancy[i] = checkCollision(x, y);
	}
}

bool checkCollision(float x, float y)
{
	for (uint i = 0; i < p.num_obstacles; i++) {
		// Collision occurs if the point to check is within an obstacle or outside 
		// the world boundaries
		if (x >= obstacles[i].x && x < obstacles[i].x + obstacles[i].z &&
			y > obstacles[i].y && y < obstacles[i].y + obstacles[i].w) {
			return true;
		}
	}
	return false;
}

void updateExplored()
{
	// Variables to used in explored grid cell updates
	float world_size_2 = p.world_size / 2.0f;

	// Update explored grid values
	for (uint i = 0; i < p.world_size * p.world_size; i++) {
		// Skip cells that are fully explored
		if (abs(explored_grid[i]) < static_cast<int>(p.max_explore)) {
			// Get the world coordinates for this iteration
			float world_x = -world_size_2 + 
				static_cast<float>(floor(i / p.world_size));
			float world_y = -world_size_2 + static_cast<float>(i % p.world_size);

			// Only do the following if cell is within range of the swarm bounds
			// and if not paused
			if ((world_x > data.bounds.x - p.range) &&
				(world_x < data.bounds.y + p.range) &&
				(world_y > data.bounds.z - p.range) &&
				(world_y < data.bounds.w + p.range) && !paused) {
				// Check each robot to see if it is within range of this cell
				for (uint n = 0; n < p.num_robots; n++) {
					if (eucl2(world_x + 0.5f, world_y + 0.5f,
						positions[n].x, positions[n].y) <= p.range) {
						// Increment/decrement based on whether cell is obstacle
						if (explored_grid[i] >= 0) {
							explored_grid[i]++;
						}
						else {
							explored_grid[i]--;
						}
					}
				}

				// Restrict the absolute value of explored value to p.max_explore
				if (explored_grid[i] > 0) {
					explored_grid[i] = min(explored_grid[i],
						static_cast<int>(p.max_explore));
				}
				else {
					explored_grid[i] = max(explored_grid[i],
						static_cast<int>(p.max_explore) * -1);
				}
			}
		}
	}
}

void exitSimulation()
{
	// Delete vertex buffer object
	deleteVBO(&vbo_swarm, cuda_vbo_resource);

	// Free CUDA variables
	cuFree();
	cudaFree(positions);
	cudaFree(velocities);
	cudaFree(modes);
	cudaFree(leaders);
	cudaFree(nearest_leaders);
	cudaFree(leader_countdowns);
	cudaFree(obstacles);

	// Free non-CUDA variables
	std::free(explored_grid);
	std::free(ap);
	std::free(targets);
	std::free(occupancy);

	// Close the output and world data files
	if (output_f != NULL) {
		fclose(output_f);
	}

	// Close window and exit
	glutDestroyWindow(0);

	// Wait for keypress to exit (if set in parameters)
	if (p.confirm_quit) {
		std::printf("Enter any key to quit.\n");
		getchar();
	}

	// Reset CUDA device
	cudaDeviceReset();

	std::exit(0);
}

void printDataHeader()
{
	if (p.log_data) {
		// Step data header
		fprintf(output_f, "step_num behavior behavior_data failure_data ");
		fprintf(output_f, "velocity avg_heading heading_var centroid_x ");
		fprintf(output_f, "centroid_y convex_hull_area connectivity ");
		fprintf(output_f, "explored_area\n");
	}
}

/************************
***** PROGRAM LOOP ******
************************/

static void step(int value)
{
	// Only perform a step if not paused
	if (!paused) {
		// Create the output file for logging
		if (!initial_passed) {
			// Print the data column headers
			if (p.log_data) {
				fopen_s(&output_f, output_fname.str().c_str(), "w");
				printDataHeader();
			}
			
			// Process data of initial state
			processData(positions, velocities, explored_grid, laplacian, ap, 
				&data, p);

			// Indicates inital state has passed
			initial_passed = true;
		}

		// Launch the main kernel to perform one simulation step
		launchMainKernel(goal_vector, goal_point, step_num, leaders, ap, p, 
				&cuda_vbo_resource);

		// Retrieve data from GPU (kernels.cu)
		getData(p.num_robots, positions, velocities, modes);
		// Update explored grid
		if (p.show_explored) {
			updateExplored();
		}
		// Get data variables (data_ops.h)
		processData(positions, velocities, explored_grid, laplacian, ap, &data, 
			p);

		// Update leader list (Very inefficient now, should compute at the same 
		// time as convex hull)
		for (uint i = 0; i < p.num_robots; i++) {
			bool is_in_ch = false;
			for (uint j = 0; j < data.ch.size(); j++) {
				if (positions[i].x == data.ch[j].x &&
					positions[i].y == data.ch[j].y) {
					is_in_ch = true;
					break;
				}
			}
			(is_in_ch) ? leaders[i] = 0 : leaders[i] = 1;
		}

		if (p.log_data) {
			// Write data to the output log at the end of every step
			fprintf(output_f, "%d %d %f %f %f %f %f %f %f %f %f %d\n",
				step_num, p.behavior, -goal_heading, -goal_heading_err, 
				p.vel_bound, data.heading_avg, data.heading_var, data.centroid.x, 
				data.centroid.y, data.ch_area, data.connectivity, data.explored);
		}

		// Increment the simulation step counter
		step_num++;
	}
}

/****************
***** MAIN ******
****************/

int main(int argc, char** argv)
{
	// Reseed RNG
	srand(static_cast<uint>(time(NULL)));

	///// PARAMETERS /////
	// If not in playback mode, load the parameters from file, given as first 
	// command line argument;
	// else, load the world file in playback mode
	loadParametersFromFile(argv[1]);

	// Begin paused
	paused = true;
	
	// Half the world size
	ws_2 = static_cast<float>(p.world_size) / 2.0f;
	// Open new data file for this trial
	output_fname << argv[2];

	///// MEMORY ALLOCATION /////
	explored_grid = (int*)malloc(p.world_size * p.world_size * sizeof(int));
	ap = (bool*)malloc(p.num_robots * sizeof(bool));
	targets = (int3*)malloc(p.targets * sizeof(int3));
	occupancy = (bool*)malloc(p.world_size * 10 * p.world_size * 10 * 
		sizeof(bool));
	// Initialize pinned host memory for data arrays
	cudaHostAlloc(&positions, p.num_robots * sizeof(float4), 0);
	cudaHostAlloc(&velocities, p.num_robots * sizeof(float3), 0);
	cudaHostAlloc(&modes, p.num_robots * sizeof(float4), 0);
	cudaHostAlloc(&leaders, p.num_robots * sizeof(int), 0);
	cudaHostAlloc(&nearest_leaders, p.num_robots * sizeof(int), 0);
	cudaHostAlloc(&leader_countdowns, p.num_robots * sizeof(uint), 0);
	cudaHostAlloc(&laplacian, p.num_robots * p.num_robots * sizeof(int), 0);
	cudaHostAlloc(&obstacles, p.num_obstacles * sizeof(float4), 0);
	// Fill the leader list with -1 initially
	fill(leaders, leaders + p.num_robots, -1);
	// GPU memory allocation
	cudaAllocate(p);

	///// OPEN GL INITIALIZATION /////
	// Initialize OpenGL
	initGL(argc, argv);
	// Create vertex buffer object (VBO)
	createVBO(&vbo_swarm, &cuda_vbo_resource,
		cudaGraphicsMapFlagsWriteDiscard);
	// Set camera to default settings
	resetCamera();

	///// CUDA INITIALIZATION /////
	launchInitKernel(p, &cuda_vbo_resource);
	// Retrieve initial data from GPU (kernels.cu)
	getData(p.num_robots, positions, velocities, modes);
	getLaplacian(p.num_robots, laplacian);

	///// WORLD GENERATION /////
	generateWorld();

	///// OCCUPANCY AND EXPLORATION GRID INITIALIZATION /////
	calculateOccupancyGrid();
	// Send occupancy grid to GPU
	setOccupancy(p, occupancy);
	// Create the explored grid
	for (uint i = 0; i < p.world_size * p.world_size; i++) {
		// Get the coordinates of the grid cell
		float y = static_cast<float>(i % p.world_size) - ws_2;
		float x = floorf(static_cast<float>(i) / 
			static_cast<float>(p.world_size)) - ws_2;

		// Initialize cell to -1 if it is covered by an obstacle; 0 otherwise
		(checkCollision(x, y)) ? explored_grid[i] = -1 : explored_grid[i] = 0;
	}

	// Initialize goal heading, goal region, and goal point to 0
	goal_heading = 0.0f;
	goal_point = make_float2(0.0f, 0.0f);
	
	///// START MAIN LOOP /////
	glutMainLoop();

	///// QUIT /////
	exitSimulation();

	return 0;
}
