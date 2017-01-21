# cuSwarm
CUDA-based simulator of agent-based swarms

DESCRIPTION

cuSwarm is a basic simulator for swarms of agents or robots using the NVIDIA CUDA platform. I use this simulation primarily to teach myself aspects fo agent-based simulation and parallel programming, and also to test new additions to experiments I run as part of the Usability Lab at the University of Pittsburgh School of information sciences.

Optimization of the simulation is always a work in progress, but if you are able to run it on your computer, you can use the params.txt file as a guide for the types of changes you can make before running the simulation.

To compile and run the simulator on a Windows system, you will need to following:
- NVIDIA GPU (typically any in the past few years will do. If the GPU is especially old, you may be able to get it to compile by lowering the compute parameters in the Visual Studio solution properties)
- Updated NVIDIA drivers
- CUDA toolkit, available for download for free at the NVIDIA website
- The libraries listed below

Once compiled, the .exe file can be run via the command line using the following syntax
> cuSwarm.exe [param file] [data ouput file]
The following is an example, that will use the parameters stored in "myparams.txt" and output simulation data to "output.txt":
> cuSwarm.exe myparams.txt output.txt

REQUIRED LIBRARIES

- OpenGL, used for drawing the interface (download at https://www.opengl.org/wiki/Getting_Started#Downloading_OpenGL)
- Eigen, used for data calculations of the robot communication graph (download at http://eigen.tuxfamily.org/index.php?title=Main_Page)

FILE DESCRIPTIONS

---run.cpp/run.h---
The main file that runs the simulation and draws the interface using OpenGL

---kernels.cu/kernels.cuh---
CUDA file containing all device kernels

---data.cpp/data.h---
Data calculations used for the simulation, output logging, and drawing certain GUI elements

---utils.cpp/utils.h---
Utilities file
