#pragma once
#ifndef SIMPLE_GL
#define SIMPLE_GL

// OpenGL Graphics includes need first include
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <GL/freeglut.h>
#include <GL/glut.h>
// includes, cuda
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

// others
#define _USE_MATH_DEFINES
#include <math.h>
#include "Structs.cuh"
#include "Kinematics.cuh"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <windows.h>


// Utilities and timing functions
#include <helper_functions.h>    // includes cuda.h and cuda_runtime_api.h

// CUDA helper functions
#include <helper_cuda.h>         // helper functions for CUDA error check
#include <helper_cuda_gl.h>      // helper functions for CUDA/GL interop

#define TIME_STEP_LEFT_RIGHT 1000

// opengl
void init(int argc, char** argv);
void reshape(int w, int h);
void motion(int x, int y);
void mouse(int button, int state, int x, int y);
void keyboard(unsigned char key, int x, int y);
void specialKeyboard(int key, int x, int y);
void timer(int value);
void sampling();
void drawCoordinate(float length, PrintItem item, Kinematics* kin);

#endif