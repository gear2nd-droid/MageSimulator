#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <math_functions.h>
#include <GL/glut.h>
#include <math.h>
#include "Structs.cuh"
#include <vector>

void calcMatMat(float ma[3 * 3], float mb[3 * 3], float ret[3 * 3]);
void calcMatVec(float m[3 * 3], float v[3], float ret[3]);
void calcRot2Quat(float rx, float ry, float rz,
	float* nx, float* ny, float* nz, float* angle);
void calcQuat2RotYxz(const float nx, const float ny, const float nz,
	const float angle, float m[3 * 3]);
void calcQuat_byVec2Vec(float stx, float sty, float stz, float edx, float edy, float edz,
	float* nx, float* ny, float* nz, float* angle);

__device__ __host__ void HSVtoRGB(float H, float S, float V, float& R, float& G, float& B);
Mesh* createMesh2Bbox(std::vector<Mesh> org, int id);
