#pragma once
#ifndef COLLISION_CHECK
#define COLLISION_CHECK

#include "PrintObject.cuh"
#include "MachineObject.cuh"
#include <stdio.h>

#define EPSION 0.000001

cudaError_t checkCollisions(int cnt, Mesh* mesh, Ray* ray, CollisionSet* col);
__global__ void checkCollision(int cnt, Mesh* mesh, Ray* ray, CollisionSet* col);

cudaError_t createLargeArrays(std::vector<Mesh> inMesh, std::vector<Ray> inRay, Mesh* outMesh, Ray* outRay);
__global__ void createLargeArray(int meshSize, int raySize, Mesh* inMesh, Ray* inRay, Mesh* outMesh, Ray* outRay);

#endif