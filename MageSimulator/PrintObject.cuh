#pragma once
#ifndef PRINT_OBJECT
#define PRINT_OBJECT

#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include "Structs.cuh"
#include "ToolPath.cuh"
#include <vector>
#include "Utility.cuh"
#include <algorithm>
#include <thrust/sort.h>

#define BVH_NODES_CNT 10000


cudaError_t calcPrintItems(int cnt, PathItem* path, PrintItem* print);
__global__ void calcPrintItem(int cnt, PathItem* path, PrintItem* print);
void drawPrintItem_box(PrintItem item, float face_r, float face_g, float face_b,
  float edge_r, float edge_g, float edge_b);
void drawPrintItem_move(PrintItem item, float move_r, float move_g, float move_b);

void constructBVH_internal(std::vector<PrintItem*>& items, int nodeIndex);
void constructBVH(std::vector<PrintItem*>& input,
  std::vector<std::vector<PrintItem*>>* outputPrint,
  std::vector<Ray*>* outputRay, std::vector<Mesh*>* outputMesh);

#endif