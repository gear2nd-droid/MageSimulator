#include "PrintObject.cuh"


cudaError_t calcPrintItems(int cnt, PathItem* path, PrintItem* print)
{
  cudaError_t cudaStatus;
  PathItem* dev_path;
  PrintItem* dev_print;

  // Choose which GPU to run on, change this on a multi-GPU system.
  cudaStatus = cudaSetDevice(0);
  if (cudaStatus != cudaSuccess) {
    fprintf(stderr, "cudaSetDevice failed!  Do you have a CUDA-capable GPU installed?");
    goto Error;
  }

  // Allocate GPU buffers for three vectors (two input, one output)    .
  cudaStatus = cudaMalloc((void**)&dev_path, cnt * sizeof(PathItem));
  if (cudaStatus != cudaSuccess) {
    fprintf(stderr, "cudaMalloc failed!");
    goto Error;
  }
  cudaStatus = cudaMalloc((void**)&dev_print, (cnt - 1) * sizeof(PrintItem));
  if (cudaStatus != cudaSuccess) {
    fprintf(stderr, "cudaMalloc failed!");
    goto Error;
  }

  // Copy input vectors from host memory to GPU buffers.
  cudaStatus = cudaMemcpy(dev_path, path, cnt * sizeof(PathItem), cudaMemcpyHostToDevice);
  if (cudaStatus != cudaSuccess) {
    fprintf(stderr, "cudaMemcpy failed!");
    goto Error;
  }

  // Launch a kernel on the GPU with one thread for each element.
  calcPrintItem<<<(cnt + 256 - 1) / 256, 256>>>(cnt, dev_path, dev_print);

  // Check for any errors launching the kernel
  cudaStatus = cudaGetLastError();
  if (cudaStatus != cudaSuccess) {
    fprintf(stderr, "addKernel launch failed: %s\n", cudaGetErrorString(cudaStatus));
    goto Error;
  }

  // cudaDeviceSynchronize waits for the kernel to finish, and returns
  // any errors encountered during the launch.
  cudaStatus = cudaDeviceSynchronize();
  if (cudaStatus != cudaSuccess) {
    fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching addKernel!\n", cudaStatus);
    goto Error;
  }

  // Copy output vector from GPU buffer to host memory.
  cudaStatus = cudaMemcpy(print, dev_print, (cnt - 1) * sizeof(PrintItem), cudaMemcpyDeviceToHost);
  if (cudaStatus != cudaSuccess) {
    fprintf(stderr, "cudaMemcpy failed!");
    goto Error;
  }

  Error:
    cudaFree(dev_path);
    cudaFree(dev_print);

    return cudaStatus;
}

__global__ void calcPrintItem(int cnt, PathItem* path, PrintItem* print)
{
  int idx = blockDim.x * blockIdx.x + threadIdx.x;
  if (cnt > idx)
  {

    PathItem pre = path[idx];
    PathItem now = path[idx + 1];
    PrintItem bufItem;
    bufItem.lineIdx = now.lineIdx;
    bufItem.blockIdx = now.blockIdx;
    bufItem.layerIdx = now.layerIdx;
    for(int i = 0; i < 8; i++) bufItem.cmdVal[i] = now.cmdVal[i];
    float dirLen = 0.0;
    float headNormLen = 0.0;
    bufItem.size[1] = (pre.width + now.width) / 2.0;
    bufItem.size[2] = (pre.thick + now.thick) / 2.0;
    for (int i = 0; i < 3; i++)
    {
      bufItem.stPnt[i] = pre.pnt[i];
      bufItem.edPnt[i] = now.pnt[i];
      bufItem.tip[i] = (pre.pnt[i] + now.pnt[i]) / 2.0;
      bufItem.dir[i] = now.pnt[i] - pre.pnt[i];
      dirLen += bufItem.dir[i] * bufItem.dir[i];
      bufItem.headNorm[i] = (pre.norm[i] + now.norm[i]) / 2.0;
      headNormLen += bufItem.headNorm[i] * bufItem.headNorm[i];
      bufItem.center[i] = bufItem.tip[i] - bufItem.headNorm[i] / 2.0 * bufItem.size[2];
    }
    for (int i = 0; i < 6; i++)
    {
      bufItem.stMoveVal[i] = pre.moveVal[i];
      bufItem.edMoveVal[i] = pre.moveVal[i];
    }
    dirLen = sqrt(dirLen);
    headNormLen = sqrt(headNormLen);
    if (dirLen > 0.0) {
      for (int i = 0; i < 3; i++)
      {
        bufItem.dir[i] /= dirLen;
        bufItem.headNorm[i] /= headNormLen;
      }
    }
    else
    {
      bufItem.dir[0] = 1.0;
      bufItem.dir[1] = 0.0;
      bufItem.dir[2] = 0.0;
    }
    bufItem.size[0] = dirLen;
    bufItem.center[0] -= bufItem.dir[0] * bufItem.size[2] / 2.0;
    bufItem.center[1] -= bufItem.dir[1] * bufItem.size[2] / 2.0;
    bufItem.center[2] -= bufItem.dir[2] * bufItem.size[2] / 2.0;

    // calc rotate
    // matY * matX * (0,0,1) = (nx,ny,nz)
    float bufI = bufItem.headNorm[0];
    if (bufI > 1.0) bufI = 1.0;
    if (bufI < -1.0) bufI = -1.0;
    float bufJ = bufItem.headNorm[1];
    if (bufJ > 1.0) bufJ = 1.0;
    if (bufJ < -1.0) bufJ = -1.0;
    float rx = asin(-bufJ);
    float ry = 0.0;
    if (cos(rx) == 0.0)
    {
      ry = 0.0;
    }
    else
    {
      float bufVal = bufI / cos(rx);
      if (bufVal > 1.0) bufVal = 1.0;
      if (bufVal < -1.0) bufVal = -1.0;
      ry = asin(bufVal);
    }
    if (abs(bufJ) < 0.000001) bufJ = 0.0;
    if (abs(bufI) < 0.000001) bufI = 0.0;
    float rz = atan2(bufJ, bufI);
    //printf("%lf,%lf,%lf,%lf,%lf,%lf\n", rx, ry, rz, bufItem.headNorm[0], bufItem.headNorm[1], bufItem.headNorm[2]);
    bufItem.rot[0] = rx;
    bufItem.rot[1] = ry;
    bufItem.rot[2] = rz;

    // calc quartenion
    float cx = cos(rx / 2.0);
    float sx = sin(rx / 2.0);
    float cy = cos(ry / 2.0);
    float sy = sin(ry / 2.0);
    float cz = cos(rz / 2.0);
    float sz = sin(rz / 2.0);
    float q[4];
    q[0] = cx * sy * sz + sx * cy * cz;
    q[1] = -sx * cy * sz + cx * sy * cz;
    q[2] = cx * cy * sz - sx * sy * cz;
    q[3] = sx * sy * sz + cx * cy * cz;
    if (q[3] > 1.0) q[3] = 1.0;
    if (q[3] < -1.0) q[3] = -1.0;
    bufItem.quatAngle = acos(q[3]) * 2.0;
    if (sin(bufItem.quatAngle / 2.0) == 0.0)
    {
      bufItem.quatNorm[0] = 0.0;
      bufItem.quatNorm[1] = 0.0;
      bufItem.quatNorm[2] = 1.0;
    }
    else
    {
      bufItem.quatNorm[0] = q[0] / sin(bufItem.quatAngle / 2.0);
      bufItem.quatNorm[1] = q[1] / sin(bufItem.quatAngle / 2.0);
      bufItem.quatNorm[2] = q[2] / sin(bufItem.quatAngle / 2.0);
      float nlen = sqrt(bufItem.quatNorm[0] * bufItem.quatNorm[0]
        + bufItem.quatNorm[1] * bufItem.quatNorm[1] + bufItem.quatNorm[2] * bufItem.quatNorm[2]);
      if (nlen == 0.0)
      {
        bufItem.quatNorm[0] = 0.0;
        bufItem.quatNorm[1] = 0.0;
        bufItem.quatNorm[2] = 1.0;
      }
      else
      {
        bufItem.quatNorm[0] /= nlen;
        bufItem.quatNorm[1] /= nlen;
        bufItem.quatNorm[2] /= nlen;
      }
    }

    // calc rotMat
    float qx = bufItem.quatNorm[0] * sin(bufItem.quatAngle / 2.0);
    float qy = bufItem.quatNorm[1] * sin(bufItem.quatAngle / 2.0);
    float qz = bufItem.quatNorm[2] * sin(bufItem.quatAngle / 2.0);
    float qw = cos(bufItem.quatAngle / 2.0);
    bufItem.rotMat[3 * 0 + 0] = 2.0 * qw * qw + 2.0 * qx * qx - 1.0;
    bufItem.rotMat[3 * 0 + 1] = 2.0 * qx * qy - 2.0 * qz * qw;
    bufItem.rotMat[3 * 0 + 2] = 2.0 * qx * qz + 2.0 * qy * qw;
    bufItem.rotMat[3 * 1 + 0] = 2.0 * qx * qy + 2.0 * qz * qw;
    bufItem.rotMat[3 * 1 + 1] = 2.0 * qw * qw + 2.0 * qy * qy - 1.0;
    bufItem.rotMat[3 * 1 + 2] = 2.0 * qy * qz - 2.0 * qx * qw;
    bufItem.rotMat[3 * 2 + 0] = 2.0 * qx * qz - 2.0 * qy * qw;
    bufItem.rotMat[3 * 2 + 1] = 2.0 * qy * qz + 2.0 * qx * qw;
    bufItem.rotMat[3 * 2 + 2] = 2.0 * qw * qw + 2.0 * qz * qz - 1.0;
    bufItem.invRotMat[3 * 0 + 0] = 2.0 * qw * qw + 2.0 * -qx * -qx - 1.0;
    bufItem.invRotMat[3 * 0 + 1] = 2.0 * -qx * -qy - 2.0 * -qz * qw;
    bufItem.invRotMat[3 * 0 + 2] = 2.0 * -qx * -qz + 2.0 * -qy * qw;
    bufItem.invRotMat[3 * 1 + 0] = 2.0 * -qx * -qy + 2.0 * -qz * qw;
    bufItem.invRotMat[3 * 1 + 1] = 2.0 * qw * qw + 2.0 * -qy * -qy - 1.0;
    bufItem.invRotMat[3 * 1 + 2] = 2.0 * -qy * -qz - 2.0 * -qx * qw;
    bufItem.invRotMat[3 * 2 + 0] = 2.0 * -qx * -qz - 2.0 * -qy * qw;
    bufItem.invRotMat[3 * 2 + 1] = 2.0 * -qy * qz + 2.0 * -qx * qw;
    bufItem.invRotMat[3 * 2 + 2] = 2.0 * qw * qw + 2.0 * -qz * -qz - 1.0;

    // calc vertex
    float basePnt[8][3] = {
      {-bufItem.size[0] / 2.0, bufItem.size[1] / 2.0, -bufItem.size[2] / 2.0},
      {bufItem.size[0] / 2.0, bufItem.size[1] / 2.0, -bufItem.size[2] / 2.0},
      {bufItem.size[0] / 2.0, bufItem.size[1] / 2.0, bufItem.size[2] / 2.0},
      {-bufItem.size[0] / 2.0, bufItem.size[1] / 2.0, bufItem.size[2] / 2.0},
      {-bufItem.size[0] / 2.0, -bufItem.size[1] / 2.0, -bufItem.size[2] / 2.0},
      {-bufItem.size[0] / 2.0, -bufItem.size[1] / 2.0, bufItem.size[2] / 2.0},
      {bufItem.size[0] / 2.0, -bufItem.size[1] / 2.0, bufItem.size[2] / 2.0},
      {bufItem.size[0] / 2.0, -bufItem.size[1] / 2.0, -bufItem.size[2] / 2.0}
    };
    for (int i = 0; i < 8; i++)
    {
      bufItem.vertex[i][0] = bufItem.rotMat[3 * 0 + 0] * basePnt[i][0]
        + bufItem.rotMat[3 * 0 + 1] * basePnt[i][1] 
        + bufItem.rotMat[3 * 0 + 2] * basePnt[i][2]
        + bufItem.center[0];
      bufItem.vertex[i][1] = bufItem.rotMat[3 * 1 + 0] * basePnt[i][0]
        + bufItem.rotMat[3 * 1 + 1] * basePnt[i][1] 
        + bufItem.rotMat[3 * 1 + 2] * basePnt[i][2]
        + bufItem.center[1];
      bufItem.vertex[i][2] = bufItem.rotMat[3 * 2 + 0] * basePnt[i][0]
        + bufItem.rotMat[3 * 2 + 1] * basePnt[i][1] 
        + bufItem.rotMat[3 * 2 + 2] * basePnt[i][2]
        + bufItem.center[2];
    }
    for (int i = 0; i < 3; i++)
    {
      bufItem.bbox[0][i] = min(bufItem.vertex[0][i], bufItem.vertex[1][i]);
      bufItem.bbox[0][i] = min(bufItem.bbox[0][i], bufItem.vertex[2][i]);
      bufItem.bbox[0][i] = min(bufItem.bbox[0][i], bufItem.vertex[3][i]);
      bufItem.bbox[0][i] = min(bufItem.bbox[0][i], bufItem.vertex[4][i]);
      bufItem.bbox[0][i] = min(bufItem.bbox[0][i], bufItem.vertex[5][i]);
      bufItem.bbox[0][i] = min(bufItem.bbox[0][i], bufItem.vertex[6][i]);
      bufItem.bbox[0][i] = min(bufItem.bbox[0][i], bufItem.vertex[7][i]);
      bufItem.bbox[1][i] = max(bufItem.vertex[1][i], bufItem.vertex[1][i]);
      bufItem.bbox[1][i] = max(bufItem.bbox[1][i], bufItem.vertex[2][i]);
      bufItem.bbox[1][i] = max(bufItem.bbox[1][i], bufItem.vertex[3][i]);
      bufItem.bbox[1][i] = max(bufItem.bbox[1][i], bufItem.vertex[4][i]);
      bufItem.bbox[1][i] = max(bufItem.bbox[1][i], bufItem.vertex[5][i]);
      bufItem.bbox[1][i] = max(bufItem.bbox[1][i], bufItem.vertex[6][i]);
      bufItem.bbox[1][i] = max(bufItem.bbox[1][i], bufItem.vertex[7][i]);
    }

    // calc face norm
    float baseFace[6][3] = {
      {0.0, 1.0, 0.0},
      {0.0, -1.0, 0.0},
      {-1.0, 0.0, 0.0},
      {0.0, 0.0, 1.0},
      {1.0, 0.0, 0.0},
      {0.0, 0.0, -1.0},
    };
    for (int i = 0; i < 6; i++)
    {
      bufItem.faceNorm[i][0] = bufItem.rotMat[3 * 0 + 0] * baseFace[i][0]
        + bufItem.rotMat[3 * 0 + 1] * baseFace[i][1] + bufItem.rotMat[3 * 0 + 2] * baseFace[i][2];
      bufItem.faceNorm[i][1] = bufItem.rotMat[3 * 1 + 0] * baseFace[i][0]
        + bufItem.rotMat[3 * 1 + 1] * baseFace[i][1] + bufItem.rotMat[3 * 1 + 2] * baseFace[i][2];
      bufItem.faceNorm[i][2] = bufItem.rotMat[3 * 2 + 0] * baseFace[i][0]
        + bufItem.rotMat[3 * 2 + 1] * baseFace[i][1] + bufItem.rotMat[3 * 2 + 2] * baseFace[i][2];
    }

    // type
    if ((pre.type == PathType::OuterWallStart && now.type == PathType::OuterWallMiddle) ||
      (pre.type == PathType::OuterWallMiddle && now.type == PathType::OuterWallEnd) ||
      (pre.type == PathType::OuterWallMiddle && now.type == PathType::OuterWallMiddle) ||
      (pre.type == PathType::OuterWallStart && now.type == PathType::OuterWallEnd))
    {
      bufItem.type = PrintType::OuterWall;
    }
    else if ((pre.type == PathType::InfillStart && now.type == PathType::InfillMiddle) ||
      (pre.type == PathType::InfillMiddle && now.type == PathType::InfillEnd) ||
      (pre.type == PathType::InfillMiddle && now.type == PathType::InfillMiddle) ||
      (pre.type == PathType::InfillStart && now.type == PathType::InfillEnd))
    {
      bufItem.type = PrintType::Infill;
    }
    else if ((pre.type == PathType::PrintStart && now.type == PathType::PrintMiddle) ||
      (pre.type == PathType::PrintMiddle && now.type == PathType::PrintEnd) ||
      (pre.type == PathType::PrintMiddle && now.type == PathType::PrintMiddle) ||
      (pre.type == PathType::PrintStart && now.type == PathType::PrintEnd))
    {
      bufItem.type = PrintType::Print;
    }
    else
    {
      bufItem.type = PrintType::Move;
    }
    bufItem.time = idx;

    // ray
    int baseRay[12][2] = {
      {1,0},
      {2,1},
      {3,2},
      {0,3},
      {5,4},
      {6,5},
      {7,6},
      {4,7},
      {4,0},
      {7,1},
      {6,2},
      {5,3}
    };
    for (int i = 0; i < 12; i++)
    {
      bufItem.ray[i].org[0] = bufItem.vertex[baseRay[i][0]][0];
      bufItem.ray[i].org[1] = bufItem.vertex[baseRay[i][0]][1];
      bufItem.ray[i].org[2] = bufItem.vertex[baseRay[i][0]][2];
      bufItem.ray[i].tar[0] = bufItem.vertex[baseRay[i][1]][0];
      bufItem.ray[i].tar[1] = bufItem.vertex[baseRay[i][1]][1];
      bufItem.ray[i].tar[2] = bufItem.vertex[baseRay[i][1]][2];
      bufItem.ray[i].id = idx;
    }

    // mesh
    int baseMesh[12][3] = {
      {0, 1, 2},
      {4, 5, 6},
      {0, 3, 5},
      {3, 2, 6},
      {2, 1, 7},
      {1, 7, 4},
      {0, 2, 3},
      {4, 6, 7},
      {0, 5, 4},
      {3, 6, 5},
      {2, 7, 6},
      {1, 4, 0}
    };
    for (int i = 0; i < 12; i++)
    {
      bufItem.mesh[i].pnt0[0] = bufItem.vertex[baseMesh[i][0]][0];
      bufItem.mesh[i].pnt0[1] = bufItem.vertex[baseMesh[i][0]][1];
      bufItem.mesh[i].pnt0[2] = bufItem.vertex[baseMesh[i][0]][2];
      bufItem.mesh[i].pnt1[0] = bufItem.vertex[baseMesh[i][1]][0];
      bufItem.mesh[i].pnt1[1] = bufItem.vertex[baseMesh[i][1]][1];
      bufItem.mesh[i].pnt1[2] = bufItem.vertex[baseMesh[i][1]][2];
      bufItem.mesh[i].pnt2[0] = bufItem.vertex[baseMesh[i][2]][0];
      bufItem.mesh[i].pnt2[1] = bufItem.vertex[baseMesh[i][2]][1];
      bufItem.mesh[i].pnt2[2] = bufItem.vertex[baseMesh[i][2]][2];
      bufItem.mesh[i].id = idx;
    }

    print[idx] = bufItem;
  }
}

void drawPrintItem_box(PrintItem item, float face_r, float face_g, float face_b,
  float edge_r, float edge_g, float edge_b)
{
  glEnable(GL_NORMALIZE);
  glColor3d(face_r, face_g, face_b);
  for (int i = 0; i < 6; i++)
  {
    glBegin(GL_QUADS);
    glNormal3f(item.faceNorm[i][0], item.faceNorm[i][1], item.faceNorm[i][2]);
    glVertex3f(item.vertex[BoxVertex[i][0]][0], item.vertex[BoxVertex[i][0]][1], item.vertex[BoxVertex[i][0]][2]);
    glVertex3f(item.vertex[BoxVertex[i][1]][0], item.vertex[BoxVertex[i][1]][1], item.vertex[BoxVertex[i][1]][2]);
    glVertex3f(item.vertex[BoxVertex[i][2]][0], item.vertex[BoxVertex[i][2]][1], item.vertex[BoxVertex[i][2]][2]);
    glVertex3f(item.vertex[BoxVertex[i][3]][0], item.vertex[BoxVertex[i][3]][1], item.vertex[BoxVertex[i][3]][2]);
    glEnd();
  }

  glColor3d(edge_r, edge_g, edge_b);
  glBegin(GL_LINES);
  glVertex3f(item.vertex[0][0], item.vertex[0][1], item.vertex[0][2]);
  glVertex3f(item.vertex[1][0], item.vertex[1][1], item.vertex[1][2]);
  glEnd();
  glBegin(GL_LINES);
  glVertex3f(item.vertex[2][0], item.vertex[2][1], item.vertex[2][2]);
  glVertex3f(item.vertex[3][0], item.vertex[3][1], item.vertex[3][2]);
  glEnd();
  glBegin(GL_LINES);
  glVertex3f(item.vertex[4][0], item.vertex[4][1], item.vertex[4][2]);
  glVertex3f(item.vertex[7][0], item.vertex[7][1], item.vertex[7][2]);
  glEnd();
  glBegin(GL_LINES);
  glVertex3f(item.vertex[5][0], item.vertex[5][1], item.vertex[5][2]);
  glVertex3f(item.vertex[6][0], item.vertex[6][1], item.vertex[6][2]);
  glEnd();
  glDisable(GL_NORMALIZE);
}

void drawPrintItem_move(PrintItem item, float move_r, float move_g, float move_b)
{
  glColor3d(move_r, move_g, move_b);
  glBegin(GL_LINES);
  glVertex3f(item.stPnt[0], item.stPnt[1], item.stPnt[2]);
  glVertex3f(item.edPnt[0], item.edPnt[1], item.edPnt[2]);
  glEnd();
}

typedef struct BVH_node {
  float bbox[2][3];  // bbox[0,1]: AABB の最小,最大座標． bbox[hoge][0,1,2]: x,y,z座標
  int children[2];    // 子ノード
  std::vector<PrintItem*> polygons;  // 格納されているポリゴン (葉ノードのみ有効)
}BvhNode;

float surfaceArea(const float bbox[2][3]) {
  float dx = bbox[1][0] - bbox[0][0];
  float dy = bbox[1][1] - bbox[0][1];
  float dz = bbox[1][2] - bbox[0][2];
  return 2 * (dx * dy + dx * dz + dy * dz);
}

void emptyAABB(float bbox[2][3]) {
  bbox[0][0] = bbox[0][1] = bbox[0][2] = FLT_MAX;
  bbox[1][0] = bbox[1][1] = bbox[1][2] = FLT_MIN;
}

void mergeAABB(const float bbox1[2][3], const float bbox2[2][3], float result[2][3]) {
  for (int j = 0; j < 3; j++) {
    result[0][j] = min(bbox1[0][j], bbox2[0][j]);
    result[1][j] = max(bbox1[1][j], bbox2[1][j]);
  }
}

void createAABBfromTriangles(const std::vector<PrintItem*>& triangles, float bbox[2][3]) {
  emptyAABB(bbox);
  for_each(triangles.begin(), triangles.end(), [&bbox](const PrintItem* t) {
    mergeAABB(t->bbox, bbox, bbox);
    });
}

BVH_node nodes[BVH_NODES_CNT];
int used_node_count = 0;
float T_tri = 1.0;
float T_aabb = 1.0;

void makeLeaf(std::vector<PrintItem*>& polygons, BVH_node* node) {
  node->children[0] = node->children[1] = -1;
  node->polygons = polygons;
  createAABBfromTriangles(polygons, node->bbox);
}

void constructBVH_internal(std::vector<PrintItem*> &items, int nodeIndex)
{  
  if (items.size() < 500)
  {
    BvhNode* node = &nodes[nodeIndex];
    makeLeaf(items, node);
  }
  else
  {
    BvhNode* node = &nodes[nodeIndex];
    createAABBfromTriangles(items, node->bbox);
    float tTri = 1.0;
    float tAabb = 1.0;
    float bestCost = tTri * items.size();
    int bestAxis = -1;
    int bestSplitIndex = -1;
    float saRoot = surfaceArea(node->bbox);

    for (int axis = 0; axis < 3; axis++)
    {
      auto comp = [axis](const PrintItem* a, const PrintItem* b)
        {
          return a->center[axis] < b->center[axis];
        };
      std::sort(items.begin(), items.end(), comp);

      std::vector<PrintItem*> s1;
      std::vector<PrintItem*> s2;
      for (int i = 0; i < items.size(); i++)
      {
        s2.push_back(items[i]);
      }

      std::vector<float> s1SA;
      std::vector<float> s2SA;
      for (int i = 0; i < items.size() + 1; i++)
      {
        s1SA.push_back(FLT_MAX);
        s2SA.push_back(FLT_MAX);
      }

      float s1bbox[2][3];
      emptyAABB(s1bbox);
      for (int i = 0; i < items.size(); i++)
      {
        s1SA[i] = abs(surfaceArea(s1bbox));
        if (s2.size() > 0)
        {
          PrintItem* p = s2.front();
          s1.push_back(p);
          s2.erase(s2.begin());
          mergeAABB(s1bbox, p->bbox, s1bbox);
        }
      }

      float s2bbox[2][3];
      emptyAABB(s2bbox);
      for (int i = items.size() - 1; i >= 0; i--)
      {
        s2SA[i] = abs(surfaceArea(s2bbox));
        if (s1.size() > 0 && s2.size() > 0)
        {
          float cost = 2.0 * tAabb + (s1SA[i] * s1.size() + s2SA[i] * s2.size()) * tTri / saRoot;
          if (cost < bestCost)
          {
            bestCost = cost;
            bestAxis = axis;
            bestSplitIndex = i;
          }
        }
        if (s1.size() > 0)
        {
          PrintItem* p = s1.back();
          s2.insert(s2.begin(), p);
          s1.pop_back();
          mergeAABB(s2bbox, p->bbox, s2bbox);
        }
      }
    }
    if (bestAxis == -1)
    {
      makeLeaf(items, node);
    }
    else
    {
      auto comp = [bestAxis](const PrintItem* a, const PrintItem* b)
        {
          return a->center[bestAxis] < b->center[bestAxis];
        };
      std::sort(items.begin(), items.end(), comp);
      used_node_count++;
      node->children[0] = used_node_count;
      used_node_count++;
      node->children[1] = used_node_count;
      std::vector<PrintItem*> left(items.begin(), items.begin() + bestSplitIndex);
      std::vector<PrintItem*> right(items.begin() + bestSplitIndex, items.end());
      constructBVH_internal(left, node->children[0]);
      constructBVH_internal(right, node->children[1]);
    }
  }
}

void constructBVH(std::vector<PrintItem*>& input, 
  std::vector<std::vector<PrintItem*>>* outputPrint, 
  std::vector<Ray*>* outputRay, std::vector<Mesh*>* outputMesh)
{
  used_node_count = 0;
  std::vector<PrintItem*> bufInput = std::vector<PrintItem*>();
  for (int i = 0; i < input.size(); i++)
  {
    if (input[i]->type == PrintType::Print || input[i]->type == PrintType::OuterWall ||
      input[i]->type == PrintType::InnerWall || input[i]->type == PrintType::Infill ||
      input[i]->type == PrintType::Support)
    {
      bufInput.push_back(input[i]);
    }
  }

  constructBVH_internal(bufInput, 0);
  int nodeIdx = -1;
  if (used_node_count > 0)
  {
    for (int i = 0; i < used_node_count; i++)
    {
      if (nodes[i].children[0] == -1 && nodes[i].children[1] == -1)
      {
        nodeIdx++;
        outputPrint->push_back(nodes[i].polygons);
        float basePnt[8][3] = {
          {nodes[i].bbox[0][0], nodes[i].bbox[1][1], nodes[i].bbox[0][2] },
          {nodes[i].bbox[1][0], nodes[i].bbox[1][1], nodes[i].bbox[0][2] },
          {nodes[i].bbox[1][0], nodes[i].bbox[1][1], nodes[i].bbox[1][2] },
          {nodes[i].bbox[0][0], nodes[i].bbox[1][1], nodes[i].bbox[1][2] },
          {nodes[i].bbox[0][0], nodes[i].bbox[0][1], nodes[i].bbox[0][2] },
          {nodes[i].bbox[0][0], nodes[i].bbox[0][1], nodes[i].bbox[1][2] },
          {nodes[i].bbox[1][0], nodes[i].bbox[0][1], nodes[i].bbox[1][2] },
          {nodes[i].bbox[1][0], nodes[i].bbox[0][1], nodes[i].bbox[0][2] }
        };
        Ray* rays = new Ray[12];
        int baseRay[12][2] = {
          {1,0},
          {2,1},
          {3,2},
          {0,3},
          {5,4},
          {6,5},
          {7,6},
          {4,7},
          {4,0},
          {7,1},
          {6,2},
          {5,3}
        };
        for (int j = 0; j < 12; j++)
        {
          Ray buf;
          buf.org[0] = basePnt[baseRay[j][0]][0];
          buf.org[1] = basePnt[baseRay[j][0]][1];
          buf.org[2] = basePnt[baseRay[j][0]][2];
          buf.tar[0] = basePnt[baseRay[j][1]][0];
          buf.tar[1] = basePnt[baseRay[j][1]][1];
          buf.tar[2] = basePnt[baseRay[j][1]][2];
          buf.id = nodeIdx;
          rays[j] = buf;
        }
        outputRay->push_back(rays);
        Mesh* meshs = new Mesh[12];
        int baseMesh[12][3] = {
          {0, 1, 2},
          {4, 5, 6},
          {0, 3, 5},
          {3, 2, 6},
          {2, 1, 7},
          {1, 7, 4},
          {0, 2, 3},
          {4, 6, 7},
          {0, 5, 4},
          {3, 6, 5},
          {2, 7, 6},
          {1, 4, 0}
        };
        for (int j = 0; j < 12; j++)
        {
          Mesh buf;
          buf.pnt0[0] = basePnt[baseMesh[j][0]][0];
          buf.pnt0[1] = basePnt[baseMesh[j][0]][1];
          buf.pnt0[2] = basePnt[baseMesh[j][0]][2];
          buf.pnt1[0] = basePnt[baseMesh[j][1]][0];
          buf.pnt1[1] = basePnt[baseMesh[j][1]][1];
          buf.pnt1[2] = basePnt[baseMesh[j][1]][2];
          buf.pnt2[0] = basePnt[baseMesh[j][2]][0];
          buf.pnt2[1] = basePnt[baseMesh[j][2]][1];
          buf.pnt2[2] = basePnt[baseMesh[j][2]][2];
          buf.id = nodeIdx;
          meshs[j] = buf;
        }
        outputMesh->push_back(meshs);
      }
    }
  }
}


