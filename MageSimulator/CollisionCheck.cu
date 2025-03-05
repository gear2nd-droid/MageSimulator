#include "CollisionCheck.cuh"



cudaError_t checkCollisions(int cnt, Mesh* mesh, Ray* ray, CollisionSet* col)
{
  if (cnt > 0)
  {
    cudaError_t cudaStatus;
    Mesh* dev_mesh;
    Ray* dev_ray;
    CollisionSet* dev_col;

    // Choose which GPU to run on, change this on a multi-GPU system.
    //cudaStatus = cudaSetDevice(0);
    //if (cudaStatus != cudaSuccess) {
    //  fprintf(stderr, "cudaSetDevice failed!  Do you have a CUDA-capable GPU installed?");
    //  goto Error4;
    //}

    // Allocate GPU buffers for three vectors (two input, one output)    .
    cudaStatus = cudaMalloc((void**)&dev_mesh, cnt * sizeof(Mesh));
    if (cudaStatus != cudaSuccess) {
      fprintf(stderr, "cudaMalloc failed_1!");
      goto Error3;
    }
    cudaStatus = cudaMalloc((void**)&dev_ray, cnt * sizeof(Ray));
    if (cudaStatus != cudaSuccess) {
      fprintf(stderr, "cudaMalloc failed_2!");
      goto Error2;
    }
    cudaStatus = cudaMalloc((void**)&dev_col, cnt * sizeof(CollisionSet));
    if (cudaStatus != cudaSuccess) {
      fprintf(stderr, "cudaMalloc failed_3!");
      goto Error1;
    }

    // create stream
    cudaStream_t stream;
    cudaStreamCreate(&stream);

    // Copy input vectors from host memory to GPU buffers.
    //cudaStatus = cudaMemcpy(dev_mesh, mesh, cnt * sizeof(Mesh), cudaMemcpyHostToDevice);
    cudaStatus = cudaMemcpyAsync(dev_mesh, mesh, cnt * sizeof(Mesh), cudaMemcpyHostToDevice, stream);
    if (cudaStatus != cudaSuccess) {
      fprintf(stderr, "cudaMemcpy failed_1!"); 
      goto Error1;
    }
    //cudaStatus = cudaMemcpy(dev_ray, ray, cnt * sizeof(Ray), cudaMemcpyHostToDevice);
    cudaStatus = cudaMemcpyAsync(dev_ray, ray, cnt * sizeof(Ray), cudaMemcpyHostToDevice, stream);
    if (cudaStatus != cudaSuccess) {
      fprintf(stderr, "cudaMemcpy failed_2!");
      goto Error1;
    }

    // Launch a kernel on the GPU with one thread for each element.
    checkCollision << <(cnt + 1024 -1) / 1024, 1024 >> > (cnt, dev_mesh, dev_ray, dev_col);

    // Check for any errors launching the kernel
    cudaStatus = cudaGetLastError();
    if (cudaStatus != cudaSuccess) {
      fprintf(stderr, "addKernel launch failed: %s\n", cudaGetErrorString(cudaStatus));
      goto Error1;
    }

    // cudaDeviceSynchronize waits for the kernel to finish, and returns
    // any errors encountered during the launch.
    //cudaStatus = cudaDeviceSynchronize();
    //if (cudaStatus != cudaSuccess) {
    //  fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching addKernel!\n", cudaStatus);
    //  
    //  cudaError_t err = cudaGetLastError();
    //  if (err != cudaSuccess) {
    //    printf("CUDA error: %s\n", cudaGetErrorString(err));
    //  }
    //  
    //  goto Error1;
    //}

    // Copy output vector from GPU buffer to host memory.
    //cudaStatus = cudaMemcpy(col, dev_col, cnt * sizeof(CollisionSet), cudaMemcpyDeviceToHost);
    //cudaEvent_t start, stop;
    //cudaEventCreate(&start);
    //cudaEventCreate(&stop);
    //cudaEventRecord(start, 0);
    cudaStatus = cudaMemcpyAsync(col, dev_col, cnt * sizeof(CollisionSet), cudaMemcpyDeviceToHost, stream);
    if (cudaStatus != cudaSuccess) {
      fprintf(stderr, "cudaMemcpy failed!");
      goto Error1;
    }
    //cudaEventRecord(stop, 0);
    // delete stream
    cudaStatus = cudaStreamSynchronize(stream);
    if (cudaStatus != cudaSuccess) {
      fprintf(stderr, "cudaStreamSynchronize returned error code %d after launching addKernel!\n", cudaStatus);
      
      cudaError_t err = cudaGetLastError();
      if (err != cudaSuccess) {
        printf("CUDA error: %s\n", cudaGetErrorString(err));
      }
      
      goto Error1;
    }

    cudaStreamDestroy(stream);
    //cudaEventSynchronize(stop);
    //float elapsedTime;
    //cudaEventElapsedTime(&elapsedTime, start, stop);
    //float bandwidth = (cnt * sizeof(CollisionSet) / elapsedTime) * 1e3;
    //printf("%lf\n", bandwidth);

  Error1:
    cudaFree(dev_col);
  Error2:
    cudaFree(dev_ray);
  Error3:
    cudaFree(dev_mesh);
  Error4:
    return cudaStatus;
  }
  else
  {
    return cudaSuccess;
  }
}

__device__ float det(float a[3], float b[3], float c[3])
{
  return (a[0] * b[1] * c[2] + a[1] * b[2] * c[0] + a[2] * b[0] * c[1]
    - a[0] * b[2] * c[1] - a[1] * b[0] * c[2] - a[2] * b[1] * c[0]);
}

//https://shikousakugo.wordpress.com/2012/07/01/ray-intersection-3/
__global__ void checkCollision(int cnt, Mesh* _mesh, Ray* _ray, CollisionSet* _col)
{
  int idx = blockDim.x * blockIdx.x + threadIdx.x;
  if (idx < cnt)
  {
    Mesh targetMesh = _mesh[idx];
    Ray targetRay = _ray[idx];
    _col[idx].meshId = targetMesh.id;
    _col[idx].rayId = targetRay.id;
    _col[idx].colFlag = false;

    // calc
    float ray[3];
    ray[0] = targetRay.tar[0] - targetRay.org[0];
    ray[1] = targetRay.tar[1] - targetRay.org[1];
    ray[2] = targetRay.tar[2] - targetRay.org[2];
    float rayLen = sqrt(ray[0] * ray[0] + ray[1] * ray[1] + ray[2] * ray[2]);
    float edge1[3];
    edge1[0] = targetMesh.pnt1[0] - targetMesh.pnt0[0];
    edge1[1] = targetMesh.pnt1[1] - targetMesh.pnt0[1];
    edge1[2] = targetMesh.pnt1[2] - targetMesh.pnt0[2];
    float edge1Len = sqrt(edge1[0] * edge1[0] + edge1[1] * edge1[1] + edge1[2] * edge1[2]);
    float edge2[3];
    edge2[0] = targetMesh.pnt2[0] - targetMesh.pnt0[0];
    edge2[1] = targetMesh.pnt2[1] - targetMesh.pnt0[1];
    edge2[2] = targetMesh.pnt2[2] - targetMesh.pnt0[2];
    float edge2Len = sqrt(edge2[0] * edge2[0] + edge2[1] * edge2[1] + edge2[2] * edge2[2]);
    float p[3];
    p[0] = ray[1] * edge2[2] - ray[2] * edge2[1];
    p[1] = ray[2] * edge2[0] - ray[0] * edge2[2];
    p[2] = ray[0] * edge2[1] - ray[1] * edge2[0];

    float det = p[0] * edge1[0] + p[1] * edge1[1] + p[2] * edge1[2];
    if (det > EPSION)
    {
      float vt[3];
      vt[0] = targetRay.org[0] - targetMesh.pnt0[0];
      vt[1] = targetRay.org[1] - targetMesh.pnt0[1];
      vt[2] = targetRay.org[2] - targetMesh.pnt0[2];
      float u = p[0] * vt[0] + p[1] * vt[1] + p[2] * vt[2];
      if (u >= 0.0 && u <= 1.0 * det)
      {
        float q[3];
        q[0] = vt[1] * edge1[2] - vt[2] * edge1[1];
        q[1] = vt[2] * edge1[0] - vt[0] * edge1[2];
        q[2] = vt[0] * edge1[1] - vt[1] * edge1[0];
        float v = q[0] * ray[0] + q[1] * ray[1] + q[2] * ray[2];
        if (v >= 0.0 && (u + v) <= 1.0 * det)
        {
          float t = (q[0] * edge2[0] + q[1] * edge2[1] + q[2] * edge2[2]) / det;
          float pnt[3];
          u /= det;
          v /= det;
          pnt[0] = edge1[0] * u + edge2[0] * v + targetMesh.pnt0[0];
          pnt[1] = edge1[1] * u + edge2[1] * v + targetMesh.pnt0[1];
          pnt[2] = edge1[2] * u + edge2[2] * v + targetMesh.pnt0[2];
          float dist1 = sqrt((pnt[0] - targetRay.org[0]) * (pnt[0] - targetRay.org[0]) +
            (pnt[1] - targetRay.org[1]) * (pnt[1] - targetRay.org[1]) + 
            (pnt[2] - targetRay.org[2]) * (pnt[2] - targetRay.org[2]));
          if (t >= 0.0 && t <= 1.0)
          {
            // distance
            float cross[3];
            cross[0] = edge1[1] * edge2[2] - edge1[2] * edge2[1];
            cross[1] = edge1[2] * edge2[0] - edge1[0] * edge2[2];
            cross[2] = edge1[0] * edge2[1] - edge1[1] * edge2[0];
            float d = -cross[0] * targetMesh.pnt0[0] - cross[1] * targetMesh.pnt0[1] - cross[2] * targetMesh.pnt0[2];
            float dist2 = (cross[0] * pnt[0] + cross[1] * pnt[1] + cross[2] * pnt[2] + d) /
              sqrt(cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]);
            // output
            _col[idx].colFlag = true;
            _col[idx].pnt[0] = pnt[0];
            _col[idx].pnt[1] = pnt[1];
            _col[idx].pnt[2] = pnt[2];
          }
          else
          {
            // calc
            ray[0] = targetRay.org[0] - targetRay.tar[0];
            ray[1] = targetRay.org[1] - targetRay.tar[1];
            ray[2] = targetRay.org[2] - targetRay.tar[2];
            rayLen = sqrt(ray[0] * ray[0] + ray[1] * ray[1] + ray[2] * ray[2]);
            edge1[0] = targetMesh.pnt1[0] - targetMesh.pnt0[0];
            edge1[1] = targetMesh.pnt1[1] - targetMesh.pnt0[1];
            edge1[2] = targetMesh.pnt1[2] - targetMesh.pnt0[2];
            edge1Len = sqrt(edge1[0] * edge1[0] + edge1[1] * edge1[1] + edge1[2] * edge1[2]);
            edge2[0] = targetMesh.pnt2[0] - targetMesh.pnt0[0];
            edge2[1] = targetMesh.pnt2[1] - targetMesh.pnt0[1];
            edge2[2] = targetMesh.pnt2[2] - targetMesh.pnt0[2];
            edge2Len = sqrt(edge2[0] * edge2[0] + edge2[1] * edge2[1] + edge2[2] * edge2[2]);
            p[0] = ray[1] * edge2[2] - ray[2] * edge2[1];
            p[1] = ray[2] * edge2[0] - ray[0] * edge2[2];
            p[2] = ray[0] * edge2[1] - ray[1] * edge2[0];

            det = p[0] * edge1[0] + p[1] * edge1[1] + p[2] * edge1[2];
            if (det > EPSION)
            {
              vt[0] = targetRay.tar[0] - targetMesh.pnt0[0];
              vt[1] = targetRay.tar[1] - targetMesh.pnt0[1];
              vt[2] = targetRay.tar[2] - targetMesh.pnt0[2];
              u = p[0] * vt[0] + p[1] * vt[1] + p[2] * vt[2];
              if (u >= 0.0 && u <= 1.0 * det)
              {
                q[0] = vt[1] * edge1[2] - vt[2] * edge1[1];
                q[1] = vt[2] * edge1[0] - vt[0] * edge1[2];
                q[2] = vt[0] * edge1[1] - vt[1] * edge1[0];
                v = q[0] * ray[0] + q[1] * ray[1] + q[2] * ray[2];
                if (v >= 0.0 && (u + v) <= 1.0 * det)
                {
                  t = (q[0] * edge2[0] + q[1] * edge2[1] + q[2] * edge2[2]) / det;
                  u /= det;
                  v /= det;
                  pnt[0] = edge1[0] * u + edge2[0] * v + targetMesh.pnt0[0];
                  pnt[1] = edge1[1] * u + edge2[1] * v + targetMesh.pnt0[1];
                  pnt[2] = edge1[2] * u + edge2[2] * v + targetMesh.pnt0[2];
                  dist1 = sqrt((pnt[0] - targetRay.tar[0]) * (pnt[0] - targetRay.tar[0]) +
                    (pnt[1] - targetRay.tar[1]) * (pnt[1] - targetRay.tar[1]) +
                    (pnt[2] - targetRay.tar[2]) * (pnt[2] - targetRay.tar[2]));
                  if (t >= 0.0 && t <= 1.0)
                  {
                    // distance
                    float cross[3];
                    cross[0] = edge1[1] * edge2[2] - edge1[2] * edge2[1];
                    cross[1] = edge1[2] * edge2[0] - edge1[0] * edge2[2];
                    cross[2] = edge1[0] * edge2[1] - edge1[1] * edge2[0];
                    float d = -cross[0] * targetMesh.pnt0[0] - cross[1] * targetMesh.pnt0[1] - cross[2] * targetMesh.pnt0[2];
                    float dist2 = (cross[0] * pnt[0] + cross[1] * pnt[1] + cross[2] * pnt[2] + d) /
                      sqrt(cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]);
                    // output
                    _col[idx].colFlag = true;
                    _col[idx].pnt[0] = pnt[0];
                    _col[idx].pnt[1] = pnt[1];
                    _col[idx].pnt[2] = pnt[2];
                  }
                }
              }
            }
          }
        }
      }
    }
  }
}

cudaError_t createLargeArrays(std::vector<Mesh> inMesh, std::vector<Ray> inRay, Mesh* outMesh, Ray* outRay)
{
  cudaError_t cudaStatus;
  Mesh* dev_mesh;
  Ray* dev_ray;
  Mesh* dev_mat_mesh;
  Ray* dev_mat_ray;
  
  // input
  //Mesh* mesh = new Mesh[inMesh.size()];
  Mesh* mesh = inMesh.data();
  //for (int i = 0; i < inMesh.size(); i++)
  //{
  //  mesh[i] = inMesh[i];
  //}
  //Ray* ray = new Ray[inRay.size()];
  Ray* ray = inRay.data();
  //for (int i = 0; i < inRay.size(); i++)
  //{
  //  ray[i] = inRay[i];
  //}

  // cuda
  cudaStatus = cudaMalloc((void**)&dev_mesh, inMesh.size() * sizeof(Mesh));
  if (cudaStatus != cudaSuccess) {
    fprintf(stderr, "cudaMalloc failed_1!");
    goto Error4;
  }
  cudaStatus = cudaMalloc((void**)&dev_ray, inRay.size() * sizeof(Ray));
  if (cudaStatus != cudaSuccess) {
    fprintf(stderr, "cudaMalloc failed_2!");
    goto Error3;
  }
  cudaStatus = cudaMalloc((void**)&dev_mat_mesh, inMesh.size() * inRay.size() * sizeof(Mesh));
  if (cudaStatus != cudaSuccess) {
    fprintf(stderr, "cudaMalloc failed_1!");
    goto Error2;
  }
  cudaStatus = cudaMalloc((void**)&dev_mat_ray, inMesh.size() * inRay.size() * sizeof(Ray));
  if (cudaStatus != cudaSuccess) {
    fprintf(stderr, "cudaMalloc failed_2!");
    goto Error1;
  }

  // create stream
  cudaStream_t stream;
  cudaStreamCreate(&stream);

  // Copy input vectors from host memory to GPU buffers.
  //cudaStatus = cudaMemcpy(dev_mesh, mesh, cnt * sizeof(Mesh), cudaMemcpyHostToDevice);
  cudaStatus = cudaMemcpyAsync(dev_mesh, mesh, inMesh.size() * sizeof(Mesh), cudaMemcpyHostToDevice, stream);
  if (cudaStatus != cudaSuccess) {
    fprintf(stderr, "cudaMemcpy failed_1!");
    goto Error1;
  }
  //cudaStatus = cudaMemcpy(dev_ray, ray, cnt * sizeof(Ray), cudaMemcpyHostToDevice);
  cudaStatus = cudaMemcpyAsync(dev_ray, ray, inRay.size() * sizeof(Ray), cudaMemcpyHostToDevice, stream);
  if (cudaStatus != cudaSuccess) {
    fprintf(stderr, "cudaMemcpy failed_2!");
    goto Error1;
  }

  // Launch a kernel on the GPU with one thread for each element.
  createLargeArray << <(inMesh.size() * inRay.size() + 1024 - 1) / 1024, 1024 >> > (inMesh.size(), inRay.size(),
    dev_mesh, dev_ray, dev_mat_mesh, dev_mat_ray);

  // Check for any errors launching the kernel
  cudaStatus = cudaGetLastError();
  if (cudaStatus != cudaSuccess) {
    fprintf(stderr, "addKernel launch failed: %s\n", cudaGetErrorString(cudaStatus));
    goto Error1;
  }

  // mem copy
  cudaStatus = cudaMemcpyAsync(outMesh, dev_mat_mesh, inMesh.size() * inRay.size() * sizeof(Mesh), cudaMemcpyDeviceToHost, stream);
  if (cudaStatus != cudaSuccess) {
    fprintf(stderr, "cudaMemcpy failed!");
    goto Error1;
  }
  cudaStatus = cudaMemcpyAsync(outRay, dev_mat_ray, inMesh.size() * inRay.size() * sizeof(Ray), cudaMemcpyDeviceToHost, stream);
  if (cudaStatus != cudaSuccess) {
    fprintf(stderr, "cudaMemcpy failed!");
    goto Error1;
  }

  // stream close
  cudaStatus = cudaStreamSynchronize(stream);
  if (cudaStatus != cudaSuccess) {
    fprintf(stderr, "cudaStreamSynchronize returned error code %d after launching addKernel!\n", cudaStatus);

    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess) {
      printf("CUDA error: %s\n", cudaGetErrorString(err));
    }
    goto Error1;
  }
  cudaStreamDestroy(stream);

  //delete[] mesh;
  //delete[] ray;
Error1:
  cudaFree(dev_mat_ray);
Error2:
  cudaFree(dev_mat_mesh);
Error3:
  cudaFree(dev_ray);
Error4:
  cudaFree(dev_mesh);
  return cudaStatus;
}

__global__ void createLargeArray(int meshSize, int raySize, Mesh* inMesh, Ray* inRay, Mesh* outMesh, Ray* outRay)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  int ix = idx % meshSize;
  int iy = idx / meshSize;
  //int ix = blockIdx.x * blockDim.x + threadIdx.x;
  //int iy = blockIdx.y * blockDim.y + threadIdx.y;
  if (ix < meshSize && iy < raySize)
  {
    //int idx = iy * meshSize + ix;
    outMesh[idx] = inMesh[ix];
    outRay[idx] = inRay[iy];
  }
}