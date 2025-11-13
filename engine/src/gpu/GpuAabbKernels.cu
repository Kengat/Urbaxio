#include "engine/gpu/GpuAabbKernels.h"
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <iostream>
#include <limits>

namespace Urbaxio::Engine {

// Helper: Atomic min for float (CUDA doesn't have native atomicMin for float)
__device__ __forceinline__ void atomicMinFloat(float* addr, float value) {
    int* addr_as_int = (int*)addr;
    int old = *addr_as_int, assumed;
    do {
        assumed = old;
        old = atomicCAS(addr_as_int, assumed,
                        __float_as_int(fminf(value, __int_as_float(assumed))));
    } while (assumed != old);
}

// Helper: Atomic max for float
__device__ __forceinline__ void atomicMaxFloat(float* addr, float value) {
    int* addr_as_int = (int*)addr;
    int old = *addr_as_int, assumed;
    do {
        assumed = old;
        old = atomicCAS(addr_as_int, assumed,
                        __float_as_int(fmaxf(value, __int_as_float(assumed))));
    } while (assumed != old);
}

__global__ void ComputeAabbKernel(
    const float* __restrict__ d_vertices,
    size_t vertexCount,
    float* d_minX, float* d_minY, float* d_minZ,
    float* d_maxX, float* d_maxY, float* d_maxZ)
{
    size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= vertexCount) return;
    
    // Read vertex
    float x = d_vertices[idx * 3 + 0];
    float y = d_vertices[idx * 3 + 1];
    float z = d_vertices[idx * 3 + 2];
    
    // Atomic updates to global min/max
    atomicMinFloat(d_minX, x);
    atomicMinFloat(d_minY, y);
    atomicMinFloat(d_minZ, z);
    atomicMaxFloat(d_maxX, x);
    atomicMaxFloat(d_maxY, y);
    atomicMaxFloat(d_maxZ, z);
}

bool GpuAabbKernels::ComputeAabb(
    const float* d_vertices,
    size_t vertexCount,
    glm::vec3& outMin,
    glm::vec3& outMax,
    void* stream)
{
    if (!d_vertices || vertexCount == 0) return false;
    
    cudaStream_t cudaStream = static_cast<cudaStream_t>(stream);
    
    // Allocate device memory for results (6 floats)
    float h_init_min = std::numeric_limits<float>::max();
    float h_init_max = std::numeric_limits<float>::lowest();
    
    float *d_minX, *d_minY, *d_minZ, *d_maxX, *d_maxY, *d_maxZ;
    cudaMalloc(&d_minX, sizeof(float));
    cudaMalloc(&d_minY, sizeof(float));
    cudaMalloc(&d_minZ, sizeof(float));
    cudaMalloc(&d_maxX, sizeof(float));
    cudaMalloc(&d_maxY, sizeof(float));
    cudaMalloc(&d_maxZ, sizeof(float));
    
    // Initialize with extremes
    cudaMemcpyAsync(d_minX, &h_init_min, sizeof(float), cudaMemcpyHostToDevice, cudaStream);
    cudaMemcpyAsync(d_minY, &h_init_min, sizeof(float), cudaMemcpyHostToDevice, cudaStream);
    cudaMemcpyAsync(d_minZ, &h_init_min, sizeof(float), cudaMemcpyHostToDevice, cudaStream);
    cudaMemcpyAsync(d_maxX, &h_init_max, sizeof(float), cudaMemcpyHostToDevice, cudaStream);
    cudaMemcpyAsync(d_maxY, &h_init_max, sizeof(float), cudaMemcpyHostToDevice, cudaStream);
    cudaMemcpyAsync(d_maxZ, &h_init_max, sizeof(float), cudaMemcpyHostToDevice, cudaStream);
    
    // Launch kernel
    int threadsPerBlock = 256;
    int blocksPerGrid = static_cast<int>((vertexCount + threadsPerBlock - 1) / threadsPerBlock);
    
    ComputeAabbKernel<<<blocksPerGrid, threadsPerBlock, 0, cudaStream>>>(
        d_vertices, vertexCount,
        d_minX, d_minY, d_minZ,
        d_maxX, d_maxY, d_maxZ
    );
    
    // Copy results back (only 24 bytes!)
    float h_min[3], h_max[3];
    cudaMemcpyAsync(h_min + 0, d_minX, sizeof(float), cudaMemcpyDeviceToHost, cudaStream);
    cudaMemcpyAsync(h_min + 1, d_minY, sizeof(float), cudaMemcpyDeviceToHost, cudaStream);
    cudaMemcpyAsync(h_min + 2, d_minZ, sizeof(float), cudaMemcpyDeviceToHost, cudaStream);
    cudaMemcpyAsync(h_max + 0, d_maxX, sizeof(float), cudaMemcpyDeviceToHost, cudaStream);
    cudaMemcpyAsync(h_max + 1, d_maxY, sizeof(float), cudaMemcpyDeviceToHost, cudaStream);
    cudaMemcpyAsync(h_max + 2, d_maxZ, sizeof(float), cudaMemcpyDeviceToHost, cudaStream);
    
    cudaStreamSynchronize(cudaStream);
    
    // Free temp memory
    cudaFree(d_minX); cudaFree(d_minY); cudaFree(d_minZ);
    cudaFree(d_maxX); cudaFree(d_maxY); cudaFree(d_maxZ);
    
    // Check for errors
    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess) {
        std::cerr << "[GpuAabbKernels] Kernel failed: "
                  << cudaGetErrorString(err) << std::endl;
        return false;
    }
    
    outMin = glm::vec3(h_min[0], h_min[1], h_min[2]);
    outMax = glm::vec3(h_max[0], h_max[1], h_max[2]);
    
    return true;
}

} // namespace Urbaxio::Engine


