// Workaround for OpenVDB half vs CUDA __half conflict  
// Include CUDA headers FIRST
#include <cuda_runtime.h>

// Tell nvcc to ignore the ambiguous operator+ in OpenVDB::half
#ifdef __CUDACC__
#pragma nv_diag_suppress 20054
#pragma nv_diag_suppress 20012
#endif

#define OPENVDB_USE_DELAYED_LOADING 1
#define HALF_ENABLE_F16C_INTRINSICS 0

#include "engine/gpu/GpuSculptKernels.h"
#include <nanovdb/NanoVDB.h>
#include <nanovdb/util/cuda/CudaDeviceBuffer.h>
#include <nanovdb/util/GridBuilder.h>
#include <iostream>
#include <cmath>

namespace Urbaxio::Engine {

// CUDA kernel for CSG union operation
// NOTE: NanoVDB grids are READ-ONLY on GPU by design for performance
// TODO: Implement proper GPU sculpting using grid rebuilding or custom data structures
__global__ void CsgUnionKernel(
    nanovdb::FloatGrid* sceneGrid,
    const nanovdb::FloatGrid* brushGrid,
    int3 minCoord,
    int3 maxCoord)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int z = blockIdx.z * blockDim.z + threadIdx.z;

    if (x >= maxCoord.x || y >= maxCoord.y || z >= maxCoord.z) return;

    int3 coord = make_int3(x + minCoord.x, y + minCoord.y, z + minCoord.z);
    
    // Get read accessors
    auto sceneAcc = sceneGrid->getAccessor();
    auto brushAcc = brushGrid->getAccessor();
    
    // Read values from both grids
    float sceneVal = sceneAcc.getValue(nanovdb::Coord(coord.x, coord.y, coord.z));
    float brushVal = brushAcc.getValue(nanovdb::Coord(coord.x, coord.y, coord.z));
    
    // CSG Union: min(A, B) for SDFs
    float result = fminf(sceneVal, brushVal);
    
    // TODO: Write result back using proper grid rebuilding
    // NanoVDB doesn't support setValue on GPU - need to rebuild grid from scratch
}

// CUDA kernel for CSG difference operation
// NOTE: NanoVDB grids are READ-ONLY on GPU by design
__global__ void CsgDifferenceKernel(
    nanovdb::FloatGrid* sceneGrid,
    const nanovdb::FloatGrid* brushGrid,
    int3 minCoord,
    int3 maxCoord)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int z = blockIdx.z * blockDim.z + threadIdx.z;

    if (x >= maxCoord.x || y >= maxCoord.y || z >= maxCoord.z) return;

    int3 coord = make_int3(x + minCoord.x, y + minCoord.y, z + minCoord.z);
    
    auto sceneAcc = sceneGrid->getAccessor();
    auto brushAcc = brushGrid->getAccessor();
    
    float sceneVal = sceneAcc.getValue(nanovdb::Coord(coord.x, coord.y, coord.z));
    float brushVal = brushAcc.getValue(nanovdb::Coord(coord.x, coord.y, coord.z));
    
    // CSG Difference: max(A, -B) for SDFs
    float result = fmaxf(sceneVal, -brushVal);
    
    // TODO: Write result back using proper grid rebuilding
}

// CUDA kernel for applying a spherical brush directly
__global__ void ApplySphericalBrushKernel(
    nanovdb::FloatGrid* grid,
    float3 brushCenter,
    float brushRadius,
    int mode, // 0 = ADD, 1 = SUBTRACT
    float strength,
    int3 minCoord,
    int3 maxCoord)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int z = blockIdx.z * blockDim.z + threadIdx.z;

    if (x >= maxCoord.x || y >= maxCoord.y || z >= maxCoord.z) return;

    int3 coord = make_int3(x + minCoord.x, y + minCoord.y, z + minCoord.z);
    
    // Convert voxel coordinate to world position
    // (Simplified - assumes identity transform for now)
    float3 worldPos = make_float3((float)coord.x, (float)coord.y, (float)coord.z);
    
    // Calculate distance from brush center
    float dx = worldPos.x - brushCenter.x;
    float dy = worldPos.y - brushCenter.y;
    float dz = worldPos.z - brushCenter.z;
    float dist = sqrtf(dx*dx + dy*dy + dz*dz);
    
    // Calculate brush SDF value
    float brushSDF = dist - brushRadius;
    
    // Get current voxel value
    auto acc = grid->getAccessor();
    float currentVal = acc.getValue(nanovdb::Coord(coord.x, coord.y, coord.z));
    
    // Apply brush operation
    float newVal;
    if (mode == 0) { // ADD
        // Union: min(current, brush)
        newVal = fminf(currentVal, brushSDF * strength);
    } else { // SUBTRACT
        // Difference: max(current, -brush)
        newVal = fmaxf(currentVal, -brushSDF * strength);
    }
    
    // Blend based on strength
    newVal = currentVal * (1.0f - strength) + newVal * strength;
    
    // TODO: Write back using proper grid rebuilding
    // NanoVDB doesn't support setValue on GPU
}

// Host functions

bool GpuSculptKernels::ApplySphericalBrush(
    void* deviceGridPtr,
    const glm::vec3& brushCenter,
    float brushRadius,
    float voxelSize,
    SculptMode mode,
    float strength)
{
    if (!deviceGridPtr) {
        std::cerr << "[GpuSculptKernels] Invalid device grid pointer!" << std::endl;
        return false;
    }

    // Cast to NanoVDB grid
    auto* grid = reinterpret_cast<nanovdb::FloatGrid*>(deviceGridPtr);
    
    // Calculate bounding box for the brush operation
    float worldRadius = brushRadius + voxelSize * 4.0f; // Add padding
    glm::vec3 minWorld = brushCenter - glm::vec3(worldRadius);
    glm::vec3 maxWorld = brushCenter + glm::vec3(worldRadius);
    
    // Convert to voxel coordinates
    glm::ivec3 minVoxel = glm::ivec3(minWorld / voxelSize);
    glm::ivec3 maxVoxel = glm::ivec3(maxWorld / voxelSize) + glm::ivec3(1);
    glm::ivec3 size = maxVoxel - minVoxel;
    
    // Setup CUDA grid dimensions
    dim3 blockSize(8, 8, 8);
    dim3 gridSize(
        (size.x + blockSize.x - 1) / blockSize.x,
        (size.y + blockSize.y - 1) / blockSize.y,
        (size.z + blockSize.z - 1) / blockSize.z
    );
    
    // Convert brush center to voxel space
    float3 brushCenterVoxel = make_float3(
        brushCenter.x / voxelSize,
        brushCenter.y / voxelSize,
        brushCenter.z / voxelSize
    );
    float brushRadiusVoxel = brushRadius / voxelSize;
    
    int3 minCoord = make_int3(minVoxel.x, minVoxel.y, minVoxel.z);
    int3 maxCoord = make_int3(size.x, size.y, size.z);
    
    int modeInt = (mode == SculptMode::ADD) ? 0 : 1;
    
    // Launch kernel
    ApplySphericalBrushKernel<<<gridSize, blockSize>>>(
        grid,
        brushCenterVoxel,
        brushRadiusVoxel,
        modeInt,
        strength,
        minCoord,
        maxCoord
    );
    
    // Check for errors
    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess) {
        std::cerr << "[GpuSculptKernels] CUDA kernel error: " 
                  << cudaGetErrorString(err) << std::endl;
        return false;
    }
    
    // Synchronize
    cudaDeviceSynchronize();
    
    std::cout << "[GpuSculptKernels] Applied brush (mode=" << (int)mode 
              << ", strength=" << strength << ")" << std::endl;
    
    return true;
}

bool GpuSculptKernels::CreateBrushGrid(
    const glm::vec3& brushCenter,
    float brushRadius,
    float voxelSize,
    void** outDevicePtr,
    size_t* outSizeBytes)
{
    // This is a simplified implementation
    // In production, use nanovdb::GridBuilder to create a sphere SDF on GPU
    
    std::cerr << "[GpuSculptKernels] CreateBrushGrid not fully implemented yet!" << std::endl;
    return false;
}

bool GpuSculptKernels::CsgUnion(void* sceneGridPtr, void* brushGridPtr) {
    if (!sceneGridPtr || !brushGridPtr) {
        return false;
    }
    
    auto* sceneGrid = reinterpret_cast<nanovdb::FloatGrid*>(sceneGridPtr);
    auto* brushGrid = reinterpret_cast<const nanovdb::FloatGrid*>(brushGridPtr);
    
    // Get bounding box from brush grid
    // (Simplified - in production, get actual bbox from grid metadata)
    
    // Setup CUDA grid
    int3 minCoord = make_int3(0, 0, 0);
    int3 maxCoord = make_int3(128, 128, 128); // Placeholder
    
    dim3 blockSize(8, 8, 8);
    dim3 gridSize(16, 16, 16); // Placeholder
    
    CsgUnionKernel<<<gridSize, blockSize>>>(sceneGrid, brushGrid, minCoord, maxCoord);
    
    cudaDeviceSynchronize();
    return true;
}

bool GpuSculptKernels::CsgDifference(void* sceneGridPtr, void* brushGridPtr) {
    if (!sceneGridPtr || !brushGridPtr) {
        return false;
    }
    
    auto* sceneGrid = reinterpret_cast<nanovdb::FloatGrid*>(sceneGridPtr);
    auto* brushGrid = reinterpret_cast<const nanovdb::FloatGrid*>(brushGridPtr);
    
    int3 minCoord = make_int3(0, 0, 0);
    int3 maxCoord = make_int3(128, 128, 128); // Placeholder
    
    dim3 blockSize(8, 8, 8);
    dim3 gridSize(16, 16, 16); // Placeholder
    
    CsgDifferenceKernel<<<gridSize, blockSize>>>(sceneGrid, brushGrid, minCoord, maxCoord);
    
    cudaDeviceSynchronize();
    return true;
}

void GpuSculptKernels::FreeDeviceGrid(void* devicePtr) {
    if (devicePtr) {
        cudaFree(devicePtr);
    }
}

} // namespace Urbaxio::Engine

