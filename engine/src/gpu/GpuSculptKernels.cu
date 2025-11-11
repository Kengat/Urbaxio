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
#include <nanovdb/cuda/DeviceBuffer.h>      // Updated path
#include <nanovdb/tools/GridBuilder.h>      // Updated path
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

// CUDA kernel: Extract dense buffer from sparse NanoVDB grid
__global__ void ExtractDenseBufferKernel(
    const nanovdb::FloatGrid* inputGrid,
    float* outputBuffer,
    int3 minCoord,
    int3 bufferDim,
    float backgroundValue)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int z = blockIdx.z * blockDim.z + threadIdx.z;

    if (x >= bufferDim.x || y >= bufferDim.y || z >= bufferDim.z) return;

    // World coordinate (offset by minCoord)
    int3 worldCoord = make_int3(x + minCoord.x, y + minCoord.y, z + minCoord.z);
    
    // Get value from sparse NanoVDB grid
    auto acc = inputGrid->getAccessor();
    float value = acc.getValue(nanovdb::Coord(worldCoord.x, worldCoord.y, worldCoord.z));
    
    // Write to dense buffer
    int bufferIdx = z * bufferDim.x * bufferDim.y + y * bufferDim.x + x;
    outputBuffer[bufferIdx] = value;
}

// CUDA kernel: Apply spherical brush to dense buffer (IN-PLACE)
__global__ void ApplySphericalBrushToDenseKernel(
    float* denseBuffer,
    float3 brushCenter,
    float brushRadius,
    int mode, // 0 = ADD, 1 = SUBTRACT
    float strength,
    int3 minCoord,
    int3 bufferDim,
    float voxelSize)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int z = blockIdx.z * blockDim.z + threadIdx.z;

    if (x >= bufferDim.x || y >= bufferDim.y || z >= bufferDim.z) return;

    // World coordinate (in voxel space)
    int3 worldCoord = make_int3(x + minCoord.x, y + minCoord.y, z + minCoord.z);
    
    // Convert to world position (in world units)
    float3 worldPos = make_float3(
        worldCoord.x * voxelSize,
        worldCoord.y * voxelSize,
        worldCoord.z * voxelSize
    );
    
    // Calculate distance from brush center
    float dx = worldPos.x - brushCenter.x;
    float dy = worldPos.y - brushCenter.y;
    float dz = worldPos.z - brushCenter.z;
    float dist = sqrtf(dx*dx + dy*dy + dz*dz);
    
    // Calculate brush SDF value
    float brushSDF = dist - brushRadius;
    
    // Get current voxel value from dense buffer
    int bufferIdx = z * bufferDim.x * bufferDim.y + y * bufferDim.x + x;
    float currentVal = denseBuffer[bufferIdx];
    
    // Apply brush operation
    float newVal;
    if (mode == 0) { // ADD
        // Union: min(current, brush)
        newVal = fminf(currentVal, brushSDF);
    } else { // SUBTRACT
        // Difference: max(current, -brush)
        newVal = fmaxf(currentVal, -brushSDF);
    }
    
    // Blend based on strength
    newVal = currentVal * (1.0f - strength) + newVal * strength;
    
    // Write back to dense buffer (IN-PLACE modification)
    denseBuffer[bufferIdx] = newVal;
}

// Helper function: Get bounding box from NanoVDB grid metadata
void GetGridBBox(const nanovdb::FloatGrid* grid, int3* outMin, int3* outMax) {
    auto bbox = grid->indexBBox();
    *outMin = make_int3(bbox.min()[0], bbox.min()[1], bbox.min()[2]);
    *outMax = make_int3(bbox.max()[0], bbox.max()[1], bbox.max()[2]);
}

// Host functions

bool GpuSculptKernels::ApplySphericalBrush(
    void* deviceGridPtr,
    const glm::vec3& brushCenter,
    float brushRadius,
    float voxelSize,
    SculptMode mode,
    float strength,
    std::vector<float>* outModifiedBuffer,
    glm::ivec3* outMinVoxel,
    glm::ivec3* outMaxVoxel)
{
    if (!deviceGridPtr) {
        std::cerr << "[GpuSculptKernels] Invalid device grid pointer!" << std::endl;
        return false;
    }

    auto* grid = reinterpret_cast<const nanovdb::FloatGrid*>(deviceGridPtr);
    
    // Get grid background value
    float backgroundValue = 3.0f; // Default SDF background (outside)
    
    // Calculate bounding box for the brush operation (in voxel space)
    float worldRadiusVoxels = (brushRadius + voxelSize * 4.0f) / voxelSize;
    glm::vec3 brushCenterVoxels = brushCenter / voxelSize;
    
    glm::ivec3 minVoxel = glm::ivec3(
        brushCenterVoxels.x - worldRadiusVoxels,
        brushCenterVoxels.y - worldRadiusVoxels,
        brushCenterVoxels.z - worldRadiusVoxels
    );
    glm::ivec3 maxVoxel = glm::ivec3(
        brushCenterVoxels.x + worldRadiusVoxels,
        brushCenterVoxels.y + worldRadiusVoxels,
        brushCenterVoxels.z + worldRadiusVoxels
    ) + glm::ivec3(1);
    
    // Clamp to grid bounds (get actual grid bbox from metadata)
    int3 gridMin, gridMax;
    GetGridBBox(grid, &gridMin, &gridMax);
    
    minVoxel = glm::max(minVoxel, glm::ivec3(gridMin.x, gridMin.y, gridMin.z));
    maxVoxel = glm::min(maxVoxel, glm::ivec3(gridMax.x, gridMax.y, gridMax.z));
    
    glm::ivec3 size = maxVoxel - minVoxel;
    
    if (size.x <= 0 || size.y <= 0 || size.z <= 0) {
        std::cerr << "[GpuSculptKernels] Invalid bounding box!" << std::endl;
        return false;
    }
    
    // Allocate device buffer for dense region
    size_t bufferSize = static_cast<size_t>(size.x) * size.y * size.z * sizeof(float);
    float* d_denseBuffer = nullptr;
    cudaError_t err = cudaMalloc(&d_denseBuffer, bufferSize);
    if (err != cudaSuccess) {
        std::cerr << "[GpuSculptKernels] cudaMalloc failed: " << cudaGetErrorString(err) << std::endl;
        return false;
    }
    
    // Setup CUDA grid dimensions
    dim3 blockSize(8, 8, 8);
    dim3 gridSize(
        (size.x + blockSize.x - 1) / blockSize.x,
        (size.y + blockSize.y - 1) / blockSize.y,
        (size.z + blockSize.z - 1) / blockSize.z
    );
    
    int3 minCoord = make_int3(minVoxel.x, minVoxel.y, minVoxel.z);
    int3 bufferDim = make_int3(size.x, size.y, size.z);
    
    // STEP 1: Extract dense buffer from sparse NanoVDB grid
    ExtractDenseBufferKernel<<<gridSize, blockSize>>>(
        grid,
        d_denseBuffer,
        minCoord,
        bufferDim,
        backgroundValue
    );
    
    err = cudaGetLastError();
    if (err != cudaSuccess) {
        std::cerr << "[GpuSculptKernels] ExtractDenseBufferKernel launch failed: " << cudaGetErrorString(err) << std::endl;
        cudaFree(d_denseBuffer);
        return false;
    }
    
    cudaDeviceSynchronize();
    
    // STEP 2: Apply sculpting operation to dense buffer (GPU)
    float3 brushCenterFloat = make_float3(brushCenter.x, brushCenter.y, brushCenter.z);
    int modeInt = (mode == SculptMode::ADD) ? 0 : 1;
    
    ApplySphericalBrushToDenseKernel<<<gridSize, blockSize>>>(
        d_denseBuffer,
        brushCenterFloat,
        brushRadius,
        modeInt,
        strength,
        minCoord,
        bufferDim,
        voxelSize
    );
    
    err = cudaGetLastError();
    if (err != cudaSuccess) {
        std::cerr << "[GpuSculptKernels] ApplySphericalBrushToDenseKernel launch failed: " << cudaGetErrorString(err) << std::endl;
        cudaFree(d_denseBuffer);
        return false;
    }
    
    cudaDeviceSynchronize();
    
    // STEP 3: Download dense buffer back to host (if requested)
    if (outModifiedBuffer && outMinVoxel && outMaxVoxel) {
        // Resize output buffer
        size_t elementCount = static_cast<size_t>(size.x) * size.y * size.z;
        outModifiedBuffer->resize(elementCount);
        
        // Download from GPU to host
        err = cudaMemcpy(outModifiedBuffer->data(), d_denseBuffer, bufferSize, cudaMemcpyDeviceToHost);
        if (err != cudaSuccess) {
            std::cerr << "[GpuSculptKernels] cudaMemcpy D2H failed: " << cudaGetErrorString(err) << std::endl;
            cudaFree(d_denseBuffer);
            return false;
        }
        
        // Return bounding box coordinates
        *outMinVoxel = minVoxel;
        *outMaxVoxel = maxVoxel;
        
        std::cout << "[GpuSculptKernels] Downloaded modified region: " 
                  << size.x << "x" << size.y << "x" << size.z 
                  << " (" << (bufferSize / 1024.0f / 1024.0f) << " MB)" << std::endl;
    }
    
    std::cout << "[GpuSculptKernels] GPU sculpting complete! Modified region: " 
              << size.x << "x" << size.y << "x" << size.z 
              << " (" << (bufferSize / 1024.0f / 1024.0f) << " MB)" << std::endl;
    
    cudaFree(d_denseBuffer);
    
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

