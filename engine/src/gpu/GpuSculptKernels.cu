// GPU Sculpting Kernels - Optimized Direct NanoVDB Modification

// No CPU roundtrips - values modified directly on GPU within existing topology



#include <cuda_runtime.h>



#ifdef __CUDACC__
#pragma nv_diag_suppress 20054
#pragma nv_diag_suppress 20012
#endif



#define OPENVDB_USE_DELAYED_LOADING 1
#define HALF_ENABLE_F16C_INTRINSICS 0



#include "engine/gpu/GpuSculptKernels.h"
#include <nanovdb/NanoVDB.h>
#include <nanovdb/cuda/DeviceBuffer.h>
#include <thrust/device_vector.h>
#include <thrust/iterator/counting_iterator.h>
#include <thrust/for_each.h>
#include <thrust/system/cuda/execution_policy.h>
#include <iostream>



namespace Urbaxio::Engine {



// CUDA Kernel: Direct leaf-level modification using Thrust-compatible lambda
// This is the FASTEST approach - no intermediate buffers, direct value modification
__host__ __device__
inline float smoothMin(float a, float b, float k) {
    float h = fmaxf(k - fabsf(a - b), 0.0f) / k;
    return fminf(a, b) - h * h * k * 0.25f;
}



__host__ __device__
inline float smoothMax(float a, float b, float k) {
    return -smoothMin(-a, -b, k);
}

// Optimized kernel with early rejection: only processes leaves within brush radius
__global__ void SculptBrushOptimizedKernel(
    nanovdb::FloatGrid* grid,
    float3 brushCenter,
    float brushRadius,
    int mode,
    float strength,
    float smoothing,
    uint64_t leafCount)
{
    uint64_t leafIdx = blockIdx.x;
    if (leafIdx >= leafCount) return;
    
    auto* leaf = grid->tree().getFirstNode<0>() + leafIdx;
    
    // Early rejection: check if leaf AABB intersects brush sphere
    nanovdb::Coord leafOrigin = leaf->origin();
    float3 leafCenter = make_float3(
        leafOrigin[0] + 4.0f,  // Leaf is 8^3, center at +4
        leafOrigin[1] + 4.0f,
        leafOrigin[2] + 4.0f
    );
    
    float dx = leafCenter.x - brushCenter.x;
    float dy = leafCenter.y - brushCenter.y;
    float dz = leafCenter.z - brushCenter.z;
    float distToLeafCenter = sqrtf(dx*dx + dy*dy + dz*dz);
    
    // Leaf diagonal is ~13.86 voxels, if brush is too far, skip entire leaf
    if (distToLeafCenter > brushRadius + 7.0f) return;
    
    // Process all 512 voxels in this leaf
    for (int voxelIdx = threadIdx.x; voxelIdx < 512; voxelIdx += blockDim.x) {
        if (!leaf->isActive(voxelIdx)) continue;
        
        nanovdb::Coord ijk = leaf->offsetToGlobalCoord(voxelIdx);
        
        float vdx = float(ijk[0]) - brushCenter.x;
        float vdy = float(ijk[1]) - brushCenter.y;
        float vdz = float(ijk[2]) - brushCenter.z;
        float dist = sqrtf(vdx*vdx + vdy*vdy + vdz*vdz);
        
        if (dist > brushRadius + 2.0f) continue;
        
        float brushSDF = dist - brushRadius;
        float currentVal = leaf->getValue(voxelIdx);
        
        float newVal;
        if (mode == 0) {
            newVal = smoothMin(currentVal, brushSDF, smoothing);
        } else {
            newVal = smoothMax(currentVal, -brushSDF, smoothing);
        }
        
        float falloff = 1.0f - (dist / (brushRadius + 1.0f));
        falloff = fmaxf(0.0f, fminf(1.0f, falloff));
        falloff = falloff * falloff;
        
        float blended = currentVal + (newVal - currentVal) * strength * falloff;
        leaf->setValueOnly(voxelIdx, blended);
    }
}



// Direct NanoVDB Leaf Modification Functor (for Thrust)
struct DirectSculptFunctor {
    nanovdb::FloatGrid* grid;
    float3 brushCenter;  // Index space
    float brushRadius;   // Voxel units
    int mode;            // 0=ADD, 1=SUBTRACT
    float strength;
    float smoothing;     // Smooth blending factor
    
    __device__ void operator()(uint64_t n) const {
        // Get leaf node and voxel index
        auto *leaf = grid->tree().getFirstNode<0>() + (n >> 9); // n / 512
        const int voxelIdx = n & 511; // n % 512
        
        // Skip inactive voxels (topology preserved)
        if (!leaf->isActive(voxelIdx)) return;
        
        // Convert to global coordinate
        nanovdb::Coord ijk = leaf->offsetToGlobalCoord(voxelIdx);
        
        // Calculate distance from brush center (index space)
        float dx = float(ijk[0]) - brushCenter.x;
        float dy = float(ijk[1]) - brushCenter.y;
        float dz = float(ijk[2]) - brushCenter.z;
        float dist = sqrtf(dx*dx + dy*dy + dz*dz);
        
        // Early exit if outside brush influence
        if (dist > brushRadius + 2.0f) return;
        
        // Calculate brush SDF
        float brushSDF = dist - brushRadius;
        
        // Get current value
        float currentVal = leaf->getValue(voxelIdx);
        
        // Apply CSG operation with smooth blending
        float newVal;
        if (mode == 0) { // ADD (union)
            newVal = smoothMin(currentVal, brushSDF, smoothing);
        } else { // SUBTRACT (difference)
            newVal = smoothMax(currentVal, -brushSDF, smoothing);
        }
        
        // Distance-based falloff
        float falloff = 1.0f - (dist / (brushRadius + 1.0f));
        falloff = fmaxf(0.0f, fminf(1.0f, falloff));
        falloff = falloff * falloff; // Squared for smoother transition
        
        // Blend with strength
        float blended = currentVal + (newVal - currentVal) * strength * falloff;
        
        // Write back (value-only, topology unchanged)
        leaf->setValueOnly(voxelIdx, blended);
    }
};



// Host function: Apply spherical brush using direct modification (async)
bool GpuSculptKernels::ApplySphericalBrush(
    void* deviceGridPtr,
    uint64_t leafCount,
    const glm::vec3& brushCenter,
    float brushRadius,
    float voxelSize,
    SculptMode mode,
    float strength,
    const glm::ivec3& gridBBoxMin,
    const glm::ivec3& gridBBoxMax,
    std::vector<float>* outModifiedBuffer,
    glm::ivec3* outMinVoxel,
    glm::ivec3* outMaxVoxel,
    void* cudaStream)
{
    if (!deviceGridPtr || leafCount == 0) {
        std::cerr << "[GpuSculptKernels] Invalid parameters!" << std::endl;
        return false;
    }
    
    // Launch optimized kernel with one block per leaf, 256 threads per block
    dim3 blockSize(256);
    dim3 gridSize(static_cast<unsigned int>(leafCount));
    
    cudaStream_t stream = cudaStream ? static_cast<cudaStream_t>(cudaStream) : 0;
    
    SculptBrushOptimizedKernel<<<gridSize, blockSize, 0, stream>>>(
        reinterpret_cast<nanovdb::FloatGrid*>(deviceGridPtr),
        make_float3(brushCenter.x, brushCenter.y, brushCenter.z),
        brushRadius,
        (mode == SculptMode::ADD) ? 0 : 1,
        strength,
        0.5f, // Smooth blending
        leafCount
    );
    
    // Don't sync - let it run async (caller will sync when needed)
    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess) {
        std::cerr << "[GpuSculptKernels] ❌ Kernel launch error: " 
                  << cudaGetErrorString(err) << std::endl;
        return false;
    }
    
    return true;
}



// Legacy/unused functions (kept for API compatibility)
bool GpuSculptKernels::CreateBrushGrid(
    const glm::vec3& brushCenter,
    float brushRadius,
    float voxelSize,
    void** outDevicePtr,
    size_t* outSizeBytes)
{
    std::cerr << "[GpuSculptKernels] CreateBrushGrid not used in direct modification!" << std::endl;
    return false;
}



bool GpuSculptKernels::CsgUnion(void* sceneGridPtr, void* brushGridPtr) {
    std::cerr << "[GpuSculptKernels] CSG not used in direct modification!" << std::endl;
    return false;
}



bool GpuSculptKernels::CsgDifference(void* sceneGridPtr, void* brushGridPtr) {
    std::cerr << "[GpuSculptKernels] CSG not used in direct modification!" << std::endl;
    return false;
}



void GpuSculptKernels::FreeDeviceGrid(void* devicePtr) {
    // Memory managed by GridHandle
}



} // namespace Urbaxio::Engine

