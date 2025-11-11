#pragma once



#include <cstdint>
#include <vector>
#include <glm/glm.hpp>



namespace Urbaxio::Engine {



// Forward declaration
struct VoxelGrid;



// Convert OpenVDB VoxelGrid to NanoVDB and upload to GPU
// Returns opaque handle pointer (managed by caller)
void* ConvertAndUploadToGpu(const VoxelGrid* voxelGrid);



// Download NanoVDB from GPU and convert back to OpenVDB
bool DownloadAndConvertFromGpu(void* nanoHandlePtr, VoxelGrid* outVoxelGrid);



// Get device pointer from handle (for kernel access)
void* GetDevicePointerFromHandle(void* nanoHandlePtr);



// Get size of grid on GPU
size_t GetSizeFromHandle(void* nanoHandlePtr);



// Get leaf count from handle (stored on CPU to avoid GPU access)
uint64_t GetLeafCountFromHandle(void* nanoHandlePtr);



// Free NanoVDB handle and GPU memory
void FreeNanoHandle(void* nanoHandlePtr);



// DEPRECATED - not used in optimized path
bool ApplyModifiedRegionToVoxelGrid(
    VoxelGrid* voxelGrid,
    const std::vector<float>& denseBuffer,
    const glm::ivec3& minVoxel,
    const glm::ivec3& maxVoxel
);



} // namespace Urbaxio::Engine

