#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>
#include <glm/glm.hpp>

// Forward declarations
namespace Urbaxio::Engine {
    struct VoxelGrid;
}

namespace Urbaxio::Engine {

// CPU-side converter functions for OpenVDB <-> NanoVDB
// These functions handle the conversion and GPU upload/download
// All functions return opaque pointers to hide NanoVDB/OpenVDB details from CUDA code

/**
 * @brief Convert OpenVDB grid to NanoVDB and upload to GPU
 * @param voxelGrid Input OpenVDB grid
 * @return Opaque pointer to NanoGridHandle (nullptr if failed)
 */
void* ConvertAndUploadToGpu(const VoxelGrid* voxelGrid);

/**
 * @brief Download NanoVDB grid from GPU and convert back to OpenVDB
 * @param nanoHandlePtr Opaque pointer to NanoGridHandle
 * @param outVoxelGrid Output OpenVDB grid (modified in-place)
 * @return True if successful
 */
bool DownloadAndConvertFromGpu(void* nanoHandlePtr, VoxelGrid* outVoxelGrid);

/**
 * @brief Get device pointer for CUDA kernels from NanoGridHandle
 * @param nanoHandlePtr Opaque pointer to NanoGridHandle
 * @return Device pointer to NanoVDB grid data
 */
void* GetDevicePointerFromHandle(void* nanoHandlePtr);

/**
 * @brief Get size in bytes from NanoGridHandle
 * @param nanoHandlePtr Opaque pointer to NanoGridHandle
 * @return Size in bytes
 */
size_t GetSizeFromHandle(void* nanoHandlePtr);

/**
 * @brief Free NanoGridHandle and release GPU memory
 * @param nanoHandlePtr Opaque pointer to NanoGridHandle
 */
void FreeNanoHandle(void* nanoHandlePtr);

/**
 * @brief Apply a modified dense region back to an OpenVDB grid
 * 
 * This is used after GPU sculpting to update the sparse OpenVDB grid
 * with modifications performed on a dense buffer.
 * 
 * @param voxelGrid Target OpenVDB grid to modify
 * @param denseBuffer Dense buffer containing modified voxel values
 * @param minVoxel Minimum coordinate of the modified region
 * @param maxVoxel Maximum coordinate of the modified region
 * @return True if successful
 */
bool ApplyModifiedRegionToVoxelGrid(
    VoxelGrid* voxelGrid,
    const std::vector<float>& denseBuffer,
    const glm::ivec3& minVoxel,
    const glm::ivec3& maxVoxel
);

} // namespace Urbaxio::Engine

