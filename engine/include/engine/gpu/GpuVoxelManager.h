#pragma once

#include <memory>
#include <cstdint>

namespace Urbaxio::Engine {

// Forward declaration
struct VoxelGrid;

/**
 * @brief GPU manager for NanoVDB grids
 * 
 * Manages the lifecycle of volumetric data on GPU:
 * - Converts OpenVDB grids to NanoVDB format
 * - Uploads data to GPU memory
 * - Provides handles for GPU kernels
 * - Downloads results back to CPU
 */
class GpuVoxelManager {
public:
    GpuVoxelManager();
    ~GpuVoxelManager();

    // Delete copy operations (GPU resources are not copyable)
    GpuVoxelManager(const GpuVoxelManager&) = delete;
    GpuVoxelManager& operator=(const GpuVoxelManager&) = delete;

    /**
     * @brief Upload an OpenVDB grid to GPU as NanoVDB
     * @param grid The OpenVDB grid to upload
     * @return Handle ID for the GPU grid (0 if failed)
     */
    uint64_t UploadGrid(const VoxelGrid* grid);

    /**
     * @brief Download a NanoVDB grid from GPU back to OpenVDB
     * @param handleId The GPU grid handle ID
     * @param outGrid Output OpenVDB grid (must be pre-allocated)
     * @return True if successful
     */
    bool DownloadGrid(uint64_t handleId, VoxelGrid* outGrid);

    /**
     * @brief Get device pointer for a grid (for use in CUDA kernels)
     * @param handleId The GPU grid handle ID
     * @return Device pointer to NanoVDB grid, or nullptr if not found
     */
    void* GetDeviceGridPointer(uint64_t handleId);

    /**
     * @brief Free a GPU grid
     * @param handleId The GPU grid handle ID
     */
    void FreeGrid(uint64_t handleId);
    
    /**
     * @brief Release a GPU grid (same as FreeGrid)
     * @param handleId The GPU grid handle ID
     */
    void ReleaseGrid(uint64_t handleId);
    
    /**
     * @brief Get memory usage of a specific grid
     * @param handleId The GPU grid handle ID
     * @return Size in bytes (0 if invalid)
     */
    size_t GetGridMemoryUsage(uint64_t handleId) const;
    
    /**
     * @brief Get leaf count of a specific grid (stored on CPU to avoid GPU access)
     * @param handleId The GPU grid handle ID
     * @return Number of leaf nodes (0 if invalid)
     */
    uint64_t GetLeafCount(uint64_t handleId) const;
    
    /**
     * @brief Clear all GPU grids
     */
    void ClearAll();

    /**
     * @brief Check if GPU is available
     * @return True if CUDA device is available
     */
    static bool IsGpuAvailable();

    /**
     * @brief Get GPU memory usage statistics
     */
    struct MemoryStats {
        size_t totalAllocated = 0;
        size_t gridCount = 0;
        size_t totalGpuMemory = 0;
        size_t freeGpuMemory = 0;
    };
    MemoryStats GetMemoryStats() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;

    uint64_t nextHandleId_;
};

} // namespace Urbaxio::Engine

