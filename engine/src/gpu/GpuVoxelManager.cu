// GPU Voxel Manager - CUDA Implementation
// CRITICAL: Include header first, then CUDA-specific code

#include <cuda_runtime.h>
#include <iostream>
#include <unordered_map>
#include <memory>

// Include the header to get the full class definition
#include "engine/gpu/GpuVoxelManager.h"
#include "engine/gpu/GpuVoxelConverter.h" // CPU-side converter

// Forward declare VoxelGrid to avoid OpenVDB includes
namespace Urbaxio {
namespace Engine {
    struct VoxelGrid;
}
}

namespace Urbaxio::Engine {

// Impl structure definition
// Now stores opaque pointers from GpuVoxelConverter instead of direct NanoVDB handles
struct GpuVoxelManager::Impl {
    struct GpuGridHandle {
        void* nanoHandlePtr; // Opaque pointer to NanoGridHandle (managed by converter)
        size_t sizeBytes;
        uint64_t leafCount;  // Number of leaf nodes (stored on CPU)
    };
    
    std::unordered_map<uint64_t, GpuGridHandle> gridHandles;
    bool gpuAvailable = false;
};

// Static method implementation
bool GpuVoxelManager::IsGpuAvailable() {
    int deviceCount = 0;
    cudaError_t err = cudaGetDeviceCount(&deviceCount);
    return (err == cudaSuccess && deviceCount > 0);
}

// Constructor
GpuVoxelManager::GpuVoxelManager() : impl_(std::make_unique<Impl>()), nextHandleId_(1) {
    impl_->gpuAvailable = IsGpuAvailable();
    
    if (impl_->gpuAvailable) {
        cudaDeviceProp prop;
        cudaGetDeviceProperties(&prop, 0);
        std::cout << "[GpuVoxelManager] Initialized with GPU: " << prop.name 
                  << " (Compute " << prop.major << "." << prop.minor << ")" << std::endl;
    } else {
        std::cerr << "[GpuVoxelManager] Warning: No CUDA-capable GPU found!" << std::endl;
    }
}

// Destructor
GpuVoxelManager::~GpuVoxelManager() = default;

// Upload grid - uses CPU-side converter
uint64_t GpuVoxelManager::UploadGrid(const VoxelGrid* grid) {
    if (!impl_->gpuAvailable) {
        std::cerr << "[GpuVoxelManager] GPU not available!" << std::endl;
        return 0;
    }

    if (!grid) {
        std::cerr << "[GpuVoxelManager] Invalid grid!" << std::endl;
        return 0;
    }

    // Use CPU-side converter to convert OpenVDB -> NanoVDB -> GPU
    void* nanoHandlePtr = ConvertAndUploadToGpu(grid);
    if (!nanoHandlePtr) {
        std::cerr << "[GpuVoxelManager] Failed to convert and upload grid!" << std::endl;
        return 0;
    }

    // Get size and leaf count from handle
    size_t sizeBytes = GetSizeFromHandle(nanoHandlePtr);
    uint64_t leafCount = GetLeafCountFromHandle(nanoHandlePtr);

    // Store handle with unique ID
    uint64_t handleId = nextHandleId_++;
    impl_->gridHandles[handleId] = {nanoHandlePtr, sizeBytes, leafCount};

    std::cout << "[GpuVoxelManager] Uploaded grid with ID " << handleId 
              << " (" << (sizeBytes / 1024.0 / 1024.0) << " MB)" << std::endl;

    return handleId;
}

// Download grid - uses CPU-side converter
bool GpuVoxelManager::DownloadGrid(uint64_t handle, VoxelGrid* outGrid) {
    if (!impl_->gpuAvailable) return false;
    
    auto it = impl_->gridHandles.find(handle);
    if (it == impl_->gridHandles.end()) {
        std::cerr << "[GpuVoxelManager] Invalid handle: " << handle << std::endl;
        return false;
    }

    if (!outGrid) {
        std::cerr << "[GpuVoxelManager] Invalid output grid!" << std::endl;
        return false;
    }

    // Use CPU-side converter to download GPU -> NanoVDB -> OpenVDB
    bool success = DownloadAndConvertFromGpu(it->second.nanoHandlePtr, outGrid);
    
    if (success) {
        std::cout << "[GpuVoxelManager] Downloaded grid with ID " << handle << std::endl;
    } else {
        std::cerr << "[GpuVoxelManager] Failed to download grid with ID " << handle << std::endl;
    }

    return success;
}

// Release grid
void GpuVoxelManager::ReleaseGrid(uint64_t handle) {
    auto it = impl_->gridHandles.find(handle);
    if (it != impl_->gridHandles.end()) {
        // Free the NanoGridHandle managed by converter
        FreeNanoHandle(it->second.nanoHandlePtr);
        impl_->gridHandles.erase(it);
        std::cout << "[GpuVoxelManager] Released GPU grid handle " << handle << std::endl;
    }
}

// DEPRECATED: Use ReleaseGrid instead
void GpuVoxelManager::FreeGrid(uint64_t handle) {
    ReleaseGrid(handle);
}

// Get device pointer
void* GpuVoxelManager::GetDeviceGridPointer(uint64_t handle) {
    auto it = impl_->gridHandles.find(handle);
    if (it == impl_->gridHandles.end()) {
        return nullptr;
    }
    
    // Use converter to extract device pointer from opaque handle
    return GetDevicePointerFromHandle(it->second.nanoHandlePtr);
}

// Get memory usage
size_t GpuVoxelManager::GetGridMemoryUsage(uint64_t handle) const {
    auto it = impl_->gridHandles.find(handle);
    if (it != impl_->gridHandles.end()) {
        return it->second.sizeBytes;
    }
    return 0;
}

// Get leaf count
uint64_t GpuVoxelManager::GetLeafCount(uint64_t handle) const {
    auto it = impl_->gridHandles.find(handle);
    if (it != impl_->gridHandles.end()) {
        return it->second.leafCount;
    }
    return 0;
}

// Get memory stats
GpuVoxelManager::MemoryStats GpuVoxelManager::GetMemoryStats() const {
    MemoryStats stats = {};
    
    if (!impl_->gpuAvailable) {
        return stats;
    }
    
    // Calculate total memory used
    for (const auto& pair : impl_->gridHandles) {
        stats.totalAllocated += pair.second.sizeBytes;
    }
    stats.gridCount = impl_->gridHandles.size();
    
    // Get GPU memory info
    size_t free = 0, total = 0;
    cudaMemGetInfo(&free, &total);
    stats.totalGpuMemory = total;
    stats.freeGpuMemory = free;
    
    return stats;
}

// Clear all grids
void GpuVoxelManager::ClearAll() {
    impl_->gridHandles.clear();
    std::cout << "[GpuVoxelManager] Cleared all GPU grids" << std::endl;
}

} // namespace Urbaxio::Engine
