// GPU Voxel Manager - CUDA Implementation
// CRITICAL: Include header first, then CUDA-specific code

#include <cuda_runtime.h>
#include <iostream>
#include <unordered_map>
#include <memory>

// Include the header to get the full class definition
#include "engine/gpu/GpuVoxelManager.h"

// NanoVDB headers
#include <nanovdb/GridHandle.h>
#include <nanovdb/cuda/DeviceBuffer.h>

// Forward declare VoxelGrid to avoid OpenVDB includes
namespace Urbaxio {
namespace Engine {
    struct VoxelGrid;
}
}

namespace Urbaxio::Engine {

// Impl structure definition
struct GpuVoxelManager::Impl {
    struct GpuGridHandle {
        nanovdb::GridHandle<nanovdb::cuda::DeviceBuffer> handle;
        size_t sizeBytes;
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

// Upload grid (placeholder - needs CPU-side converter)
uint64_t GpuVoxelManager::UploadGrid(const VoxelGrid* grid) {
    if (!impl_->gpuAvailable) {
        std::cerr << "[GpuVoxelManager] GPU not available!" << std::endl;
        return 0;
    }

    if (!grid) {
        std::cerr << "[GpuVoxelManager] Invalid grid!" << std::endl;
        return 0;
    }

    // TODO: Implement OpenVDB -> NanoVDB conversion
    // This must be done in a separate CPU-side .cpp file to avoid including OpenVDB here
    std::cerr << "[GpuVoxelManager] UploadGrid not yet implemented (requires CPU-side converter)" << std::endl;
    return 0;
}

// Download grid (placeholder)
bool GpuVoxelManager::DownloadGrid(uint64_t handle, VoxelGrid* outGrid) {
    if (!impl_->gpuAvailable) return false;
    
    auto it = impl_->gridHandles.find(handle);
    if (it == impl_->gridHandles.end()) {
        std::cerr << "[GpuVoxelManager] Invalid handle: " << handle << std::endl;
        return false;
    }

    // TODO: Download NanoVDB grid from GPU and convert back to OpenVDB
    std::cerr << "[GpuVoxelManager] DownloadGrid not yet implemented" << std::endl;
    return false;
}

// Release grid
void GpuVoxelManager::ReleaseGrid(uint64_t handle) {
    auto it = impl_->gridHandles.find(handle);
    if (it != impl_->gridHandles.end()) {
        std::cout << "[GpuVoxelManager] Released GPU grid handle " << handle << std::endl;
        impl_->gridHandles.erase(it);
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
    
    return it->second.handle.deviceData();
}

// Get memory usage
size_t GpuVoxelManager::GetGridMemoryUsage(uint64_t handle) const {
    auto it = impl_->gridHandles.find(handle);
    if (it != impl_->gridHandles.end()) {
        return it->second.sizeBytes;
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
