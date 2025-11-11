// GPU Voxel Converter - CPU-side OpenVDB <-> NanoVDB conversion
// This file handles CPU-side conversion only (OpenVDB -> NanoVDB host buffer)
// GPU upload is handled in .cu files

// CRITICAL: Define this BEFORE including NanoVDB headers to enable openToNanoVDB
#define NANOVDB_USE_OPENVDB

#include "engine/geometry/VoxelGrid.h"
#include <openvdb/openvdb.h>
#include <nanovdb/tools/CreateNanoGrid.h> // OpenVDB -> NanoVDB conversion
#include <nanovdb/tools/NanoToOpenVDB.h>  // NanoVDB -> OpenVDB conversion
#include <nanovdb/HostBuffer.h>           // Host buffer (CPU only)
#include <nanovdb/GridHandle.h>           // Grid handle
#include <cuda_runtime.h>                 // For cudaMalloc/cudaMemcpy
#include <iostream>

namespace Urbaxio::Engine {

// Forward declaration of internal handle storage
// NOTE: Store raw buffer data, not GridHandle (to avoid CUDA linker issues)
struct NanoGridHandle {
    void* hostData;      // Raw host buffer data
    size_t sizeBytes;    // Size of buffer
    void* deviceData;    // Device pointer (uploaded by CUDA code)
};

// Convert OpenVDB grid to NanoVDB and upload to GPU
// Returns opaque pointer to NanoGridHandle (caller must cast and manage lifetime)
void* ConvertAndUploadToGpu(const VoxelGrid* voxelGrid) {
    if (!voxelGrid || !voxelGrid->grid_) {
        std::cerr << "[GpuVoxelConverter] Invalid input grid!" << std::endl;
        return nullptr;
    }

    try {
        // Step 1: Convert OpenVDB FloatGrid to NanoVDB (host buffer only)
        // openToNanoVDB accepts a shared_ptr and creates a NanoVDB grid in host memory
        
        // Cast to GridBase::Ptr (required by openToNanoVDB API)
        openvdb::GridBase::Ptr baseGrid = voxelGrid->grid_;
        
        // Convert with default settings (creates HostBuffer)
        auto nanoHandle = nanovdb::tools::openToNanoVDB<nanovdb::HostBuffer>(baseGrid);
        
        if (!nanoHandle) {
            std::cerr << "[GpuVoxelConverter] Failed to convert OpenVDB to NanoVDB!" << std::endl;
            return nullptr;
        }

        size_t sizeBytes = nanoHandle.size();
        std::cout << "[GpuVoxelConverter] Converted OpenVDB grid (" 
                  << voxelGrid->getActiveVoxelCount() << " active voxels) to NanoVDB ("
                  << sizeBytes << " bytes)" << std::endl;

        // Step 2: Allocate device memory and upload raw data
        void* d_data = nullptr;
        cudaError_t err = cudaMalloc(&d_data, sizeBytes);
        if (err != cudaSuccess) {
            std::cerr << "[GpuVoxelConverter] cudaMalloc failed: " << cudaGetErrorString(err) << std::endl;
            return nullptr;
        }

        // Copy host data to device
        err = cudaMemcpy(d_data, nanoHandle.data(), sizeBytes, cudaMemcpyHostToDevice);
        if (err != cudaSuccess) {
            std::cerr << "[GpuVoxelConverter] cudaMemcpy failed: " << cudaGetErrorString(err) << std::endl;
            cudaFree(d_data);
            return nullptr;
        }

        std::cout << "[GpuVoxelConverter] Uploaded to GPU: " 
                  << (sizeBytes / 1024.0 / 1024.0) << " MB" << std::endl;

        // Step 3: Store handle info (raw pointers to avoid CUDA linker issues)
        auto* handlePtr = new NanoGridHandle{
            nullptr,      // hostData (not stored - nanoHandle will be destroyed)
            sizeBytes,
            d_data        // deviceData
        };

        return handlePtr;

    } catch (const std::exception& e) {
        std::cerr << "[GpuVoxelConverter] Exception during conversion: " << e.what() << std::endl;
        return nullptr;
    }
}

// Download NanoVDB grid from GPU and convert back to OpenVDB
bool DownloadAndConvertFromGpu(void* nanoHandlePtr, VoxelGrid* outVoxelGrid) {
    if (!nanoHandlePtr || !outVoxelGrid) {
        std::cerr << "[GpuVoxelConverter] Invalid input parameters!" << std::endl;
        return false;
    }

    try {
        auto* handlePtr = reinterpret_cast<NanoGridHandle*>(nanoHandlePtr);
        
        // Step 1: Download device data to host
        std::vector<char> hostBuffer(handlePtr->sizeBytes);
        cudaError_t err = cudaMemcpy(hostBuffer.data(), handlePtr->deviceData, 
                                     handlePtr->sizeBytes, cudaMemcpyDeviceToHost);
        if (err != cudaSuccess) {
            std::cerr << "[GpuVoxelConverter] cudaMemcpy D2H failed: " << cudaGetErrorString(err) << std::endl;
            return false;
        }

        // Step 2: Create GridHandle from host buffer
        // createFull wraps existing memory (doesn't copy again)
        nanovdb::HostBuffer buffer = nanovdb::HostBuffer::createFull(handlePtr->sizeBytes, hostBuffer.data());
        nanovdb::GridHandle<nanovdb::HostBuffer> hostHandle(std::move(buffer));
        
        // Step 3: Convert NanoVDB to OpenVDB
        openvdb::GridBase::Ptr baseGrid = nanovdb::tools::nanoToOpenVDB(hostHandle);
        
        if (!baseGrid) {
            std::cerr << "[GpuVoxelConverter] Failed to convert NanoVDB to OpenVDB!" << std::endl;
            return false;
        }

        // Step 4: Cast to FloatGrid
        auto floatGrid = openvdb::gridPtrCast<openvdb::FloatGrid>(baseGrid);
        if (!floatGrid) {
            std::cerr << "[GpuVoxelConverter] Grid is not a FloatGrid!" << std::endl;
            return false;
        }

        // Step 5: Update metadata
        floatGrid->setName("SDF_Grid_Downloaded");
        floatGrid->setGridClass(openvdb::GRID_LEVEL_SET);
        
        // Preserve original transform if needed
        if (outVoxelGrid->grid_ && outVoxelGrid->grid_->transformPtr()) {
            floatGrid->setTransform(outVoxelGrid->grid_->transformPtr());
        }

        std::cout << "[GpuVoxelConverter] Downloaded and converted: " 
                  << floatGrid->activeVoxelCount() << " active voxels" << std::endl;

        // Step 6: Replace the grid
        outVoxelGrid->grid_ = floatGrid;
        outVoxelGrid->backgroundValue_ = floatGrid->background();
        outVoxelGrid->updateDimensions();

        return true;

    } catch (const std::exception& e) {
        std::cerr << "[GpuVoxelConverter] Exception during download: " << e.what() << std::endl;
        return false;
    }
}

// Get device pointer from NanoGridHandle
void* GetDevicePointerFromHandle(void* nanoHandlePtr) {
    if (!nanoHandlePtr) return nullptr;
    
    auto* handlePtr = reinterpret_cast<NanoGridHandle*>(nanoHandlePtr);
    return handlePtr->deviceData;
}

// Get size in bytes from NanoGridHandle
size_t GetSizeFromHandle(void* nanoHandlePtr) {
    if (!nanoHandlePtr) return 0;
    
    auto* handlePtr = reinterpret_cast<NanoGridHandle*>(nanoHandlePtr);
    return handlePtr->sizeBytes;
}

// Free NanoGridHandle
void FreeNanoHandle(void* nanoHandlePtr) {
    if (nanoHandlePtr) {
        auto* handlePtr = reinterpret_cast<NanoGridHandle*>(nanoHandlePtr);
        if (handlePtr->deviceData) {
            cudaFree(handlePtr->deviceData);
        }
        delete handlePtr;
    }
}

// Apply modified dense region back to OpenVDB grid
bool ApplyModifiedRegionToVoxelGrid(
    VoxelGrid* voxelGrid,
    const std::vector<float>& denseBuffer,
    const glm::ivec3& minVoxel,
    const glm::ivec3& maxVoxel)
{
    if (!voxelGrid || !voxelGrid->grid_) {
        std::cerr << "[GpuVoxelConverter] Invalid voxel grid!" << std::endl;
        return false;
    }

    glm::ivec3 size = maxVoxel - minVoxel;
    size_t expectedSize = static_cast<size_t>(size.x) * size.y * size.z;
    
    if (denseBuffer.size() != expectedSize) {
        std::cerr << "[GpuVoxelConverter] Dense buffer size mismatch! Expected " 
                  << expectedSize << ", got " << denseBuffer.size() << std::endl;
        return false;
    }

    // Get accessor for writing to the OpenVDB grid
    auto accessor = voxelGrid->grid_->getAccessor();
    
    // Copy dense buffer values back to sparse OpenVDB grid
    for (int z = 0; z < size.z; ++z) {
        for (int y = 0; y < size.y; ++y) {
            for (int x = 0; x < size.x; ++x) {
                int worldX = minVoxel.x + x;
                int worldY = minVoxel.y + y;
                int worldZ = minVoxel.z + z;
                
                size_t bufferIdx = static_cast<size_t>(z) * size.x * size.y + y * size.x + x;
                float value = denseBuffer[bufferIdx];
                
                // Write value to OpenVDB grid
                accessor.setValue(openvdb::Coord(worldX, worldY, worldZ), value);
            }
        }
    }
    
    std::cout << "[GpuVoxelConverter] Applied modified region to OpenVDB grid: " 
              << size.x << "x" << size.y << "x" << size.z << " voxels" << std::endl;
    
    return true;
}

} // namespace Urbaxio::Engine

