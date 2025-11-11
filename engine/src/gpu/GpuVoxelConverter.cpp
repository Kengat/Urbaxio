// GPU Voxel Converter - Optimized OpenVDB <-> NanoVDB conversion
// Minimal copies, persistent GPU memory for sculpting sessions



#define NANOVDB_USE_OPENVDB



#include "engine/geometry/VoxelGrid.h"
#include <openvdb/openvdb.h>
#include <nanovdb/tools/CreateNanoGrid.h>
#include <nanovdb/tools/NanoToOpenVDB.h>
#include <nanovdb/HostBuffer.h>
#include <nanovdb/GridHandle.h>
#include <cuda_runtime.h>
#include <iostream>



namespace Urbaxio::Engine {



// Internal handle storage
struct NanoGridHandle {
    void* hostData;      // Host buffer (for persistence)
    size_t sizeBytes;
    void* deviceData;    // Device pointer
    bool isSequential;   // Required for direct leaf access
    uint64_t leafCount;  // Number of leaf nodes (stored on CPU to avoid GPU access)
};



// Convert OpenVDB to NanoVDB and upload to GPU (optimized path)
void* ConvertAndUploadToGpu(const VoxelGrid* voxelGrid) {
    if (!voxelGrid || !voxelGrid->grid_) {
        std::cerr << "[GpuVoxelConverter] ❌ Invalid input grid!" << std::endl;
        return nullptr;
    }



    try {
        // Convert OpenVDB -> NanoVDB with sequential leaf ordering
        // This is CRITICAL for direct leaf access in sculpting
        openvdb::GridBase::Ptr baseGrid = voxelGrid->grid_;
        
        std::cout << "[GpuVoxelConverter] Converting OpenVDB (" 
                  << voxelGrid->getActiveVoxelCount() << " active voxels)..." << std::endl;
        
        // Create NanoVDB with HostBuffer
        auto nanoHandle = nanovdb::tools::openToNanoVDB<nanovdb::HostBuffer>(baseGrid);
        
        if (!nanoHandle) {
            std::cerr << "[GpuVoxelConverter] ❌ Conversion failed!" << std::endl;
            return nullptr;
        }



        size_t sizeBytes = nanoHandle.size();
        
        // Get grid and verify it's sequential (ON CPU - before upload to GPU)
        auto* nanoGrid = nanoHandle.grid<float>();
        bool isSeq = nanoGrid->isSequential<0>();
        uint64_t leafCount = nanoGrid->tree().nodeCount(0); // Get leaf count on CPU
        
        std::cout << "[GpuVoxelConverter] ✅ NanoVDB created: " 
                  << (sizeBytes / 1024.0 / 1024.0) << " MB, "
                  << (isSeq ? "SEQUENTIAL" : "NOT SEQUENTIAL") << ", "
                  << leafCount << " leaves" << std::endl;
        
        if (!isSeq) {
            std::cerr << "[GpuVoxelConverter] ⚠️ WARNING: Grid not sequential! "
                      << "Direct modification may not work correctly." << std::endl;
        }



        // Allocate GPU memory
        void* d_data = nullptr;
        cudaError_t err = cudaMalloc(&d_data, sizeBytes);
        if (err != cudaSuccess) {
            std::cerr << "[GpuVoxelConverter] ❌ cudaMalloc failed: " 
                      << cudaGetErrorString(err) << std::endl;
            return nullptr;
        }



        // Upload to GPU
        err = cudaMemcpy(d_data, nanoHandle.data(), sizeBytes, cudaMemcpyHostToDevice);
        if (err != cudaSuccess) {
            std::cerr << "[GpuVoxelConverter] ❌ cudaMemcpy failed: " 
                      << cudaGetErrorString(err) << std::endl;
            cudaFree(d_data);
            return nullptr;
        }



        std::cout << "[GpuVoxelConverter] ✅ Uploaded to GPU: " 
                  << (sizeBytes / 1024.0 / 1024.0) << " MB" << std::endl;



        // Store handle (with metadata captured on CPU)
        auto* handlePtr = new NanoGridHandle{
            nullptr,      // Don't keep host copy (save memory)
            sizeBytes,
            d_data,
            isSeq,
            leafCount      // Store leaf count (captured on CPU before upload)
        };



        return handlePtr;



    } catch (const std::exception& e) {
        std::cerr << "[GpuVoxelConverter] ❌ Exception: " << e.what() << std::endl;
        return nullptr;
    }
}



// Download and convert back to OpenVDB (for checkpoints/undo)
bool DownloadAndConvertFromGpu(void* nanoHandlePtr, VoxelGrid* outVoxelGrid) {
    if (!nanoHandlePtr || !outVoxelGrid) {
        std::cerr << "[GpuVoxelConverter] ❌ Invalid parameters!" << std::endl;
        return false;
    }



    try {
        auto* handlePtr = reinterpret_cast<NanoGridHandle*>(nanoHandlePtr);
        
        std::cout << "[GpuVoxelConverter] Downloading from GPU: " 
                  << (handlePtr->sizeBytes / 1024.0 / 1024.0) << " MB..." << std::endl;
        
        // Download from GPU
        std::vector<char> hostBuffer(handlePtr->sizeBytes);
        cudaError_t err = cudaMemcpy(
            hostBuffer.data(), 
            handlePtr->deviceData, 
            handlePtr->sizeBytes, 
            cudaMemcpyDeviceToHost
        );
        
        if (err != cudaSuccess) {
            std::cerr << "[GpuVoxelConverter] ❌ cudaMemcpy D2H failed: " 
                      << cudaGetErrorString(err) << std::endl;
            return false;
        }



        // Create GridHandle from downloaded data
        nanovdb::HostBuffer buffer = nanovdb::HostBuffer::createFull(
            handlePtr->sizeBytes, 
            hostBuffer.data()
        );
        nanovdb::GridHandle<nanovdb::HostBuffer> hostHandle(std::move(buffer));
        
        // Convert NanoVDB -> OpenVDB
        openvdb::GridBase::Ptr baseGrid = nanovdb::tools::nanoToOpenVDB(hostHandle);
        
        if (!baseGrid) {
            std::cerr << "[GpuVoxelConverter] ❌ NanoVDB->OpenVDB conversion failed!" << std::endl;
            return false;
        }



        // Cast to FloatGrid
        auto floatGrid = openvdb::gridPtrCast<openvdb::FloatGrid>(baseGrid);
        if (!floatGrid) {
            std::cerr << "[GpuVoxelConverter] ❌ Not a FloatGrid!" << std::endl;
            return false;
        }



        // Update metadata
        floatGrid->setName("SDF_Grid");
        floatGrid->setGridClass(openvdb::GRID_LEVEL_SET);
        
        // Preserve transform if possible
        if (outVoxelGrid->grid_ && outVoxelGrid->grid_->transformPtr()) {
            floatGrid->setTransform(outVoxelGrid->grid_->transformPtr());
        }



        std::cout << "[GpuVoxelConverter] ✅ Downloaded: " 
                  << floatGrid->activeVoxelCount() << " active voxels" << std::endl;



        // Replace grid
        outVoxelGrid->grid_ = floatGrid;
        outVoxelGrid->backgroundValue_ = floatGrid->background();
        outVoxelGrid->updateDimensions();



        return true;



    } catch (const std::exception& e) {
        std::cerr << "[GpuVoxelConverter] ❌ Exception: " << e.what() << std::endl;
        return false;
    }
}



// Get device pointer from handle
void* GetDevicePointerFromHandle(void* nanoHandlePtr) {
    if (!nanoHandlePtr) return nullptr;
    auto* handlePtr = reinterpret_cast<NanoGridHandle*>(nanoHandlePtr);
    return handlePtr->deviceData;
}



// Get size from handle
size_t GetSizeFromHandle(void* nanoHandlePtr) {
    if (!nanoHandlePtr) return 0;
    auto* handlePtr = reinterpret_cast<NanoGridHandle*>(nanoHandlePtr);
    return handlePtr->sizeBytes;
}



// Get leaf count from handle (stored on CPU to avoid GPU access)
uint64_t GetLeafCountFromHandle(void* nanoHandlePtr) {
    if (!nanoHandlePtr) return 0;
    auto* handlePtr = reinterpret_cast<NanoGridHandle*>(nanoHandlePtr);
    return handlePtr->leafCount;
}



// Free handle
void FreeNanoHandle(void* nanoHandlePtr) {
    if (nanoHandlePtr) {
        auto* handlePtr = reinterpret_cast<NanoGridHandle*>(nanoHandlePtr);
        if (handlePtr->deviceData) {
            cudaFree(handlePtr->deviceData);
        }
        delete handlePtr;
        std::cout << "[GpuVoxelConverter] ✅ Freed GPU memory" << std::endl;
    }
}



// NOT USED in optimized path - grid modified in-place on GPU
bool ApplyModifiedRegionToVoxelGrid(
    VoxelGrid* voxelGrid,
    const std::vector<float>& denseBuffer,
    const glm::ivec3& minVoxel,
    const glm::ivec3& maxVoxel)
{
    std::cerr << "[GpuVoxelConverter] ApplyModifiedRegionToVoxelGrid NOT USED in optimized path!" << std::endl;
    return false;
}



} // namespace Urbaxio::Engine

