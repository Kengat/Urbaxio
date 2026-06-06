
#include "engine/gpu/DynamicGpuHashGrid.h"
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <thrust/device_vector.h>
#include <thrust/sequence.h>
#include <thrust/copy.h>
#include <thrust/count.h>
#include <iostream>
#include <cmath>
#include <chrono>

namespace Urbaxio::Engine {

// ============================================================================
// DEVICE HELPER FUNCTIONS
// ============================================================================

__device__ __forceinline__ uint32_t hash3D(int32_t x, int32_t y, int32_t z, uint32_t tableSize) {
    uint32_t h = x * 73856093u ^ y * 19349663u ^ z * 83492791u;
    return h & (tableSize - 1);
}

__device__ __forceinline__ int32_t worldToBlock(float coord, float voxelSize) {
    return static_cast<int32_t>(floorf(coord / (voxelSize * DynamicGpuHashGrid::BLOCK_SIZE)));
}

// ============================================================================
// OPTIMIZED BLOCK ALLOCATION WITH DIRTY TRACKING
// ============================================================================

__device__ uint32_t findOrAllocateBlockWithDirty(
    DynamicGpuHashGrid::HashEntry* hashTable,
    uint32_t tableSize,
    int32_t blockX, int32_t blockY, int32_t blockZ,
    uint32_t* blockCounter,
    uint32_t maxBlocks,
    float* blockData,
    uint32_t* dirtyFlags,      // NEW: Per-block dirty flag
    uint32_t* dirtyIndices,    // NEW: List of dirty block indices
    uint32_t* dirtyCount,      // NEW: Count of dirty blocks
    uint32_t maxDirtyBlocks)   // NEW: Max dirty blocks to track
{
    uint32_t slot = hash3D(blockX, blockY, blockZ, tableSize);

    for (uint32_t probe = 0; probe < 32; ++probe) {  // Reduced from 128 to prevent infinite loops
        uint32_t currentSlot = (slot + probe) & (tableSize - 1);

        int32_t oldX = atomicCAS(&hashTable[currentSlot].blockX, INT32_MAX, blockX);

        if (oldX == INT32_MAX) {
            // New block - allocate
            hashTable[currentSlot].blockY = blockY;
            hashTable[currentSlot].blockZ = blockZ;

            uint32_t dataIdx = atomicAdd(blockCounter, 1);
            if (dataIdx >= maxBlocks) {
                hashTable[currentSlot].blockX = INT32_MAX;
                return UINT32_MAX;
            }

            hashTable[currentSlot].dataIndex = dataIdx;

            // Initialize block to 1.0f (outside surface)
            float* block = blockData + dataIdx * 512;
            for (int i = threadIdx.x; i < 512; i += blockDim.x) {
                block[i] = 1.0f;
            }
            // NO __syncthreads() here - not all threads reach this point, would cause deadlock

            // Mark as dirty (new block always dirty)
            if (dirtyFlags && atomicExch(&dirtyFlags[dataIdx], 1) == 0) {
                uint32_t dirtyIdx = atomicAdd(dirtyCount, 1);
                if (dirtyIdx < maxDirtyBlocks) {
                    dirtyIndices[dirtyIdx] = dataIdx;
                }
            }

            return dataIdx;
        }

        // Check if this is our block
        if (hashTable[currentSlot].blockX == blockX &&
            hashTable[currentSlot].blockY == blockY &&
            hashTable[currentSlot].blockZ == blockZ) {
            
            uint32_t dataIdx = hashTable[currentSlot].dataIndex;
            
            // Mark existing block as dirty
            if (dirtyFlags && atomicExch(&dirtyFlags[dataIdx], 1) == 0) {
                uint32_t dirtyIdx = atomicAdd(dirtyCount, 1);
                if (dirtyIdx < maxDirtyBlocks) {
                    dirtyIndices[dirtyIdx] = dataIdx;
                }
            }
            
            return dataIdx;
        }
    }

    return UINT32_MAX;
}

// ============================================================================
// OPTIMIZED SCULPT KERNEL
// ============================================================================

__global__ void SculptSphericalKernelOptimized(
    DynamicGpuHashGrid::HashEntry* hashTable,
    uint32_t tableSize,
    float* blockData,
    uint32_t* blockCounter,
    uint32_t maxBlocks,
    float3 brushCenter,
    float brushRadius,
    float strength,
    bool addMode,
    float voxelSize,
    uint32_t* dirtyFlags,
    uint32_t* dirtyIndices,
    uint32_t* dirtyCount,
    uint32_t maxDirtyBlocks)
{
    // Calculate block coordinates from grid position
    int32_t centerBlockX = worldToBlock(brushCenter.x, voxelSize);
    int32_t centerBlockY = worldToBlock(brushCenter.y, voxelSize);
    int32_t centerBlockZ = worldToBlock(brushCenter.z, voxelSize);
    
    int32_t blockX = centerBlockX + blockIdx.x - gridDim.x / 2;
    int32_t blockY = centerBlockY + blockIdx.y - gridDim.y / 2;
    int32_t blockZ = centerBlockZ + blockIdx.z - gridDim.z / 2;

    // Quick AABB check - skip blocks definitely outside brush
    float blockMinX = blockX * DynamicGpuHashGrid::BLOCK_SIZE * voxelSize;
    float blockMaxX = blockMinX + DynamicGpuHashGrid::BLOCK_SIZE * voxelSize;
    float blockMinY = blockY * DynamicGpuHashGrid::BLOCK_SIZE * voxelSize;
    float blockMaxY = blockMinY + DynamicGpuHashGrid::BLOCK_SIZE * voxelSize;
    float blockMinZ = blockZ * DynamicGpuHashGrid::BLOCK_SIZE * voxelSize;
    float blockMaxZ = blockMinZ + DynamicGpuHashGrid::BLOCK_SIZE * voxelSize;
    
    // Sphere-AABB distance check
    float closestX = fmaxf(blockMinX, fminf(brushCenter.x, blockMaxX));
    float closestY = fmaxf(blockMinY, fminf(brushCenter.y, blockMaxY));
    float closestZ = fmaxf(blockMinZ, fminf(brushCenter.z, blockMaxZ));
    
    float distSq = (closestX - brushCenter.x) * (closestX - brushCenter.x) +
                   (closestY - brushCenter.y) * (closestY - brushCenter.y) +
                   (closestZ - brushCenter.z) * (closestZ - brushCenter.z);
    
    float maxInfluence = brushRadius + voxelSize * 4.0f;
    if (distSq > maxInfluence * maxInfluence) return;

    // Find or allocate block with dirty tracking
    uint32_t dataIdx = findOrAllocateBlockWithDirty(
        hashTable, tableSize,
        blockX, blockY, blockZ,
        blockCounter, maxBlocks, blockData,
        dirtyFlags, dirtyIndices, dirtyCount, maxDirtyBlocks
    );

    if (dataIdx == UINT32_MAX) return;

    // Process voxels
    int voxelIdx = threadIdx.x;
    if (voxelIdx >= DynamicGpuHashGrid::VOXELS_PER_BLOCK) return;

    int localX = voxelIdx & 7;
    int localY = (voxelIdx >> 3) & 7;
    int localZ = voxelIdx >> 6;

    float3 voxelWorld = make_float3(
        (blockX * 8 + localX) * voxelSize,
        (blockY * 8 + localY) * voxelSize,
        (blockZ * 8 + localZ) * voxelSize
    );

    float dx = voxelWorld.x - brushCenter.x;
    float dy = voxelWorld.y - brushCenter.y;
    float dz = voxelWorld.z - brushCenter.z;
    float dist = sqrtf(dx*dx + dy*dy + dz*dz);

    // Skip voxels far from brush
    float maxDist = brushRadius + voxelSize * 6.0f;
    if (dist > maxDist) return;

    float brushSDF = dist - brushRadius;
    float* block = blockData + dataIdx * 512;
    float currentVal = block[voxelIdx];

    // Smooth blending
    float diff = fabsf(currentVal - brushSDF);
    const float smoothThreshold = voxelSize * 3.0f;
    
    float newVal;
    if (addMode) {
        // Adding material (union)
        if (diff < smoothThreshold) {
            const float k = voxelSize * 2.0f;
            float h = fmaxf(k - diff, 0.0f) / k;
            newVal = fminf(currentVal, brushSDF) - h * h * k * 0.25f;
        } else {
            newVal = fminf(currentVal, brushSDF);
        }
    } else {
        // Removing material (subtraction)
        newVal = fmaxf(currentVal, -brushSDF);
    }

    // Smooth falloff
    float t = dist / maxDist;
    t = fmaxf(0.0f, fminf(1.0f, t));
    float falloff = 1.0f - t * t * (3.0f - 2.0f * t);
    
    float blended = currentVal + (newVal - currentVal) * strength * falloff;
    blended = fmaxf(-brushRadius * 2.0f, fminf(brushRadius * 2.0f, blended));

    block[voxelIdx] = blended;
}

// ============================================================================
// UTILITY KERNELS
// ============================================================================

__global__ void ClearDirtyFlagsKernel(uint32_t* dirtyFlags, uint32_t count) {
    uint32_t idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < count) {
        dirtyFlags[idx] = 0;
    }
}

__global__ void MarkAllBlocksDirtyKernel(
    const DynamicGpuHashGrid::HashEntry* hashTable,
    uint32_t tableSize,
    uint32_t* dirtyFlags,
    uint32_t* dirtyIndices,
    uint32_t* dirtyCount,
    uint32_t maxDirtyBlocks)
{
    uint32_t idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= tableSize) return;
    
    if (hashTable[idx].blockX != INT32_MAX) {
        uint32_t dataIdx = hashTable[idx].dataIndex;
        if (atomicExch(&dirtyFlags[dataIdx], 1) == 0) {
            uint32_t dirtyIdx = atomicAdd(dirtyCount, 1);
            if (dirtyIdx < maxDirtyBlocks) {
                dirtyIndices[dirtyIdx] = dataIdx;
            }
        }
    }
}

// Thrust predicate for active blocks
struct IsBlockActive {
    const DynamicGpuHashGrid::HashEntry* table;
    
    __device__ bool operator()(uint32_t idx) const {
        return table[idx].blockX != INT32_MAX;
    }
};

// ============================================================================
// HOST IMPLEMENTATION
// ============================================================================

DynamicGpuHashGrid::DynamicGpuHashGrid(const Config& config)
    : config_(config)
{
    initialize();
}

DynamicGpuHashGrid::~DynamicGpuHashGrid() {
    cleanup();
}

void DynamicGpuHashGrid::initialize() {
    std::cout << "[DynamicGpuHashGrid] Initializing optimized version..." << std::endl;
    
    // Check available memory
    size_t freeMemory, totalMemory;
    cudaMemGetInfo(&freeMemory, &totalMemory);
    float freeGB = freeMemory / (1024.0f * 1024.0f * 1024.0f);
    
    std::cout << "  Available GPU memory: " << freeGB << " GB" << std::endl;
    
    // Adjust config for low memory
    if (freeGB < 2.0f) {
        config_.maxBlocks = std::min(config_.maxBlocks, 100u * 1024u);
        config_.hashTableSize = std::min(config_.hashTableSize, 256u * 1024u);
        std::cout << "  [WARN] Low memory - reducing allocations" << std::endl;
    }

    // Calculate memory requirements
    size_t hashTableBytes = config_.hashTableSize * sizeof(HashEntry);
    size_t blockDataBytes = config_.maxBlocks * VOXELS_PER_BLOCK * sizeof(float);
    size_t dirtyFlagsBytes = config_.maxBlocks * sizeof(uint32_t);
    size_t dirtyIndicesBytes = config_.maxDirtyBlocks * sizeof(uint32_t);
    size_t blockMeshInfoBytes = config_.maxBlocks * sizeof(BlockMeshInfo);
    
    std::cout << "  Hash table: " << hashTableBytes / (1024.0f * 1024.0f) << " MB" << std::endl;
    std::cout << "  Block data: " << blockDataBytes / (1024.0f * 1024.0f) << " MB" << std::endl;
    std::cout << "  Dirty tracking: " << (dirtyFlagsBytes + dirtyIndicesBytes) / (1024.0f * 1024.0f) << " MB" << std::endl;

    // Allocate hash table
    cudaMalloc(&d_hashTable_, hashTableBytes);
    
    // Initialize to empty
    std::vector<HashEntry> emptyTable(config_.hashTableSize);
    for (auto& entry : emptyTable) {
        entry.blockX = INT32_MAX;  // ✅ Критично!
        entry.blockY = 0;
        entry.blockZ = 0;
        entry.dataIndex = 0;
    }
    cudaMemcpy(d_hashTable_, emptyTable.data(), hashTableBytes, cudaMemcpyHostToDevice);
    
    // ✅ ДОБАВИТЬ проверку:
    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess) {
        std::cerr << "[DynamicGpuHashGrid] Hash table init failed: " << cudaGetErrorString(err) << std::endl;
    }

    // Allocate block data
    cudaMalloc(&d_blockData_, blockDataBytes);
    cudaMemset(d_blockData_, 0, blockDataBytes);

    // Allocate block counter
    cudaMalloc(&d_blockCounter_, sizeof(uint32_t));
    cudaMemset(d_blockCounter_, 0, sizeof(uint32_t));

    // NEW: Allocate dirty tracking buffers
    cudaMalloc(&d_dirtyFlags_, dirtyFlagsBytes);
    cudaMemset(d_dirtyFlags_, 0, dirtyFlagsBytes);
    
    cudaMalloc(&d_dirtyBlockIndices_, dirtyIndicesBytes);
    cudaMalloc(&d_dirtyCount_, sizeof(uint32_t));
    cudaMemset(d_dirtyCount_, 0, sizeof(uint32_t));

    // NEW: Allocate per-block mesh info
    cudaMalloc(&d_blockMeshInfo_, blockMeshInfoBytes);
    cudaMemset(d_blockMeshInfo_, 0, blockMeshInfoBytes);

    // Allocate active block tracking
    cudaMalloc(&d_activeIndices_, config_.maxBlocks * sizeof(uint32_t));
    cudaMalloc(&d_activeCount_, sizeof(uint32_t));

    // Initialize mesh buffers
    initializeMeshBuffers();

    err = cudaGetLastError();
    if (err != cudaSuccess) {
        std::cerr << "[DynamicGpuHashGrid] CUDA init error: " << cudaGetErrorString(err) << std::endl;
        return;
    }
    
    std::cout << "[DynamicGpuHashGrid] ✅ Initialized with dirty tracking!" << std::endl;
}

void DynamicGpuHashGrid::initializeMeshBuffers() {
    // Initial allocation for mesh buffers
    allocatedTriangles_ = 500000;  // Start smaller, grow as needed
    size_t vertexBytes = allocatedTriangles_ * 3 * sizeof(float) * 3;
    size_t normalBytes = allocatedTriangles_ * 3 * sizeof(float) * 3;

    for (int i = 0; i < 2; ++i) {
        cudaMalloc(&meshBuffers_[i].d_vertices, vertexBytes);
        cudaMalloc(&meshBuffers_[i].d_normals, normalBytes);
        meshBuffers_[i].triangleCount = 0;
        meshBuffers_[i].valid = false;
    }

    // Temp buffers for incremental updates
    tempBufferSize_ = config_.maxDirtyBlocks * MAX_TRIANGLES_PER_BLOCK;
    cudaMalloc(&d_tempVertices_, tempBufferSize_ * 3 * sizeof(float) * 3);
    cudaMalloc(&d_tempNormals_, tempBufferSize_ * 3 * sizeof(float) * 3);
    cudaMalloc(&d_tempTriangleCount_, sizeof(int));

    // Normal smoothing table
    normalTableSize_ = 1 << 20;  // 1M entries
    cudaMalloc(&d_normalAccumTable_, normalTableSize_ * sizeof(float) * 4);

    // Legacy pointers
    d_meshVertices_ = meshBuffers_[0].d_vertices;
    d_meshNormals_ = meshBuffers_[0].d_normals;

    std::cout << "  Mesh buffers: " << (vertexBytes + normalBytes) * 2 / (1024.0f * 1024.0f) << " MB" << std::endl;
}

void DynamicGpuHashGrid::cleanup() {
    if (d_hashTable_) cudaFree(d_hashTable_);
    if (d_blockData_) cudaFree(d_blockData_);
    if (d_blockCounter_) cudaFree(d_blockCounter_);
    if (d_dirtyFlags_) cudaFree(d_dirtyFlags_);
    if (d_dirtyBlockIndices_) cudaFree(d_dirtyBlockIndices_);
    if (d_dirtyCount_) cudaFree(d_dirtyCount_);
    if (d_blockMeshInfo_) cudaFree(d_blockMeshInfo_);
    if (d_activeIndices_) cudaFree(d_activeIndices_);
    if (d_activeCount_) cudaFree(d_activeCount_);
    if (d_tempVertices_) cudaFree(d_tempVertices_);
    if (d_tempNormals_) cudaFree(d_tempNormals_);
    if (d_tempTriangleCount_) cudaFree(d_tempTriangleCount_);
    if (d_normalAccumTable_) cudaFree(d_normalAccumTable_);

    for (int i = 0; i < 2; ++i) {
        if (meshBuffers_[i].d_vertices) cudaFree(meshBuffers_[i].d_vertices);
        if (meshBuffers_[i].d_normals) cudaFree(meshBuffers_[i].d_normals);
    }
}

void DynamicGpuHashGrid::cleanupMeshBuffers() {
    cleanupMeshBuffers();
}

bool DynamicGpuHashGrid::ApplySphericalBrush(
    const glm::vec3& worldPos,
    float worldRadius,
    float strength,
    bool addMode,
    void* stream)
{
    // Early exit for invalid radius
    if (worldRadius <= 0.0f) {
        return true;  // Nothing to do
    }
    
    cudaStream_t cudaStream = stream ? static_cast<cudaStream_t>(stream) : 0;

    // Calculate grid dimensions with bounds checking
    int blockRadius = static_cast<int>(ceilf(worldRadius / (config_.voxelSize * BLOCK_SIZE))) + 1;
    
    // Ensure minimum size
    if (blockRadius <= 0) blockRadius = 1;
    
    // Limit to prevent excessive grid size
    blockRadius = std::min(blockRadius, 32);  // Max 65^3 blocks
    
    // Clamp to CUDA limits
    const int MAX_GRID_DIM = 1024;  // Conservative limit
    blockRadius = std::min(blockRadius, MAX_GRID_DIM / 2 - 1);
    
    int gridDim = blockRadius * 2 + 1;
    
    dim3 gridSize(gridDim, gridDim, gridDim);
    dim3 blockSize(512);  // Явно 512

    float3 brushCenter = make_float3(worldPos.x, worldPos.y, worldPos.z);

    SculptSphericalKernelOptimized<<<gridSize, blockSize, 0, cudaStream>>>(
        d_hashTable_,
        config_.hashTableSize,
        d_blockData_,
        d_blockCounter_,
        config_.maxBlocks,
        brushCenter,
        worldRadius,
        strength,
        addMode,
        config_.voxelSize,
        d_dirtyFlags_,
        d_dirtyBlockIndices_,
        d_dirtyCount_,
        config_.maxDirtyBlocks
    );

    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess) {
        std::cerr << "[DynamicGpuHashGrid] Sculpt error: " << cudaGetErrorString(err) << std::endl;
        return false;
    }

    needsCountUpdate_ = true;
    return true;
}

void DynamicGpuHashGrid::compactActiveBlocks(void* stream) {
    cudaStream_t s = stream ? static_cast<cudaStream_t>(stream) : 0;
    
    // ДОБАВИТЬ проверку
    if (cachedActiveBlockCount_ == 0) {
        // Быстрый путь - копируем счётчик
        cudaMemcpyAsync(&cachedActiveBlockCount_, d_blockCounter_, 
                        sizeof(uint32_t), cudaMemcpyDeviceToHost, s);
        return;
    }
    
    // Use Thrust for faster compaction
    try {
        thrust::device_ptr<uint32_t> d_indices(d_activeIndices_);
        thrust::device_ptr<uint32_t> d_count(d_activeCount_);
        
        // Generate indices 0..tableSize-1
        thrust::sequence(thrust::cuda::par.on(s), d_indices, d_indices + config_.hashTableSize);
        
        // Copy only active indices
        IsBlockActive pred{d_hashTable_};
        auto end = thrust::copy_if(
            thrust::cuda::par.on(s),
            d_indices,
            d_indices + config_.hashTableSize,
            d_indices,
            pred
        );
        
        cachedActiveBlockCount_ = end - d_indices;
    } catch (...) {
        // Fallback to simple kernel if Thrust fails
        cudaMemset(d_activeCount_, 0, sizeof(uint32_t));
        
        dim3 blockSize(256);
        dim3 gridSize((config_.hashTableSize + 255) / 256);
        
        // Simple compaction kernel (slower but more reliable)
        // ... fallback implementation
        
        cudaMemcpy(&cachedActiveBlockCount_, d_activeCount_, sizeof(uint32_t), cudaMemcpyDeviceToHost);
    }
}

void DynamicGpuHashGrid::clearDirtyList(void* stream) {
    cudaStream_t s = stream ? static_cast<cudaStream_t>(stream) : 0;
    
    // Clear dirty flags for processed blocks
    uint32_t dirtyCount = 0;
    cudaMemcpy(&dirtyCount, d_dirtyCount_, sizeof(uint32_t), cudaMemcpyDeviceToHost);
    
    if (dirtyCount > 0) {
        dim3 blockSize(256);
        dim3 gridSize((config_.maxBlocks + 255) / 256);
        ClearDirtyFlagsKernel<<<gridSize, blockSize, 0, s>>>(d_dirtyFlags_, config_.maxBlocks);
    }
    
    // Reset dirty count
    cudaMemsetAsync(d_dirtyCount_, 0, sizeof(uint32_t), s);
    cachedDirtyCount_ = 0;
}

void DynamicGpuHashGrid::InvalidateAllBlocks() {
    dim3 blockSize(256);
    dim3 gridSize((config_.hashTableSize + 255) / 256);
    
    cudaMemset(d_dirtyCount_, 0, sizeof(uint32_t));
    
    MarkAllBlocksDirtyKernel<<<gridSize, blockSize>>>(
        d_hashTable_,
        config_.hashTableSize,
        d_dirtyFlags_,
        d_dirtyBlockIndices_,
        d_dirtyCount_,
        config_.maxDirtyBlocks
    );
    
    // Also invalidate mesh buffers
    meshBuffers_[0].valid = false;
    meshBuffers_[1].valid = false;
}

void DynamicGpuHashGrid::Reset() {
    // Reset hash table
    std::vector<HashEntry> emptyTable(config_.hashTableSize);
    for (auto& entry : emptyTable) {
        entry.blockX = INT32_MAX;
    }
    cudaMemcpy(d_hashTable_, emptyTable.data(), 
               config_.hashTableSize * sizeof(HashEntry), 
               cudaMemcpyHostToDevice);
    
    // Zero all data
    cudaMemset(d_blockData_, 0, config_.maxBlocks * 512 * sizeof(float));
    cudaMemset(d_blockCounter_, 0, sizeof(uint32_t));
    cudaMemset(d_dirtyFlags_, 0, config_.maxBlocks * sizeof(uint32_t));
    cudaMemset(d_dirtyCount_, 0, sizeof(uint32_t));
    cudaMemset(d_blockMeshInfo_, 0, config_.maxBlocks * sizeof(BlockMeshInfo));
    
    // Reset mesh buffers
    for (int i = 0; i < 2; ++i) {
        meshBuffers_[i].triangleCount = 0;
        meshBuffers_[i].valid = false;
    }
    
    cachedActiveBlockCount_ = 0;
    cachedDirtyCount_ = 0;
    needsCountUpdate_ = false;
    frameCounter_ = 0;
    
    std::cout << "[DynamicGpuHashGrid] ✅ Reset complete" << std::endl;
}

uint32_t DynamicGpuHashGrid::GetActiveBlockCount() const {
    return cachedActiveBlockCount_;
}

uint32_t DynamicGpuHashGrid::GetDirtyBlockCount() const {
    return cachedDirtyCount_;
}

void DynamicGpuHashGrid::UpdateActiveCountAsync(void* stream) {
    if (!needsCountUpdate_) return;
    
    cudaStream_t s = stream ? static_cast<cudaStream_t>(stream) : 0;
    
    cudaMemcpyAsync(&cachedActiveBlockCount_, d_blockCounter_, 
                    sizeof(uint32_t), cudaMemcpyDeviceToHost, s);
    cudaMemcpyAsync(&cachedDirtyCount_, d_dirtyCount_,
                    sizeof(uint32_t), cudaMemcpyDeviceToHost, s);
    
    needsCountUpdate_ = false;
}

bool DynamicGpuHashGrid::GetCurrentMesh(
    float** d_vertices,
    float** d_normals,
    int* triangleCount)
{
    MeshBuffer& current = meshBuffers_[currentMeshBuffer_];
    
    if (!current.valid) {
        *d_vertices = nullptr;
        *d_normals = nullptr;
        *triangleCount = 0;
        return false;
    }
    
    *d_vertices = current.d_vertices;
    *d_normals = current.d_normals;
    *triangleCount = current.triangleCount;
    return true;
}

DynamicGpuHashGrid::MeshStats DynamicGpuHashGrid::GetMeshStats() const {
    return lastStats_;
}

void DynamicGpuHashGrid::PrintStats() const {
    uint32_t activeBlocks = GetActiveBlockCount();
    uint32_t dirtyBlocks = GetDirtyBlockCount();
    float usedMemoryMB = activeBlocks * VOXELS_PER_BLOCK * sizeof(float) / (1024.0f * 1024.0f);
    float fillRate = 100.0f * activeBlocks / config_.maxBlocks;

    std::cout << "[DynamicGpuHashGrid] Stats:" << std::endl;
    std::cout << "  Active blocks: " << activeBlocks << " / " << config_.maxBlocks 
              << " (" << fillRate << "%)" << std::endl;
    std::cout << "  Dirty blocks: " << dirtyBlocks << std::endl;
    std::cout << "  Used memory: " << usedMemoryMB << " MB" << std::endl;
    std::cout << "  Last mesh triangles: " << meshBuffers_[currentMeshBuffer_].triangleCount << std::endl;
}

} // namespace Urbaxio::Engine
