#include "engine/gpu/DynamicGpuHashGrid.h"
#include <cuda_runtime.h>
#include <iostream>
#include <cmath>
#include <chrono>

namespace Urbaxio::Engine {

// ============================================================================
// DEVICE FUNCTIONS (GPU side)
// ============================================================================

__device__ inline uint32_t hash3D(int32_t x, int32_t y, int32_t z, uint32_t tableSize) {
    // Simple hash function
    uint32_t h = x * 73856093u ^ y * 19349663u ^ z * 83492791u;
    return h & (tableSize - 1); // Assumes tableSize is power of 2
}

__device__ inline int32_t voxelToBlock(float coord, float voxelSize) {
    return static_cast<int32_t>(floorf(coord / (voxelSize * DynamicGpuHashGrid::BLOCK_SIZE)));
}

__device__ inline int3 voxelToBlockCoord(float3 worldPos, float voxelSize) {
    return make_int3(
        voxelToBlock(worldPos.x, voxelSize),
        voxelToBlock(worldPos.y, voxelSize),
        voxelToBlock(worldPos.z, voxelSize)
    );
}

// Find or allocate block in hash table
// Returns dataIndex (UINT32_MAX if allocation failed)
__device__ uint32_t findOrAllocateBlock(
    DynamicGpuHashGrid::HashEntry* hashTable,
    uint32_t tableSize,
    int32_t blockX, int32_t blockY, int32_t blockZ,
    uint32_t* blockCounter,
    uint32_t maxBlocks,
    float* blockData)
{
    uint32_t slot = hash3D(blockX, blockY, blockZ, tableSize);

    // Linear probing
    for (uint32_t probe = 0; probe < 100; ++probe) {
        uint32_t currentSlot = (slot + probe) & (tableSize - 1);

        // Try to claim this slot
        int32_t oldX = atomicCAS(&hashTable[currentSlot].blockX, INT32_MAX, blockX);

        if (oldX == INT32_MAX) {
            hashTable[currentSlot].blockY = blockY;
            hashTable[currentSlot].blockZ = blockZ;

            // Allocate block data
            uint32_t dataIdx = atomicAdd(blockCounter, 1);

            if (dataIdx >= maxBlocks) {
                hashTable[currentSlot].blockX = INT32_MAX;
                return UINT32_MAX;
            }

            hashTable[currentSlot].dataIndex = dataIdx;

            // Initialize block to 1.0f (background)
            float* block = blockData + dataIdx * 512;
            for (int i = threadIdx.x; i < 512; i += blockDim.x) {
                block[i] = 1.0f;
            }
            __syncthreads();

            return dataIdx;
        }

        // Check if this slot has our block
        if (hashTable[currentSlot].blockX == blockX &&
            hashTable[currentSlot].blockY == blockY &&
            hashTable[currentSlot].blockZ == blockZ) {
            return hashTable[currentSlot].dataIndex;
        }
    }

    // Hash table collision limit reached
    return UINT32_MAX;
}

// Sculpt kernel - processes one voxel per thread
__global__ void SculptSphericalKernel(
    DynamicGpuHashGrid::HashEntry* hashTable,
    uint32_t tableSize,
    float* blockData,
    uint32_t* blockCounter,
    uint32_t maxBlocks,
    float3 brushCenter,
    float brushRadius,
    float strength,
    bool addMode,
    float voxelSize)
{
    // Each block processes one potential block coordinate
    // Use floorf for consistent coordinate system with meshing
    int32_t centerBlockX = static_cast<int32_t>(floorf(brushCenter.x / (voxelSize * DynamicGpuHashGrid::BLOCK_SIZE)));
    int32_t centerBlockY = static_cast<int32_t>(floorf(brushCenter.y / (voxelSize * DynamicGpuHashGrid::BLOCK_SIZE)));
    int32_t centerBlockZ = static_cast<int32_t>(floorf(brushCenter.z / (voxelSize * DynamicGpuHashGrid::BLOCK_SIZE)));
    
    int32_t blockX = centerBlockX + blockIdx.x - gridDim.x / 2;
    int32_t blockY = centerBlockY + blockIdx.y - gridDim.y / 2;
    int32_t blockZ = centerBlockZ + blockIdx.z - gridDim.z / 2;

    // Find or allocate this block
    uint32_t dataIdx = findOrAllocateBlock(
        hashTable, tableSize,
        blockX, blockY, blockZ,
        blockCounter, maxBlocks, blockData
    );

    if (dataIdx == UINT32_MAX) return; // Allocation failed

    // Process ALL voxels in block
    int voxelIdx = threadIdx.x;
    if (voxelIdx >= DynamicGpuHashGrid::VOXELS_PER_BLOCK) return;

    int localX = voxelIdx % 8;
    int localY = (voxelIdx / 8) % 8;
    int localZ = voxelIdx / 64;

    float3 voxelWorld = make_float3(
        (blockX * 8 + localX) * voxelSize,
        (blockY * 8 + localY) * voxelSize,
        (blockZ * 8 + localZ) * voxelSize
    );

    float dx = voxelWorld.x - brushCenter.x;
    float dy = voxelWorld.y - brushCenter.y;
    float dz = voxelWorld.z - brushCenter.z;
    float dist = sqrtf(dx*dx + dy*dy + dz*dz);

    // SIMPLE: Direct SDF, no smoothing
    float brushSDF = dist - brushRadius;

    float* block = blockData + dataIdx * 512;
    float currentVal = block[voxelIdx];

    // SIMPLE: Direct min (CSG union)
    float newVal = fminf(currentVal, brushSDF);

    // WIDER influence, SIMPLER falloff
    float maxDist = brushRadius * 4.0f; // 4x radius instead of 8x
    if (dist > maxDist) return; // Skip far voxels for performance

    float t = dist / maxDist;
    t = fmaxf(0.0f, fminf(1.0f, t));
    
    // Simple linear falloff
    float falloff = 1.0f - t;
    
    // Apply with strength
    float blended = currentVal + (newVal - currentVal) * strength * falloff;

    block[voxelIdx] = blended;
}

// ============================================================================
// HOST FUNCTIONS (CPU side)
// ============================================================================

DynamicGpuHashGrid::DynamicGpuHashGrid(const Config& config)
    : config_(config)
    , d_hashTable_(nullptr)
    , d_blockData_(nullptr)
    , d_blockCounter_(nullptr)
    , activeBlockCount_(0)
{
    initialize();
}

DynamicGpuHashGrid::~DynamicGpuHashGrid() {
    cleanup();
}

void DynamicGpuHashGrid::initialize() {
    std::cout << "[DynamicGpuHashGrid] Initializing..." << std::endl;
    
    size_t freeMemory, totalMemory;
    cudaMemGetInfo(&freeMemory, &totalMemory);
    float freeGB = freeMemory / (1024.0f * 1024.0f * 1024.0f);
    
    std::cout << "  Available GPU memory: " << freeGB << " GB" << std::endl;
    
    if (freeGB < 2.0f) {
        std::cout << "[DynamicGpuHashGrid] ⚠️ Low GPU memory (" << freeGB 
                  << " GB), reducing allocation..." << std::endl;
        config_.maxBlocks = std::min(config_.maxBlocks, 100u * 1024u);
        config_.hashTableSize = std::min(config_.hashTableSize, 256u * 1024u);
    }
    
    std::cout << "  Max blocks: " << config_.maxBlocks << std::endl;
    std::cout << "  Hash table size: " << config_.hashTableSize << std::endl;
    std::cout << "  Voxel size: " << config_.voxelSize << "m" << std::endl;

    size_t hashTableBytes = config_.hashTableSize * sizeof(HashEntry);
    size_t blockDataBytes = config_.maxBlocks * VOXELS_PER_BLOCK * sizeof(float);

    std::cout << "  Memory required: " << (hashTableBytes + blockDataBytes) / (1024.0f * 1024.0f) << " MB" << std::endl;

    // Allocate hash table
    cudaMalloc(&d_hashTable_, hashTableBytes);

    // Initialize hash table to empty (blockX = INT32_MAX means empty)
    std::vector<HashEntry> emptyTable(config_.hashTableSize);
    for (auto& entry : emptyTable) {
        entry.blockX = INT32_MAX;
        entry.blockY = 0;
        entry.blockZ = 0;
        entry.dataIndex = 0;
    }
    cudaMemcpy(d_hashTable_, emptyTable.data(), hashTableBytes, cudaMemcpyHostToDevice);

    // Allocate block data
    cudaMalloc(&d_blockData_, blockDataBytes);
    
    // FIX: ZERO entire memory!
    cudaMemset(d_blockData_, 0, blockDataBytes);

    // Allocate block counter
    cudaMalloc(&d_blockCounter_, sizeof(uint32_t));
    cudaMemset(d_blockCounter_, 0, sizeof(uint32_t));
    
    std::cout << "[DynamicGpuHashGrid] Memory zeroed: " 
              << (blockDataBytes / 1024.0 / 1024.0) << " MB" << std::endl;

    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess) {
        std::cerr << "[DynamicGpuHashGrid] CUDA error: " << cudaGetErrorString(err) << std::endl;
        return;
    }
    
    std::cout << "[DynamicGpuHashGrid] ✅ Initialized successfully!" << std::endl;
}

void DynamicGpuHashGrid::cleanup() {
    if (d_hashTable_) cudaFree(d_hashTable_);
    if (d_blockData_) cudaFree(d_blockData_);
    if (d_blockCounter_) cudaFree(d_blockCounter_);
    if (d_meshVertices_) cudaFree(d_meshVertices_);
    if (d_meshNormals_) cudaFree(d_meshNormals_);
}

void DynamicGpuHashGrid::Reset() {
    // Reset hash table
    std::vector<HashEntry> emptyTable(config_.hashTableSize);
    for (auto& entry : emptyTable) {
        entry.blockX = INT32_MAX;
        entry.blockY = 0;
        entry.blockZ = 0;
        entry.dataIndex = 0;
    }
    cudaMemcpy(d_hashTable_, emptyTable.data(), 
               config_.hashTableSize * sizeof(HashEntry), 
               cudaMemcpyHostToDevice);
    
    // Zero block data
    cudaMemset(d_blockData_, 0, config_.maxBlocks * 512 * sizeof(float));
    
    // Reset counter
    cudaMemset(d_blockCounter_, 0, sizeof(uint32_t));
    
    // Reset mesh buffers
    if (d_meshVertices_ && allocatedTriangles_ > 0) {
        cudaMemset(d_meshVertices_, 0, allocatedTriangles_ * 9 * sizeof(float3));
    }
    if (d_meshNormals_ && allocatedTriangles_ > 0) {
        cudaMemset(d_meshNormals_, 0, allocatedTriangles_ * 9 * sizeof(float3));
    }
    
    activeBlockCount_ = 0;
    cachedActiveBlockCount_ = 0;
    needsCountUpdate_ = false;
    
    std::cout << "[DynamicGpuHashGrid] ✅ Reset complete" << std::endl;
}

bool DynamicGpuHashGrid::ApplySphericalBrush(
    const glm::vec3& worldPos,
    float worldRadius,
    float strength,
    bool addMode,
    void* stream)
{
    // Calculate affected blocks
    int blockRadius = static_cast<int>(ceilf(worldRadius / (config_.voxelSize * BLOCK_SIZE))) + 1;

    dim3 gridSize(blockRadius * 2 + 1, blockRadius * 2 + 1, blockRadius * 2 + 1);
    dim3 blockSize(VOXELS_PER_BLOCK);

    float3 brushCenter = make_float3(worldPos.x, worldPos.y, worldPos.z);

    cudaStream_t cudaStream = stream ? static_cast<cudaStream_t>(stream) : 0;

    SculptSphericalKernel<<<gridSize, blockSize, 0, cudaStream>>>(
        d_hashTable_,
        config_.hashTableSize,
        d_blockData_,
        d_blockCounter_,
        config_.maxBlocks,
        brushCenter,
        worldRadius,
        strength,
        addMode,
        config_.voxelSize
    );

    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess) {
        std::cerr << "[DynamicGpuHashGrid] Sculpt kernel error: " << cudaGetErrorString(err) << std::endl;
        return false;
    }
    
    // ✅ Mark count as dirty (don't fetch yet!)
    needsCountUpdate_ = true;

    return true;
}

uint32_t DynamicGpuHashGrid::GetActiveBlockCount() const {
    // ✅ Return cached value (update ASYNC in background)
    return cachedActiveBlockCount_;
}

void DynamicGpuHashGrid::UpdateActiveCountAsync(void* stream) {
    if (!needsCountUpdate_) return;
    
    cudaStream_t cudaStream = stream ? static_cast<cudaStream_t>(stream) : 0;
    
    // Non-blocking copy
    cudaMemcpyAsync(&cachedActiveBlockCount_, d_blockCounter_, 
                    sizeof(uint32_t), cudaMemcpyDeviceToHost, cudaStream);
    
    needsCountUpdate_ = false;
}

void DynamicGpuHashGrid::PrintStats() const {
    uint32_t activeBlocks = GetActiveBlockCount();
    float usedMemoryMB = activeBlocks * VOXELS_PER_BLOCK * sizeof(float) / (1024.0f * 1024.0f);
    float fillRate = 100.0f * activeBlocks / config_.maxBlocks;

    std::cout << "[DynamicGpuHashGrid] Stats:" << std::endl;
    std::cout << "  Active blocks: " << activeBlocks << " / " << config_.maxBlocks 
              << " (" << fillRate << "%)" << std::endl;
    std::cout << "  Used memory: " << usedMemoryMB << " MB" << std::endl;
}

// Forward declaration - implementation in HashGridMeshing.cu
// (ExtractMesh is defined in a separate file)

} // namespace Urbaxio::Engine


